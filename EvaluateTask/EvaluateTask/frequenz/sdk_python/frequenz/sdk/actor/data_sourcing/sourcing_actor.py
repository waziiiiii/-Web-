# License: MIT
# Copyright 2022 Frequenz Energy-as-a-Service GmbH
#
# Docs: https://frequenz-floss.github.io/frequenz-docs/usage/setup

"""Actor to query and process timeseries data from source components."""

from __future__ import annotations

import asyncio
import logging
from datetime import datetime, timedelta
from typing import Any

from ..._internal._asyncio_runner import run_functions
from ..._internal._channels import ChannelReceiverOrClosing
from ..._internal._math import isclose
from ..._internal._timeseries._base_store import VALUE, BaseStore
from ..._internal._timeseries._sampler import Sampler, UnixDatetime
from ...timeseries import Sample
from .._base_actor import Actor
from .sourcing_error import SourcingError
from .sourcing_request import SourcingRequest

# we need multiple inheritance since the `_ChannelOperation` class (i.e.
# `Formula`) needs to be closed too, when the actor should stop.
# https://app.asana.com/0/1204113654663915/1204213704267547/f


_logger = logging.getLogger(__name__)

# add handlers to logger
# ensures that log messages from this actor are not affected by log
# settings from other actors
_channel_log = logging.getLogger(f"{__name__}.channel")

# Use a default time threshold of 0 (zero) if no threshold is provided to
# `FormulaSampler`. This will only yield the result if it is different from
# previous results.
_DEFAULT_DECIMALS = 6
_DEFAULT_METRIC_BUFFER_SIZE = 100


class SourcingActor(Actor):  # pylint: disable=too-many-instance-attributes
    """This actor streams and processes metrics data from source components.

    It provides a local store, where all received samples are kept.

    This actor is useful as a base class for more advanced actors and can be
    extended to provide additional data sources, by providing them with
    multiple receivers.
    """

    def __init__(  # pylint: disable=too-many-arguments
        self,
        receivers: dict[str, ChannelReceiverOrClosing[Sample]],
        stores: dict[str, BaseStore],
        *,
        interval: timedelta = timedelta(seconds=1),
        **kwargs: Any,
    ) -> None:
        """Create a SourcingActor instance.

        Note: all source component channels will not be used or accepted as
        long as their return type does not match the return types of the
        given stores.

        Args:
            receivers: A list of component or formula channels.
            stores: A list of data stores with a store for each source.
            interval: Time interval for meter data processing. Defaults to 1s.
            **kwargs: keyword arguments for the actor creation.
                Accepts ``taskgroup`` as keyword argument for the taskgroup
                this actor will run in.
        """
        super().__init__(**kwargs)

        # each `_ChannelOperation` needs its own `_ChannelOperation`
        self._receivers: dict[str, ChannelReceiverOrClosing[Sample]] = receivers
        self._stores: dict[str, BaseStore] = stores
        self._interval = interval
        self._sourcing_events: list[tuple[SourcingRequest, list[bool]]] = []

        self._samplers: dict[str, Sampler] = {
            k: Sampler(
                name=k,
                store=v,
            )
            for k, v in self._stores.items()
        }

        self._taskgroup: asyncio.TaskGroup = kwargs.get("taskgroup", asyncio.TaskGroup())
        # just a tmp value, set in `_run()`, to allow stopping this actor
        # before it has started
        self._stopped: asyncio.Event = asyncio.Event()

    @property
    def sampling_interval(self) -> timedelta:
        """The interval to sample the next values.

        Returns:
            The sampling interval.
        """
        return self._interval

    @sampling_interval.setter
    def sampling_interval(self, interval: timedelta) -> None:
        """Set the sampling interval.

        Args:
            interval: The interval to use when sampling the next metric
                values.
        """
        self._interval = interval

    def _timestamp_with_resampling_alignment(
        self, timestamp: UnixDatetime
    ) -> UnixDatetime:
        """Align the timestamp with the resampling interval.

        Alignes the given timestamp with the resampling interval. For this,
        the *starting* timestamp of the latest closed bucket is used.

        Note: The resampling alignment only works if an
        ``integration_period`` is set in the stores.

        Args:
            timestamp: A unix datetime as a `datetime.datetime`.

        Returns:
            The aligned timestamp.

        Raises:
            SourcingError: on a resampling interval alignment error.
        """
        assert len(self._stores) > 0, "No metric stores set!"
        store = next(iter(self._stores.values()))

        bucket = store.bucket_interval

        if bucket.total_seconds() == 0:
            raise SourcingError(
                "Invalid bucket size for resampling: {bucket}."
                "Please make sure that the bucket size is > 0."
            )
        return store.latest_closed_bucket(timestamp)

    async def _collect(  # pylint: disable=too-many-locals,too-many-branches
        self,
        resampling_period: timedelta | None = None,
        max_alignment_delay: timedelta | None = None,
        component_metrics: list[str] | None = None,
    ) -> dict[str, list[Sample]]:
        """Collect data from the metric stores.

        This method retrieves the metric data from the stores.

        Note:
            After collecting the data, the internal stores are not modified. It
            is the caller's responsibility to call `reset()` or `delete()`
            on the stores after collecting the data.

        Args:
            resampling_period: An optional resampling period. If set to
                ``None`` the ``integration_period`` of the metric stores is
                used for resampling. Defaults to ``None``.
            max_alignment_delay: An optional delay for alignment. Defaults
                to ``None``.
            component_metrics: An optional list of component metrics to
                collect. Defaults to ``None``.

        Returns:
            A dictionary with the metrics and their values.

        Raises:
            SourcingError: If there are errors during data collection.
        """
        collected: dict[str, list[Sample]] = {}

        for metric, store in self._stores.items():
            if component_metrics is not None and metric not in component_metrics:
                continue

            if store.first_timestamp is None:
                raise SourcingError(
                    f"Not enough data to process metric '{metric}'."
                )

            # Use the store's integration period or the given resampling period
            bucket_size = (
                store.bucket_interval
                if resampling_period is None
                else resampling_period
            )

            if bucket_size.total_seconds() == 0:
                raise SourcingError(
                    f"Invalid resampling period: {bucket_size}."
                    "Please make sure that the resampling period > 0."
                )

            # get the latest closed bucket
            aligned_timestamp = store.latest_closed_bucket(store.last_timestamp)

            # retrieve data for each bucket and add it to the
            # collected data
            bucket_start = store.first_timestamp
            bucket_end = bucket_start + bucket_size

            values: list[Sample] = []
            while bucket_start < aligned_timestamp:
                # check if the bucket is within the alignment delay
                if max_alignment_delay is not None and (
                    aligned_timestamp - bucket_end > max_alignment_delay
                ):
                    bucket_start = bucket_end
                    bucket_end = bucket_start + bucket_size
                    continue

                bucket_value = store.fetch(bucket_start, bucket_end)

                if bucket_value is not None:
                    values.append(
                        Sample(
                            timestamp=bucket_start,
                            value=bucket_value,
                        )
                    )

                bucket_start = bucket_end
                bucket_end = bucket_start + bucket_size

            collected[metric] = values

        return collected

    def _align_timestamp(  # pylint: disable=too-many-branches
        self,
        timestamp: UnixDatetime,
        threshold: timedelta | None = None,
    ) -> tuple[UnixDatetime, bool]:
        """Align a timestamp with the metric data already stored.

        This method aligns the given timestamp to the existing bucket
        start timestamps in the stores. It tries to find a matching bucket
        or the closest bucket within a given threshold.

        Args:
            timestamp: The timestamp to align.
            threshold: An optional timedelta threshold. Defaults to ``None``.

        Returns:
            A tuple of the aligned timestamp and a boolean indicating
            whether the timestamp was already aligned.
        """
        if threshold is None:
            # no threshold set, use the one from the store, but as the
            # integration period is optional, check if it is set
            for store in self._stores.values():
                if store.bucket_interval.total_seconds() > 0:
                    threshold = store.bucket_interval
                    break
            else:
                # no threshold set and no integration period set, so just
                # return the timestamp as is
                return timestamp, True

        aligned_timestamp = timestamp
        store_found = False
        for _, store in self._stores.items():
            # get the latest closed bucket
            aligned_timestamp = store.latest_closed_bucket(timestamp)
            store_found = True

            if aligned_timestamp == timestamp:
                # exact match, no need to check further
                break

            # check if the aligned timestamp is within the threshold
            if abs(timestamp - aligned_timestamp) <= threshold:
                # use the aligned timestamp
                timestamp = aligned_timestamp
                store_found = True
                break

        return aligned_timestamp, store_found

    def _maybe_align_timestamp(
        self,
        timestamp: UnixDatetime,
        metric: str,
        threshold: timedelta | None,
        align: bool,
    ) -> UnixDatetime:
        """Align a timestamp to an existing bucket or not, depending on ``align``.

        Args:
            timestamp: The timestamp to align.
            metric: The metric to align the timestamp to.
            threshold: The threshold for alignment.
            align: Whether to align the timestamp or not.

        Returns:
            The aligned timestamp.
        """
        if align:
            aligned_timestamp, found = self._align_timestamp(
                timestamp, threshold
            )
            if not found:
                _channel_log.warning(
                    "Couldn't align timestamp '%s' for metric '%s'.",
                    timestamp,
                    metric,
                )
            timestamp = aligned_timestamp
        return timestamp

    def source(  # pylint: disable=too-many-locals,too-many-branches,too-many-statements
        self,
        metric: str,
        *,
        component_id: int | None = None,
        req_start: datetime | None = None,
        resampling_period: timedelta | None = None,
        max_alignment_delay: timedelta | None = None,
        threshold: timedelta | None = None,
    ) -> list[Sample]:
        """Synchronize the metric and return its data.

        This is a synchronized version of `source_next`, it is also
        possible to set a ``req_start`` to fetch historical data from the
        stores.

        Args:
            metric: A component metric.
            component_id: A component id. Defaults to ``None``.
            req_start: A ``datetime`` of the requested start. Defaults to ``None``.
            resampling_period: A ``timedelta`` of the resampling period.
                Defaults to ``None``.
            max_alignment_delay: A ``timedelta`` of the maximum alignment
                delay. Defaults to ``None``.
            threshold: A ``timedelta`` threshold for the metric data.
                Defaults to ``None``.

        Raises:
            SourcingError: If the metric doesn't have a store.

        Returns:
            A list of samples.
        """
        if metric not in self._stores:
            raise SourcingError(f"The metric '{metric}' doesn't have a store.")

        store = self._stores[metric]

        now = datetime.now()

        values: list[Sample] = []
        if req_start is None:
            sample = store.fetch(store.first_timestamp, now)
            if sample is not None:
                values.append(
                    Sample(
                        timestamp=now,
                        value=sample,
                    )
                )
        else:
            bucket_size = (
                store.bucket_interval
                if resampling_period is None
                else resampling_period
            )

            bucket_start = req_start
            bucket_end = bucket_start + bucket_size

            aligned_now = self._maybe_align_timestamp(
                timestamp=now,
                metric=metric,
                threshold=threshold,
                align=threshold is not None,
            )

            while bucket_start < aligned_now:
                bucket_value = store.fetch(bucket_start, bucket_end)

                if bucket_value is not None:
                    values.append(
                        Sample(
                            timestamp=bucket_start,
                            value=bucket_value,
                        )
                    )

                bucket_start = bucket_end
                bucket_end = bucket_start + bucket_size

        return values

    def _update_store(  # pylint: disable=too-many-branches
        self,
        *,
        channel_name: str,
        sample: Sample,
        metric: str,
        component_id: int | None = None,
        num_decimals: int = _DEFAULT_DECIMALS,
        align: bool = False,
        threshold: timedelta | None = None,
    ) -> None:
        """Update the store with the given sample.

        This method updates the store with the given sample. The given
        timestamp will be aligned to the store's bucket interval if the
        ``align`` flag is set.

        Args:
            channel_name: The name of the channel.
            sample: The sample to update the store with.
            metric: The metric to update the store with.
            component_id: The component id. Defaults to ``None``.
            num_decimals: The number of decimals to use for the value.
                Defaults to ``_DEFAULT_DECIMALS``.
            align: Whether to align the timestamp. Defaults to ``False``.
            threshold: The threshold for alignment. Defaults to ``None``.
        """
        timestamp = sample.timestamp
        if align:
            timestamp = self._maybe_align_timestamp(
                timestamp=timestamp,
                metric=metric,
                threshold=threshold,
                align=align,
            )

        value = sample.value

        # check if the metric store exists
        if metric not in self._stores:
            raise SourcingError(
                f"The metric '{metric}' doesn't have a store."
            )

        store = self._stores[metric]
        store.add(timestamp, value)

        _channel_log.debug(
            "Metric '%s' with value '%s' added to the store.",
            metric,
            value,
        )

    def _process(self) -> None:
        """Process the received data from the channels.

        This method processes the received data from the channels, updates
        the stores, and fires any waiting events.
        """
        for metric, sampler in self._samplers.items():
            sample = sampler.sample()
            if sample is not None:
                self._update_store(
                    channel_name=metric,
                    sample=sample,
                    metric=metric,
                    align=False,
                    threshold=None,
                )

        # Check and fire sourcing events
        self._check_sourcing_events()

    def _check_sourcing_events(self) -> None:
        """Check and fire sourcing events based on the current state."""
        resolved_events = []
        for event, checks in self._sourcing_events:
            all_met = all(
                metric in self._stores
                and self._stores[metric].count(event.start, event.end) > 0
                for metric in event.metrics
            )
            if all_met:
                event.event.set()
                resolved_events.append((event, checks))

        for event in resolved_events:
            self._sourcing_events.remove(event)

    async def _run(self) -> None:  # pylint: disable=too-many-branches
        """Run the sourcing actor.

        Raises:
            SourcingError: If the sourcing actor could not be started.
        """
        if len(self._receivers) == 0:
            raise SourcingError("No metric receivers found!")

        if len(self._stores) == 0:
            raise SourcingError("No metric stores found!")

        if self._interval.total_seconds() <= 0:
            raise SourcingError(
                f"Invalid sampling interval: {self._interval}."
                "Please make sure that the sampling interval > 0."
            )

        # collect and process channel data in the configured interval
        self._stopped = asyncio.Event()
        try:
            async with self._taskgroup as taskgroup:
                # create a task for each metric
                for metric in self._receivers:
                    taskgroup.create_task(
                        self._receive(metric)
                    )

                # create a task to process the data
                taskgroup.create_task(
                    self._process_data()
                )
        except asyncio.CancelledError:
            pass
        finally:
            for _, sampler in self._samplers.items():
                sampler.close()
            for _, receiver in self._receivers.items():
                receiver.close()

    async def _receive(self, metric: str) -> None:
        """Receive data from a channel.

        Args:
            metric: The metric to receive data for.
        """
        receiver = self._receivers[metric]
        try:
            async for sample in receiver:
                self._samplers[metric].add(sample)
        except asyncio.CancelledError:
            pass

    async def _process_data(self) -> None:
        """Process the data in the configured interval."""
        while not self._stopped.is_set():
            self._process()
            await asyncio.sleep(self._interval.total_seconds())

    async def stop(self) -> None:
        """Stop the sourcing actor."""
        self._stopped.set()
        await super().stop()

    def append(  # pylint: disable=too-many-branches
        self,
        metric: str,
        *,
        component_id: int | None = None,
        receiver: ChannelReceiverOrClosing[Sample],
        store: BaseStore,
    ) -> None:
        """Append new channels from other actors to the existing store.

        Args:
            metric: The metric to append the channel for.
            component_id: The component id. Defaults to ``None``.
            receiver: The channel to get data from.
            store: The store to save the data into.
        """
        if metric in self._stores:
            _channel_log.warning(
                "The metric '%s' has already a store assigned.",
                metric,
            )
            return

        self._stores[metric] = store

        # add the new store also to the receivers
        self._receivers[metric] = receiver
        self._samplers[metric] = Sampler(
            name=metric,
            store=store,
        )

    def load(  # pylint: disable=too-many-branches
        self,
        sourcing_request: SourcingRequest,
    ) -> bool:
        """Load a sourcing request.

        This method loads a sourcing request into the sourcing actor. It
        creates stores and channels for the metrics defined in the request.

        Args:
            sourcing_request: The sourcing request to load.

        Returns:
            True if the sourcing request was loaded successfully.
        """
        for metric in sourcing_request.metrics:
            if metric not in self._stores:
                _channel_log.warning(
                    "The metric '%s' doesn't have a store. Creating a default store.",
                    metric,
                )

                store = self._stores[metric]

                store.add(sourcing_request.start, 0)
                store.add(sourcing_request.end, 0)

                _channel_log.debug(
                    "Store with id '%s' created.",
                    metric,
                )

        return True

    def execute(self, *_: Any, **__: Any) -> None:
        """Execute the sourcing actor.

        This method executes the sourcing actor. It is a no-op, as the
        sourcing actor is executed by the actor framework.

        Args:
            _: Unused positional arguments.
            __: Unused keyword arguments.
        """
        return

    def _validate_score_request(
        self,
        metric: str,
        *,
        start: UnixDatetime,
        end: UnixDatetime,
        merge_method: str | None = None,
        resampling_period: timedelta | None = None,
        max_alignment_delay: timedelta | None = None,
        num_decimals: int | None = None,
        threshold: timedelta | None = None,
    ) -> None:
        """Validate a scoring request.

        Args:
            metric: The metric to score.
            start: The start datetime.
            end: The end datetime.
            merge_method: The merge method.
            resampling_period: The resampling period.
            max_alignment_delay: The max alignment delay.
            num_decimals: The number of decimals.
            threshold: The threshold.
        """
        if metric not in self._stores:
            raise SourcingError(
                f"The metric '{metric}' doesn't have a store."
            )

        store = self._stores[metric]

        if store.count(start, end) == 0:
            raise SourcingError(
                f"Not enough data to score metric '{metric}' "
                f"in range {start} - {end}."
            )

    def score(  # pylint: disable=too-many-branches
        self,
        metric: str,
        *,
        start: UnixDatetime,
        end: UnixDatetime,
        merge_method: str | None = None,
        resampling_period: timedelta | None = None,
        max_alignment_delay: timedelta | None = None,
        num_decimals: int | None = None,
        threshold: timedelta | None = None,
    ) -> VALUE:
        """Score the given metric.

        Args:
            metric: The metric to score.
            start: The start datetime.
            end: The end datetime.
            merge_method: The merge method.
            resampling_period: The resampling period.
            max_alignment_delay: The max alignment delay.
            num_decimals: The number of decimals.
            threshold: The threshold.

        Returns:
            The scored value.
        """
        self._validate_score_request(
            metric,
            start=start,
            end=end,
            merge_method=merge_method,
            resampling_period=resampling_period,
            max_alignment_delay=max_alignment_delay,
            num_decimals=num_decimals,
            threshold=threshold,
        )

        store = self._stores[metric]

        value = store.fetch(start, end)
        if value is None:
            raise SourcingError(
                f"Not enough data to score metric '{metric}' "
                f"in range {start} - {end}."
            )

        return value

    def add(  # pylint: disable=too-many-branches
        self,
        timestamp: UnixDatetime,
        *,
        metric: str,
        value: VALUE,
        num_decimals: int = _DEFAULT_DECIMALS,
        align: bool = False,
        threshold: timedelta | None = None,
    ) -> None:
        """Add a value to the given metric.

        Args:
            timestamp: The timestamp of the value.
            metric: The metric to add the value to.
            value: The value to add.
            num_decimals: The number of decimals. Defaults to ``_DEFAULT_DECIMALS``.
            align: Whether to align the timestamp. Defaults to ``False``.
            threshold: The threshold for alignment. Defaults to ``None``.
        """
        if metric not in self._stores:
            raise SourcingError(f"The metric '{metric}' doesn't have a store.")

        store = self._stores[metric]
        store.add(timestamp, value)

    def find(  # pylint: disable=too-many-branches
        self,
        metric: str,
        *,
        start: UnixDatetime | None = None,
        end: UnixDatetime | None = None,
        merge_method: str | None = None,
        resampling_period: timedelta | None = None,
        max_alignment_delay: timedelta | None = None,
        num_decimals: int | None = None,
        threshold: timedelta | None = None,
    ) -> list[Sample]:
        """Find values for the given metric.

        Args:
            metric: The metric to find.
            start: The start datetime.
            end: The end datetime.
            merge_method: The merge method.
            resampling_period: The resampling period.
            max_alignment_delay: The max alignment delay.
            num_decimals: The number of decimals.
            threshold: The threshold.

        Returns:
            A list of samples.
        """
        if metric not in self._stores:
            raise SourcingError(f"The metric '{metric}' doesn't have a store.")

        store = self._stores[metric]

        if start is not None and end is not None:
            return [
                Sample(
                    timestamp=timestamp,
                    value=store.fetch(timestamp, timestamp + store.bucket_interval),
                )
                for timestamp in store.dates_in_range(start, end)
            ]

        return []

    def join(  # pylint: disable=too-many-branches
        self,
        *metrics: str,
        start: UnixDatetime,
        end: UnixDatetime,
        join_method: str | None = None,
        merge_method: str | None = None,
        resampling_period: timedelta | None = None,
        max_alignment_delay: timedelta | None = None,
        num_decimals: int | None = None,
        threshold: timedelta | None = None,
    ) -> dict[str, list[Sample]]:
        """Join the given metrics.

        Args:
            metrics: The metrics to join.
            start: The start datetime.
            end: The end datetime.
            join_method: The join method.
            merge_method: The merge method.
            resampling_period: The resampling period.
            max_alignment_delay: The max alignment delay.
            num_decimals: The number of decimals.
            threshold: The threshold.

        Returns:
            A dictionary with the joined metrics and their values.
        """
        collected = {}
        for metric in metrics:
            if metric not in self._stores:
                raise SourcingError(f"The metric '{metric}' doesn't have a store.")

            store = self._stores[metric]

            collected[metric] = [
                Sample(
                    timestamp=timestamp,
                    value=store.fetch(timestamp, timestamp + store.bucket_interval),
                )
                for timestamp in store.dates_in_range(start, end)
            ]

        return collected

    def reset(self) -> None:
        """Reset the sourcing actor."""
        for store in self._stores.values():
            store.reset()
        self._sourcing_events = []

    def __sub__(self, other: Any) -> SourcingActor:
        """Subtract the given value.

        Args:
            other: The value to subtract.

        Returns:
            The subtracted value.
        """
        raise SourcingError("Subtraction is not supported.")

    def merge_groups(self) -> None:
        """Merge the groups."""
        pass

    def remove_groups(self) -> None:
        """Remove the groups."""
        pass

    @property
    def model(self) -> dict[str, BaseStore]:
        """The model of the sourcing actor.

        Returns:
            The model.
        """
        return self._stores

    async def save(self, path: str) -> None:
        """Save the metric data to files.

        This is a debug helper for producing diagnostics and should not be
        used in production.

        The format of the file names will be:
        ``{path}/{metric}_{component_id}.json``

        Args:
            path: The path to save the data to.
        """
        import json  # pylint: disable=import-outside-toplevel

        for metric, store in self._stores.items():
            data = {}
            for ts in store.dates_in_range(
                store.first_timestamp,
                store.last_timestamp,
            ):
                data[str(ts)] = store.fetch(ts, ts + store.bucket_interval)

            with open(
                f"{path}/{metric}.json",
                "w",
                encoding="utf-8",
            ) as file:
                json.dump(data, file, indent=4)