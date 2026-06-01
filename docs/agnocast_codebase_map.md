# Agnocast codebase map for a ROS 2 Humble experiment

This document is a repository-local map for an experiment that compares ordinary ROS 2 Humble intra-host communication with Agnocast communication. It describes what is implemented in this checkout; it is not an installation guide or a performance claim.

## 1. Folder structure

| Path | Role in the repository | Useful entry points |
| --- | --- | --- |
| `src/agnocastlib/` | Main C++ userspace library. This is the core implementation to read first. | `include/agnocast/agnocast_publisher.hpp`, `include/agnocast/agnocast_subscription.hpp`, `include/agnocast/agnocast_smart_pointer.hpp`, `src/agnocast.cpp`, `src/agnocast_publisher.cpp`, `src/agnocast_subscription.cpp`, `src/agnocast_callback_info.cpp` |
| `agnocast_kmod/` | Linux character-device kernel module. It stores process/topic/publisher/subscriber/message metadata and serves the ioctl API. | `agnocast.h`, `agnocast_init.c`, `agnocast_ioctl.c`, `agnocast_memory_allocator.c` |
| `agnocast_heaphook/` | Rust `LD_PRELOAD` heap hook. It initializes the process mempool and routes allocations made while a publisher has a borrowed message into the shared-memory allocator. | `src/lib.rs`, `src/tlsf.rs` |
| `src/agnocast/` | ROS package aggregation/export layer. | `package.xml`, `CMakeLists.txt` |
| `src/agnocast_components/` | Component-container and node-registration support. | `src/agnocast_component_container.cpp`, `src/agnocast_component_container_mt.cpp`, `src/agnocast_component_container_cie.cpp` |
| `src/agnocast_ioctl_wrapper/` | Small userspace wrapper used for introspection commands against `/dev/agnocast`. | `src/topic_list.cpp`, `src/topic_info.cpp`, `src/node_info.cpp`, `src/version.cpp` |
| `src/ros2agnocast/` | Python ROS 2 CLI extension. | `ros2agnocast/command/agnocast.py`, `ros2agnocast/verb/` |
| `src/ros2agnocast_discovery_agent/` and `src/ros2agnocast_discovery_msgs/` | Discovery/type-registry support used by ROS 2 CLI bridging. | `ros2agnocast_discovery_agent/agent.py`, `ros2agnocast_discovery_agent/type_registry.py`, `src/ros2agnocast_discovery_msgs/msg/` |
| `src/agnocast_sample_application/` and `src/agnocast_sample_interfaces/` | Minimal runnable examples and sample message/service definitions. | `src/minimal_publisher.cpp`, `src/minimal_subscriber.cpp`, `msg/DynamicSizeArray.msg`, launch files |
| `src/agnocast_e2e_test/` | End-to-end test nodes and launch-driven tests, including plain ROS 2 peers. | `src/test_publisher.cpp`, `src/test_subscriber.cpp`, `src/test_ros2_publisher.cpp`, `src/test_ros2_subscriber.cpp`, `test/test_1to1.py`, `test/test_2to2.py` |
| `src/agnocast_cie_thread_configurator/` and `src/agnocast_cie_config_msgs/` | Callback-isolated executor (CIE) configuration support. | `src/`, `include/`, `msg/`, `srv/` |
| `docs/` | Design notes, including dedicated shared-memory and message-queue descriptions. | `shared_memory.md`, `message_queue.md`, `agnocast_ros2_bridge.md` |
| `scripts/` | Development, test, release, setup, and sample-application helpers. | `scripts/dev/`, `scripts/test/`, `scripts/sample_application/` |
| `model_checking/` | Model-checking material for the callback-isolated executor. | `agnocast_callback_isolated_executor/` |

## 2. Main packages and runtime layers

### ROS packages

The ROS package manifests under `src/` are:

- `agnocastlib`: the main C++ API and runtime implementation.
- `agnocast`: an aggregation/export package.
- `agnocast_components`: component-container support.
- `agnocast_ioctl_wrapper`: CLI-facing `/dev/agnocast` introspection wrapper.
- `ros2agnocast`: Python `ros2` command extension.
- `ros2agnocast_discovery_agent` and `ros2agnocast_discovery_msgs`: discovery/type-registry agent and its messages.
- `agnocast_sample_application` and `agnocast_sample_interfaces`: examples and experiment-friendly sample types.
- `agnocast_e2e_test`: mixed Agnocast/ROS 2 end-to-end test nodes.
- `agnocast_cie_thread_configurator` and `agnocast_cie_config_msgs`: callback-isolated executor configuration.

### Runtime architecture

For the publish/subscribe data path, think of the repository as four cooperating layers:

1. **C++ API:** `agnocast::Publisher<MessageT>` is an alias for `BasicPublisher<MessageT, AgnocastToRosPubsubRequestPolicy>`, and `agnocast::Subscription<MessageT>` is an alias for `BasicSubscription<MessageT, RosToAgnocastPubsubRequestPolicy>`. Polling variants are exposed as `TakeSubscription<MessageT>` and `PollingSubscriber<MessageT>`.
2. **Shared-memory allocator hook:** the preloaded Rust library initializes a per-process TLSF-backed mempool and decides whether intercepted heap allocation should use shared memory.
3. **POSIX IPC primitives:** POSIX shared-memory objects hold the allocated message objects; POSIX message queues carry wake-up notifications, not serialized message payloads.
4. **Kernel module:** `/dev/agnocast` ioctls register processes and endpoints, retain message virtual addresses, track subscriber references, choose entries to deliver, and tell subscribers which publisher shared-memory regions must be mapped.

The bridge code under `src/agnocastlib/include/agnocast/bridge/` and `src/agnocastlib/src/bridge/` connects Agnocast endpoints to ordinary ROS 2 endpoints. It is relevant when an experiment intentionally mixes one Agnocast side with one ROS 2 side, but it is not the direct Agnocast-to-Agnocast fast path.

## 3. Publisher and subscriber implementation

### Publisher path

The user-facing publisher type is `agnocast::Publisher<MessageT>`, declared as an alias of `BasicPublisher`. The important call chain is:

1. `BasicPublisher::constructor_impl()` resolves the topic, validates QoS, invokes `initialize_publisher()`, generates a GID, and requests a bridge according to its bridge policy.
2. `initialize_publisher()` calls `validate_ld_preload()`, optionally records the ROS message type through `internal::TypeRegistryWriter::register_type()`, and registers the endpoint with `ioctl(agnocast_fd, AGNOCAST_ADD_PUBLISHER_CMD, ...)`.
3. `BasicPublisher::borrow_loaned_message()` increments the thread-local `borrowed_publisher_num`, performs `new MessageT()`, and wraps the pointer in `ipc_shared_ptr<MessageT>`.
4. `BasicPublisher::publish(ipc_shared_ptr<MessageT> &&)` captures the virtual address, invalidates all publisher-side aliases, decrements `borrowed_publisher_num`, invokes `publish_core()`, deletes addresses that the kernel reports as releasable, and resets the moved pointer.
5. `publish_core()` sends `AGNOCAST_PUBLISH_MSG_CMD`. The kernel returns subscriber IDs and releasable addresses. Userspace then opens each subscriber's POSIX message queue and calls `mq_send()` with a deliberately zero-length notification.

The key implementation files and symbols are:

- `src/agnocastlib/include/agnocast/agnocast_publisher.hpp`: `BasicPublisher`, `BasicPublisher::borrow_loaned_message()`, `BasicPublisher::publish()`, and the `Publisher` alias.
- `src/agnocastlib/src/agnocast_publisher.cpp`: `initialize_publisher()`, `publish_core()`, `increment_borrowed_publisher_num()`, and `decrement_borrowed_publisher_num()`.
- `agnocast_kmod/agnocast_ioctl.c`: `agnocast_ioctl_publish_msg()`.

The minimal example is `src/agnocast_sample_application/src/minimal_publisher.cpp`: `MinimalPublisher::timer_callback()` calls `publisher_->borrow_loaned_message()`, fills `message->data`, and calls `publisher_->publish(std::move(message))`.

### Event-driven subscriber path

The user-facing event-driven subscriber is `agnocast::Subscription<MessageT>`, an alias of `BasicSubscription`. The important call chain is:

1. `BasicSubscription::constructor_impl()` validates QoS and calls `SubscriptionBase::initialize()`.
2. `SubscriptionBase::initialize()` optionally records the type and registers the endpoint with `AGNOCAST_ADD_SUBSCRIBER_CMD`.
3. `open_mq_for_subscription()` creates a nonblocking, receive-only POSIX message queue for the subscriber ID.
4. `BasicSubscription::constructor_impl()` calls `register_callback()` so the queue descriptor is monitored by the Agnocast executor's epoll machinery.
5. When a queue becomes readable, `SubscriptionEventHandler::handle()` drains its wake-up message with `mq_receive()` and enqueues work.
6. `receive_and_execute_message()` calls `AGNOCAST_RECEIVE_MSG_CMD`, maps newly discovered publisher mempools read-only with `map_read_only_area()`, wraps each returned virtual address in an `ipc_shared_ptr`, and runs the callback.
7. When the last subscriber-side smart-pointer copy is destroyed, `release_subscriber_reference()` sends `AGNOCAST_RELEASE_SUB_REF_CMD` to the kernel module.

The key implementation files and symbols are:

- `src/agnocastlib/include/agnocast/agnocast_subscription.hpp`: `SubscriptionBase`, `BasicSubscription`, `open_mq_for_subscription()`, and `Subscription`.
- `src/agnocastlib/src/agnocast_subscription.cpp`: `SubscriptionBase::initialize()`, `open_mq_for_subscription()`, and `remove_mq()`.
- `src/agnocastlib/src/agnocast_callback_info.cpp`: `SubscriptionEventHandler::handle()`, `enqueue_receive_and_execute()`, and `receive_and_execute_message()`.
- `src/agnocastlib/src/agnocast_smart_pointer.cpp`: `release_subscriber_reference()`.
- `agnocast_kmod/agnocast_ioctl.c`: `agnocast_ioctl_receive_msg()`.

The minimal example is `src/agnocast_sample_application/src/minimal_subscriber.cpp`: `MinimalSubscriber::callback()` receives an `agnocast::ipc_shared_ptr<DynamicSizeArray>`.

### Polling subscriber path

For an experiment that wants explicit polling instead of callback wake-ups, `agnocast::TakeSubscription<MessageT>` aliases `BasicTakeSubscription`. Its `take(bool allow_same_message = false)` method issues `AGNOCAST_TAKE_MSG_CMD`, maps newly reported publisher mempools, and returns `ipc_shared_ptr<const MessageT>`. `agnocast::PollingSubscriber<MessageT>` wraps that take subscription and exposes `takeData()` / `take_data()`.

## 4. Shared-memory usage

Agnocast does **not** serialize the ROS message into a shared buffer. The publisher allocates the actual generated ROS message object, including its dynamically sized members, from a shared-memory-backed heap. The consumer maps the publisher's mempool at the same virtual address and dereferences the returned address.

### Initialization and mapping

`initialize_agnocast()` in `src/agnocastlib/src/agnocast.cpp` opens `/dev/agnocast`, checks the kernel-module version with `AGNOCAST_GET_VERSION_CMD`, registers the process with `AGNOCAST_ADD_PROCESS_CMD`, and maps the returned address range with `map_writable_area()`. The underlying `map_area()` function:

- derives the POSIX shared-memory name with `create_shm_name(pid)`, which returns `/agnocast@{pid}`;
- calls `shm_open()` and `ftruncate()` for a publisher's writable region;
- changes the shared-memory object's mode to read-only after creation;
- maps at the kernel-selected address with `mmap(..., MAP_SHARED | MAP_FIXED_NOREPLACE, ...)`.

On first delivery from a publisher, `receive_and_execute_message()` or `BasicTakeSubscription::take()` receives `publisher_shm_info` records from the kernel and calls `map_read_only_area(pid, addr, size)`. This same-address mapping is why the kernel allocator manages virtual-address ranges and why the repository's `docs/shared_memory.md` warns that overlapping mappings can cause `mmap()` failures or later pointer crashes.

### Allocation routing

The preloaded `agnocast_heaphook` library is part of the data path:

- `AgnocastSharedMemory::new()` resolves and calls C++ `initialize_agnocast()`; the `__libc_start_main()` hook then initializes `AGNOCAST_SHARED_MEMORY` and `AGNOCAST_SHARED_MEMORY_ALLOCATOR` over the returned mempool;
- its intercepted `malloc()` calls `should_use_heap()`;
- `should_use_heap()` calls C++ `agnocast_get_borrowed_publisher_num()` and returns false while a publisher has an outstanding borrowed message, selecting the shared-memory allocator;
- while true, the `new MessageT()` executed by `BasicPublisher::borrow_loaned_message()` and nested allocations such as a message's `std::vector` storage come from the shared-memory allocator.

This means an experiment must load the heap hook correctly; `initialize_publisher()` and `SubscriptionBase` call `validate_ld_preload()` to enforce that expectation.

### Kernel-side virtual-address allocation

`agnocast_kmod/agnocast_memory_allocator.c` manages allocatable virtual-address chunks, and `agnocast_ioctl_add_process()` returns an address and shared-memory size for each process. The module tracks addresses and message entries; the payload bytes stay in POSIX shared memory mapped into userspace.

## 5. POSIX message-queue usage

For the direct Agnocast publish/subscribe path, POSIX message queues are **notification channels**, not payload channels:

- `open_mq_for_subscription()` creates the subscriber-side queue with `mq_open(..., O_CREAT | O_RDONLY | O_NONBLOCK, ...)`, a capacity of one message, and `sizeof(MqMsgAgnocast)` as its configured maximum message size.
- `publish_core()` opens the same queue with `O_WRONLY | O_NONBLOCK` and calls `mq_send(..., 0 /* msg_len */, ...)`.
- `SubscriptionEventHandler::handle()` calls `mq_receive()`, after which `receive_and_execute_message()` asks the kernel for actual addresses through `AGNOCAST_RECEIVE_MSG_CMD`.
- `MqMsgAgnocast` is an empty struct by design.

Queue names are built by `create_mq_name_for_agnocast_publish()`. For a resolved topic `/foo/bar` and subscriber-local ID `3`, `create_mq_name()` produces `/agnocast@foo_bar@3`: the leading topic slash becomes `@`, and remaining slashes become `_`.

POSIX queues are also used separately for bridge-manager requests. `src/agnocastlib/include/agnocast/agnocast_mq.hpp` defines `MqMsgBridge`, `MqMsgPerformanceBridge`, `BridgeDirection`, and `create_mq_name_for_bridge()` naming support. Do not confuse bridge request queues with the per-subscriber zero-length publish-notification queues.

## 6. ioctl and kernel-driver interaction

### Device creation and userspace opening

`agnocast_kmod/agnocast_init.c` defines `fops` with `.unlocked_ioctl = agnocast_ioctl`, registers the character device, and creates the device filename `agnocast`, resulting in `/dev/agnocast`. Userspace opens it in `initialize_agnocast()`.

The shared ioctl ABI is declared in both:

- `src/agnocastlib/include/agnocast/agnocast_ioctl.hpp` for the C++ runtime;
- `agnocast_kmod/agnocast.h` for the kernel module.

The kernel dispatcher is `agnocast_ioctl()` in `agnocast_kmod/agnocast_ioctl.c`.

### Data-path ioctls

| ioctl macro | Userspace caller | Kernel handling role |
| --- | --- | --- |
| `AGNOCAST_GET_VERSION_CMD` | `initialize_agnocast()` | Confirms userspace/heaphook/kernel compatibility before initialization. |
| `AGNOCAST_ADD_PROCESS_CMD` | `initialize_agnocast()` | Registers a process and returns the mempool address and size that userspace maps writable. |
| `AGNOCAST_ADD_PUBLISHER_CMD` | `initialize_publisher()` | Registers a publisher and returns its topic-local ID. |
| `AGNOCAST_ADD_SUBSCRIBER_CMD` | `SubscriptionBase::initialize()` | Registers a subscriber and returns its topic-local ID. |
| `AGNOCAST_PUBLISH_MSG_CMD` | `publish_core()` | Records the message virtual address, applies topic/QoS bookkeeping, returns interested subscriber IDs, and returns old addresses whose storage can be deleted. |
| `AGNOCAST_RECEIVE_MSG_CMD` | `receive_and_execute_message()` | Returns queued entry IDs and addresses for callback delivery and reports publisher mempools that the subscriber must map. |
| `AGNOCAST_TAKE_MSG_CMD` | `BasicTakeSubscription::take()` | Returns the newest eligible entry address for polling delivery and reports publisher mempools to map. |
| `AGNOCAST_RELEASE_SUB_REF_CMD` | `release_subscriber_reference()` | Drops the subscriber reference after the last subscriber-side `ipc_shared_ptr` copy is destroyed. |
| `AGNOCAST_REMOVE_PUBLISHER_CMD` | `BasicPublisher::~BasicPublisher()` | Removes a publisher endpoint. |
| `AGNOCAST_REMOVE_SUBSCRIBER_CMD` | `SubscriptionBase::~SubscriptionBase()` | Removes a subscriber endpoint. |

The file also defines count, QoS, bridge, exit-cleanup, and CLI-introspection commands. For example, `get_subscription_count_core()` uses `AGNOCAST_GET_SUBSCRIBER_NUM_CMD`, and the separate `src/agnocast_ioctl_wrapper/` library uses topic/node/version introspection commands.

### What crosses the ioctl boundary

The key data crossing the driver boundary is metadata:

- topic and node names;
- topic-local publisher/subscriber IDs;
- QoS-related flags and queue depths;
- message virtual addresses and entry IDs;
- publisher shared-memory `(pid, shm_addr, shm_size)` records;
- subscriber IDs used by userspace to send POSIX queue notifications;
- releasable virtual addresses returned to publishers for deletion.

The ROS message payload itself remains in userspace-mapped shared memory.

## 7. Is `ipc_shared_pointer` integrated?

**Yes, if by `ipc_shared_pointer` you mean the repository's `agnocast::ipc_shared_ptr`.** The implemented symbol is named `ipc_shared_ptr`, not `ipc_shared_pointer`.

It is integrated into the primary pub/sub path rather than being an isolated prototype:

- `BasicPublisher::borrow_loaned_message()` returns `ipc_shared_ptr<MessageT>`.
- `BasicPublisher::publish()` accepts `ipc_shared_ptr<MessageT> &&` and invalidates all aliases sharing the publisher-side control block before publishing.
- `receive_and_execute_message()` constructs subscriber-side pointers through callback metadata, and `BasicTakeSubscription::take()` directly returns `ipc_shared_ptr<const MessageT>`.
- `ipc_shared_ptr` owns a `detail::control_block` with atomic `ref_count` and `valid` fields, topic name, entry ID, and pub/sub ID.
- Subscriber-side final release calls `release_subscriber_reference()`, which issues `AGNOCAST_RELEASE_SUB_REF_CMD`.
- The sample publisher and subscriber use this type directly.

One important nuance: the C++ control block tracks copies *inside one process*, while the kernel module tracks publish/subscribe-aware references across processes. The pointer is therefore integrated with both userspace atomic reference counting and kernel entry-reference bookkeeping.

## 8. Minimal experiment ideas for ROS 2 Humble

The repository README lists ROS 2 Humble as a supported environment. For a clean comparison, keep the message type, payload size, publish rate, process placement, CPU affinity, QoS, subscriber count, warm-up duration, measurement window, and logging level fixed. Change only the transport path. Start with separate publisher and subscriber processes so the comparison targets host-local IPC rather than a composable-node intra-process special case.

### Experiment A: smallest two-process baseline

**Goal:** validate setup before collecting performance data.

- Plain ROS 2 run: create an `rclcpp::Publisher<DynamicSizeArray>` and `rclcpp::Subscription<DynamicSizeArray>` pair using the same message definition as the sample.
- Agnocast run: start from `MinimalPublisher` and `MinimalSubscriber` in `src/agnocast_sample_application/src/minimal_publisher.cpp` and `src/agnocast_sample_application/src/minimal_subscriber.cpp`.
- Begin with one publisher, one subscriber, reliable `KeepLast(10)`, a fixed vector length, and a low rate such as 10 Hz.
- Record successful delivery count and sequence values first. Only after correctness is stable should latency and CPU measurements be trusted.

### Experiment B: payload-size sweep

**Goal:** expose the difference between ordinary ROS 2 transport work and Agnocast's shared-memory object sharing.

- Sweep `DynamicSizeArray::data` sizes, for example 64 B, 4 KiB, 64 KiB, 1 MiB, and 8 MiB.
- Hold rate constant at first; then reduce rate if large messages saturate the baseline.
- Measure end-to-end latency distribution, publisher CPU, subscriber CPU, and process RSS. If possible, also record context switches and page faults because first-touch shared-memory behavior can affect warm-up.
- Separate warm-up samples from steady-state samples: Agnocast maps a newly discovered publisher's mempool on first delivery.

### Experiment C: fan-out sweep

**Goal:** measure one publisher with multiple subscribers.

- Run one publisher and 1, 2, 4, then 8 subscriber processes.
- For Agnocast, note that `publish_core()` receives subscriber IDs from the kernel and sends one POSIX queue notification per subscriber ID while the payload object remains shared.
- Measure publisher CPU, per-subscriber CPU, total delivered messages, and tail latency.

### Experiment D: callback versus polling inside Agnocast

**Goal:** understand executor wake-up overhead separately from the shared-memory mechanism.

- Compare `agnocast::Subscription<MessageT>` with `agnocast::TakeSubscription<MessageT>` or `agnocast::PollingSubscriber<MessageT>`.
- The callback path uses POSIX queue notification, epoll, `mq_receive()`, and `AGNOCAST_RECEIVE_MSG_CMD`.
- The polling path calls `BasicTakeSubscription::take()` and `AGNOCAST_TAKE_MSG_CMD` explicitly.

### Experiment E: mixed ROS 2/Agnocast bridge smoke tests

**Goal:** distinguish the direct fast path from compatibility bridging.

- Test ordinary ROS 2 publisher to Agnocast subscriber and Agnocast publisher to ordinary ROS 2 subscriber separately.
- Reuse the repository's `src/agnocast_e2e_test/src/test_ros2_publisher.cpp` and `src/agnocast_e2e_test/src/test_ros2_subscriber.cpp` as references for plain `rclcpp` peers, and `test_publisher.cpp` / `test_subscriber.cpp` for Agnocast peers.
- Report bridge mode and `AGNOCAST_BRIDGE_MODE` explicitly. Do not merge bridge results into the direct Agnocast-to-Agnocast comparison.

### Suggested first deliverable

A minimal first report can be a matrix with two rows (`ROS 2`, `Agnocast`) and three payload sizes (`4 KiB`, `64 KiB`, `1 MiB`) for one publisher and one subscriber in separate processes at a fixed rate. Include p50/p95/p99 latency, delivered count, publisher CPU, subscriber CPU, and the exact Humble/RMW/kernel/module configuration. Then add fan-out and bridge measurements as separate follow-up tables.
