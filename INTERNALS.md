# spangap-ble — internals

Maintainer reference. The [README](README.md) is the operator guide; this is for
changing the code without breaking it. It is self-authoritative.

```
BleService::onInit()            main task   seed readouts, register CLI,
 ↓                                          spawn the ble task
bleTaskMain                     ble task    itsClientInit, the host→task queue,
 ↓                                          storage subscriptions
applyConfig()                   ble task    s.ble.enable + s.ble.txpower
 ↓ enabled AND some consumer called bleUp()?   no → hostStop(); park. Stop.
hostStart()                     ble task    nimble_port_init → ble_hs_cfg →
 ↓                                          svc_gap/svc_gatt init →
 ↓                                          count_cfg + add_svcs per consumer →
 ↓                                          ble_store_config_init →
 ↓                                          nimble_port_freertos_init
onHostSync()                    HOST task   set a flag, wake the ble task
 ↓
hostSynced()                    ble task    set a random static address, BLE_EV_UP
 ↓ every pass, while up
bleGapDrain()                   ble task    run what the host task queued
bleAdvReconcile()               ble task    rotate the advertising claim
bleScanReconcile()              ble task    start/stop/re-parameterise the scan
blePairingTick()                ble task    close the window when it expires
```

Two rules the whole file set rests on:

- **A consumer's spangap-ble callback never runs in host context.** Host
  callbacks copy into a queue and wake the ble task; every decision is made
  there. The one deliberate exception is `bleOnNotifyRx`, documented as such in
  `ble.h`, because its payload is far larger than an event message.
- **Nothing that touches the radio runs on a caller's task.** The public calls
  write a want under `s_lock` and wake the ble task; the task reconciles against
  that want. There is no request queue and no completion, which is what makes a
  lost wake harmless.

## 1. What this straddle adds

Everything is new on top of ESP-IDF's `components/bt` (Apache NimBLE) — no
upstream fork, no vendored code. Four source files plus a private header:

- **`ble.cpp`** — `BleService`, the ble task, the host lifecycle, the queued
  GATT registry, the event registry and its fan-out, the connection table and
  budget, the settings reconcile and the two command sentinels.
- **`ble_gap.cpp`** — the single legacy advertising instance and its round-robin,
  the single scanner and its duty cycle, the one GAP event handler every
  connection is opened with, the host→task marshal queue and its drain, the
  notification flow-control arm, and the recently-seen table `ble scan` reads.
- **`ble_sec.cpp`** — one `ble_hs_cfg`, the timed pairing window and its
  two-sided enforcement, and the bond store.
- **`ble_store.cpp`** — bond persistence in `/state` around NimBLE's RAM-only
  store: snapshot on change (debounced), restore at every host start. Bonds
  ride `/state` backups; NVS is never written.
- **`ble_cli.cpp`** — the six `ble` verbs.
- **`ble_priv.h`** — the cross-file state, in a `spangap_ble` namespace pulled in
  unqualified. Plain `s_`-prefixed globals would be external symbols in a flat
  link of forty straddles, where `s_enabled` and `s_configDirty` are names half
  of them use for something file-local.

Plus the `kconfig:` block in `straddle.yaml`, which is where `CONFIG_BT_*` is
set for the whole image. The build generator warns when two straddles set the
same symbol, which is one more reason exactly one straddle owns it.

## 2. Tasks and what may touch what

Three tasks, and the boundary between them is the contract.

| Task | Owns | May call |
|---|---|---|
| NimBLE host | NimBLE's own state | `hevPost`, `s_notifyRx`, `ble_gap_*` |
| `ble` (prio 2, `CORE_PRIMARY`, **DRAM stack**) | everything in `ble_priv.h` | anything |
| a consumer's task | its own state | the public API in `ble.h` |

Nothing Bluetooth writes NVS. NimBLE's stock store runs RAM-only
(`CONFIG_BT_NIMBLE_NVS_PERSIST` off) and `ble_store.cpp` persists it to
`/state/ble/bonds.bin` — debounced on the ble task, flushed before a host
restart's `ble_store_config_init()` (which zeroes the RAM counts), reloaded
right after. PHY calibration persistence is off too (full calibration once
per boot, ~100–200 ms), so the ble task carries a normal PSRAM stack.
`bleForget`/`bleForgetAll` still POST here rather than unpairing on the
caller: bond mutations stay serialized on the task that owns the store's
snapshot.

The ble task sits at priority 2 because it owns a radio, the way spangap-net's
task does; every consumer stays at 1. A producer above its consumer never lets
back-pressure clear, and here the owner is the one thing that must keep running.

`bleGapEvent` is the single GAP callback for every connection this straddle
opens — advertising-originated and `bleConnect`-originated alike — so there is
one place connection events arrive. It builds a `ble_event_t`, posts it to a
16-deep queue and wakes the ble task. `bleGapDrain()` then updates the
connection table, enforces the budget, and fans the event out.

**Event delivery is ITS aux, to the registering task.** `bleRegister` records
`{task, callback, event}` and installs one trampoline on that task's
`BLE_AUX_PORT_EVENT`; `bleFire` sends **one** message per distinct task however
many callbacks it registered, and the trampoline fans out locally. A consumer's
handler therefore runs under its own `itsPoll`, on its own stack, with its own
state safe to touch — the same shape `storageSubscribeChanges` has.

`BLE_EV_UP` is level-replayed synchronously, on the registering task, so
registration order against bring-up does not matter. Everything else is
edge-only: replaying a teardown to a handler that never set up is wrong.

## 3. Why the host starts lazily, and what it costs

`nimble_port_init()` claims the controller, its internal-DRAM activity pool and
the host's memory. Doing that at boot on a build where nothing uses Bluetooth
would be a permanent tax for a feature nobody asked for, so the host comes up on
the first reconcile pass in which some owner's want and `s.ble.enable` are both
set.

The consequence is the one thing that shapes the public API. **NimBLE's GATT
registry only exists between `nimble_port_init()` and the host start** —
`ble_hs_start()` calls `ble_gatts_start()`, which closes it. A consumer's
`onInit()` runs long before that, so `bleGattAdd()` cannot register anything: it
records the pointer in `s_svcs[]` and `hostStart()` feeds the whole list to
`ble_gatts_count_cfg()` + `ble_gatts_add_svcs()` at bring-up. Hence the rule
that a consumer's `ble_gatt_svc_def` must be `static const` and outlive the
process — the pointer is dereferenced long after `onInit()` returned.

`s_svcRegistered` records how many services the running host was built with. A
`bleGattAdd` after the host started sets the config-dirty flag; the next
reconcile pass sees the mismatch and rebuilds the host, which drops every
connection. Legal, and only the reconcile path ever reaches it.

Teardown is `nimble_port_stop()` then `nimble_port_deinit()`; the host task
function returns from `nimble_port_run()` and calls
`nimble_port_freertos_deinit()` itself. Transmit power is a controller setting
that `nimble_port_deinit` resets, so it is re-applied on every start.

## 4. The advertising rotation

One legacy instance, one 31-byte payload. Flags cost 3 bytes and a 128-bit
service UUID costs 18, leaving no room for a name — so the UUID goes in the
advertisement and the name in the scan response, and two consumers wanting
different UUIDs cannot share one payload at all.

**Why not extended advertising**, which would carry an instance per consumer:
Android scanners use legacy scan parameters by default, and both peer
implementations that matter (Sideband through `able`, Columba) scan that way. An
extended-only advertisement would be invisible to them.

So claims are round-robined: `BLE_ADV_SLOT_MS` (2 s) per claim, stop → set
fields → set scan response → start, because legacy advertising data cannot be
changed in place. `advNextSlot` wraps over the used slots; a single claim is
recognised and left on air rather than restarted, which is what makes the
round-robin free in the common case.

**A bonded central can connect during any slot.** A central dials an address,
not a payload — the RNS `RNodeInterface` client never scans at all, it
enumerates the phone's bonded devices — so as long as the currently-active slot
is connectable, its connect lands. Only *discovery* of a payload that is not
currently on air waits for the rotation, and that costs at most one slot period.

`bleAdvNextDueMs()` is what sets the task's poll timeout while advertising, so
the rotation is a deadline rather than a timer.

## 5. Security: global policy, per-characteristic demand

One `ble_hs_cfg` for the process: `sm_sc = 1` (LE Secure Connections),
`sm_io_cap = NO_INPUT_OUTPUT` (Just Works), `sm_mitm = 0`, ENC+ID key
distribution both ways, `ble_store_util_status_rr` so a full bond store evicts
round-robin rather than refusing the pairing an operator just asked for.

What differs between consumers is the **access flags on their own
characteristics**. `rnode-ble`'s Nordic UART Service carries `_ENC` flags and so
forces a bond before a write lands; `iface-ble`'s v2.2 service carries none and
never raises a pairing dialog. That is the whole mechanism, and it is why one
device can demand a bond on one service and stay open on another.

**`_ENC`, never `_AUTHEN`.** Just Works produces an *unauthenticated* bond. A
characteristic flagged `WRITE_AUTHEN` would reject every write from a peer that
paired exactly as intended.

### The pairing window

NimBLE has no hook that can refuse an incoming pairing request before the
exchange starts, so the window is enforced from both ends:

- while it is closed, `ble_hs_cfg.sm_bonding` is 0, so no long-term key is
  distributed and no bond is written;
- and `bleSecOnPairingComplete` — reached from `BLE_GAP_EVENT_ENC_CHANGE` /
  `BLE_GAP_EVENT_PARING_COMPLETE`, marshalled onto the ble task — unpairs and
  drops any peer that bonded anyway.

`ble_conn_t::wasBonded` records whether the peer was *already* bonded when it
connected, so a re-encryption of an existing bond is not mistaken for a new one.

`BLE_GAP_EVENT_REPEAT_PAIRING` drops our side of the bond and returns
`BLE_GAP_REPEAT_PAIRING_RETRY`. A device that could not recover from a one-sided
forget without an operator finding the menu is a device that looks broken.

## 6. The connection budget

`CONFIG_BT_NIMBLE_MAX_CONNECTIONS` is 4 — three mesh peers beside the RNode
door, or four peers with the door off. The protocol side could carry eight,
but the controller's internal DRAM cannot be bought on this image: its heap
and its exchange-memory arena both scale with `CONFIG_BT_CTRL_BLE_MAX_ACT`
(connections + advertising + scanning = 6 here), and the controller allocates
internal regardless of `BT_NIMBLE_MEM_ALLOC_MODE_EXTERNAL` — that symbol moves
the *host's* memory to PSRAM and nothing else.

Because the controller also **fails by asserting** (a mid-init malloc failure
reboots the chip through the interrupt watchdog), `hostStart` verifies the
heap before touching it and defers with a retry otherwise — and the RAM itself
is EARMARKED: `onInit` grabs `BLE_CTRL_RESERVE` while the boot heap is still
one large block, `hostStart` frees it into the controller's hands, and
`hostStop` grabs it back. Fragmentation from the boot storm is what the
earmark defeats; the contiguous arena is guaranteed by construction, not
hoped for. The earmark is only held when `s.ble.enable` was set at boot.

The controller also carries **~22 KB of mandatory IRAM** (libbtdm_app + libbt +
libbtbb), and IRAM and DRAM share the same internal SRAM — so staging this
straddle shrinks the boot-time internal heap, which must still fit
spangap-core's `CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL` pool or startup aborts
before `app_main`. The `straddle.yaml` kconfig block pays that IRAM back by
moving flash-safe code out of IRAM (FreeRTOS non-ISR functions and ringbuf)
and by returning Wi-Fi's optional IRAM placement to flash — a throughput trade
that is the right side of the bargain on a build where the two radios already
split the 2.4 GHz airtime. `CONFIG_SPI_FLASH_ROM_IMPL` and
`CONFIG_HEAP_PLACE_FUNCTION_INTO_FLASH` would buy ~20 KB more and must not be
added: both sit on the flash erase path, and the ROM driver dies mid-way
through the first-boot 8 MB `/state` format — a silent watchdog reset with the
console gone. If the image grows tighter still, the remaining levers are
core's reserve dial and `CONFIG_LWIP_IRAM_OPTIMIZATION`.

`bleSlotReserve(owner, n)` divides that budget. It is an **agreement**, checked
when it is made — the sum of the claims cannot exceed the budget — and kept by
each consumer capping its own peers at what it reserved. It is not a runtime
gate, and cannot be: an inbound connection carries no owner, so there is nothing
to charge it to. What actually leaves a slot for the RNode door is `iface-ble`
refusing to dial a fourth peer, because `s.ble.rns.max_peers` defaults to one
under the table size. In a full reticulous image `iface-ble` reserves three and
`rnode-ble` one, and `ble` prints both.

`bleGapDrain` still refuses a connection when the table is full, but that is the
controller's own ceiling reached rather than a policy — the table is exactly
`CONFIG_BT_NIMBLE_MAX_CONNECTIONS` deep.

## 7. Coexistence with Wi-Fi and ESP-NOW

Bluetooth has its own controller; only the 2.4 GHz antenna is shared, so there
is no `netIsUp()`-style gate to copy from iface-espnow. Software coexistence is
on by default in IDF and arbitrates the airtime.

Scanning is the expensive part and the part we can cheapen, so
`bleScanReconcile` halves the scan window while any connection is attached — an
established link keeps its airtime and discovery slows down, which is the right
way round. The connect and disconnect paths both set `s_scanDirty` so the duty
cycle is re-derived whenever the connection count changes. A dial also cancels
the scan outright: this controller cannot scan and initiate at the same time,
and the reconcile pass brings the scan back afterwards.

## 8. Pitfalls

- **A `ble_gatt_svc_def` handed to `bleGattAdd` must be `static const`.** The
  pointer is dereferenced at host bring-up, arbitrarily later than the call.
- **Register events from the task that should handle them, once per process.**
  The registry is a fixed append-only array with no unregister; re-registering
  on every start piles up duplicates, and registering from the wrong task
  delivers to the wrong `itsPoll`.
- **`bleUp()` is a want, not a command.** It returns before the radio does
  anything, and `bleIsUp()` is false until `BLE_EV_UP`. Code that reads
  `bleIsUp()` immediately after `bleUp()` is wrong; wait for the event.
- **Wants are per owner.** Each consumer's `bleUp(owner)` stands until its own
  `bleDown(owner)`; the host is torn down only when the last want is gone, so
  one consumer disabling itself cannot take the radio from another.
- **`s.ble.enable = 0` beats every consumer.** The master switch is applied at
  reconcile time; the want is still recorded, so switching the master back on
  restores the radio with no action from the consumer.
- **A failed dial is a ZEROED NimBLE event.** `BLE_GAP_EVENT_CONNECT` with a
  non-zero status carries `conn_handle` 0 — a live connection's handle — so the
  failure is marshalled as `BLE_IEV_DIAL_FAILED` without reading it, and reaches
  consumers as `BLE_EV_DISCONNECT` with conn none and the dialled address.
- **A dial's outcome must never depend on the host-event queue alone.** The
  queue drops on overflow (a scan-report burst; the drop count is reported by
  the ble task) and a connect can die between its event and the conn lookup.
  `s_dialInFlight` gates the scan, all dialling and the address rotation, so
  a stuck flag paralyses all three — the dial watchdog in the task loop
  force-fails any dial still unresolved past its own timeout plus margin.
- **The address is a non-resolvable private address and rotates on every
  disconnect.** A fresh one is generated at every host start, after every
  disconnect (`bleAddrRotateDue()`, marked in the gap drain), and at latest
  every `BLE_ADDR_ROTATE_MS` while idle, so stale per-address state on a peer
  can never match this device twice — the per-connection freshness phones get
  from their radios, achieved sequentially with the one address slot this
  stack has. Non-resolvable, not random static, for the top bits: 00
  sorts below every phone's resolvable private address (01), which an
  address-comparison role election needs to take the dialling side against
  phones — a random static address (11) loses that election every time. A
  consumer that caches `bleOwnAddr()` must re-read it on every `BLE_EV_UP` —
  one fires after each rotation. Rotation strands bonds (the peer recorded the
  old address), which is accepted until resolvable-private-address privacy
  with a persisted identity resolving key exists; the rotation itself stops
  advertising and the scan and defers past any pending dial before
  `ble_hs_id_set_rnd`, because the controller refuses the command while any of
  them uses the address. Live connections keep running through it.
- **`ble_gatts_notify_custom` consumes its mbuf on every path**, success or
  failure. Bytes handed to it are gone, so a consumer that cannot afford to lose
  them must hold the chunk itself and re-send — see `rnode-ble`'s outbound carry.
- **`BLE_EV_NOTIFY_TX` fires only after `bleNotifyArm`.** It is one-shot and
  per connection. An unconditional fan-out would put an ITS message beside every
  chunk of a busy stream.
- **512-byte ATT values live on the host task's stack.** Both the
  inbound-notification hand-off and a consumer's `access_cb` flatten a value
  there, which is why `CONFIG_BT_NIMBLE_HOST_TASK_STACK_SIZE` is raised to 6144
  from IDF's 4096. Adding another such buffer means revisiting that number.
- **Do not log, allocate through the platform, or touch storage in host
  context.** Copy and post. The queue is deliberately shallow so a violation
  shows up as a stall rather than as corruption.
