# spangap-ble

```
s.ble.enable = 0?        yes → the radio never starts, whatever asks. Stop.
 ↓ no
a consumer calls bleUp() → the ble task brings the NimBLE host up
 ↓                         (lazily: nothing asks ⇒ nothing starts)
host synced              → BLE_EV_UP; a fresh random adapter address is set
 ↓
consumers' bleGattAdd()  → their services are registered, once, at this point
 ↓
consumers' bleAdvRequest / bleScanStart
 ↓ round-robin, ~2 s per advertising claim
a peer connects          → BLE_EV_CONNECT → BLE_EV_MTU → BLE_EV_SUBSCRIBE
 ↓
`ble pair 60`            → a NEW bond is accepted only inside that window
```

**spangap-ble** owns the Bluetooth Low Energy stack for the device. NimBLE
initialises once, advertises from one legacy instance, holds one bond store and
one connection budget — those are single resources, so one straddle owns them
and every Bluetooth feature is a consumer of `ble.h`. It is the same relation
[spangap-net](../spangap-net) has with the Wi-Fi radio.

Nothing in here knows what rides the link: no Reticulum, no RNode, no rnsd. It
knows that a consumer wants a service advertised and connections accounted for,
and nothing about what they carry.

## Origins

Not a fork or a port. It wraps **Apache NimBLE**, the BLE-only host that ships
inside ESP-IDF's `components/bt`, and adds the platform shape around it: a
lifecycle, an event registry, a lazily-built GATT registry, an advertising
round-robin, a duty-cycled scanner, a bond store with a timed pairing window,
and a connection budget consumers reserve against.

Every board in this workspace is an ESP32-S3, which has **no Bluetooth Classic
radio**. Everything here is Low Energy.

## What it does

One FreeRTOS task, `ble`, at priority 2 on the primary core — the
[spangap-net](../spangap-net) precedent for a radio owner, with consumers at
priority 1 so the owner is never starved behind the traffic it carries. The task
owns the host lifecycle, the advertising rotation, the scanner, the bond store
and the connection table. NimBLE runs its own host task; every GAP and GATT
callback lands there and is marshalled onto the ble task before anything is
decided, so consumer code never runs in host context.

**The host starts lazily.** Not at boot — on the first reconcile pass in which a
consumer has called `bleUp()` and `s.ble.enable` is 1. A build that stages this
straddle without using it costs flash and nothing else. Wants are per owner:
`bleDown()` releases one consumer's interest without touching another's, and
when nothing wants the radio it stops and gives its heap back.

**Advertising is round-robined.** There is one legacy advertising instance and
one 31-byte payload, and a 128-bit service UUID plus a name will not fit in it
together. Each consumer's claim gets the instance to itself for about two
seconds, with its UUID in the advertisement and its name in the scan response,
before the next claim takes over. A single claim never rotates — it is simply
left on air. A central dials an *address*, not a payload, so a peer that already
knows this device connects during whichever slot happens to be up; only
*discovery* of a not-currently-advertised payload pays for the rotation.

**Scanning is one shared instrument.** The 128-bit-UUID match over the raw
advertisement bytes is done once, here — ESP-IDF offers no BlueZ-style service
filter — and results are delivered to every consumer that registered for them.
The scan window is halved automatically while a connection is attached, because
scanning is the expensive half of sharing 2.4 GHz with Wi-Fi and ESP-NOW.

**Bonding is global; enforcement is per characteristic.** There is one security
policy — LE Secure Connections, Just Works, no MITM — and what differs between
consumers is the access flags on their own characteristics. A service whose
characteristics carry an encryption flag forces pairing; one whose do not never
raises a pairing dialog. New bonds are accepted **only inside the pairing
window**; existing bonds reconnect whenever they like.

**The adapter address rotates, resolvably.** The on-air address is a
resolvable private address derived from a persisted identity resolving key,
and it still rotates — at every host start, after every disconnect, and at
latest every 15 minutes idle. Phones mint a fresh address per connection
attempt, and per-disconnect rotation is the same protection: peers key
per-address state by our address (dedupe sets, role flags, back-off lists),
and a device that presents the same address twice reconnects into whatever
stale entries the peer failed to clean in between. What the key adds is that a
**bonded** peer receives it at pairing and resolves every rotation back to one
identity, so bonds survive rotation and reboot alike (the key persists with
the bonds in `/state/ble/bonds.bin` — losing it would strand every bond, so
they live or die together); strangers still see an unlinkable fresh address.
`bleOwnAddr()` returns the current address and `BLE_EV_UP` fires again
whenever it changes; live connections survive a rotation. The cost is the
election thumb: a resolvable address carries top bits 01, the same as every
phone's, so a consumer electing roles by address comparison now gets a coin
flip against phones per rotation window instead of the old guaranteed
dialling side (the non-resolvable 00 prefix sorted below everything). If
privacy fails to start, the old non-resolvable rotation is the fallback —
fresh addresses, stranded bonds.

### Starts automatically

When `spangap-ble` is in the build its service starts on its own — the generated
init dispatcher constructs `BleService` and calls `onInit()`, which seeds the
defaults, registers the CLI verbs and spawns the ble task. There is no init call
for a consumer to make. A consumer `requires:` this straddle, which puts this
`onInit()` ahead of its own.

## Consuming it

`esp-idf/include/ble.h` is the whole surface and carries the exact contracts.
The shape of a consumer:

```c
void MyService::onInit() {
    bleGattAdd(MY_SVCS);          /* static const, alive forever */
    /* … spawn your own task … */
}

static void myTaskMain(void*) {
    itsClientInit(2);
    bleRegister(BLE_EV_UP,        onBleUp);       /* delivered to THIS task */
    bleRegister(BLE_EV_SUBSCRIBE, onBleSubscribe);
    bleSlotReserve("my-thing", 1);
    bleUp("my-thing");
    for (;;) { while (itsPoll(0)) {} /* work */ itsPoll(portMAX_DELAY); }
}
```

Three things are worth knowing before writing one:

- **`bleGattAdd` queues, it does not register.** NimBLE's GATT registry only
  exists between `nimble_port_init()` and the host start, and the host starts
  lazily, so the definition is recorded and fed to NimBLE at bring-up. It must
  be `static const` and outlive the process.
- **`bleRegister` delivers to the calling task.** Events arrive as ITS aux
  messages, so register from the task that should handle them, and only once per
  process — the registry is append-only. `BLE_EV_UP` is level-replayed to a late
  registrant, so UP handlers must be idempotent.
- **Data does not flow through this straddle.** A consumer's own characteristic
  `access_cb`, and any `ble_gattc_*` completion it registers, run in NimBLE host
  context and must be copy-and-post and nothing else. Inbound notifications on a
  connection this device dialled arrive through `bleOnNotifyRx`, under the same
  rule.

## Storage variables

### Settings (persisted, `s.`)

| Key | Default | Meaning |
|---|---|---|
| `s.ble.enable` | `0` | Master switch. With it off the host never starts, whatever a consumer asks for; the want survives, so switching it back on restores the radio with no action from the consumer. Live. |
| `s.ble.txpower` | `9` | Transmit power in dBm. The controller quantises to 3 dB steps between −24 and +20; the pane offers 0…+20. Live. |

Consumers keep their own keys under this same `s.ble.*` namespace —
`s.ble.rnode.*` for the RNode door, `s.ble.rns.*` for the mesh interface — so an
operator sees one Bluetooth namespace. Those keys belong to those straddles and
are documented there.

### Command sentinels (ephemeral)

| Key | Payload | Answered on |
|---|---|---|
| `ble.pair` | seconds, `0`–`3600` (`0` closes the window immediately) | `ble.pair.error` / `ble.pair.done` |
| `ble.forget` | a Bluetooth address `AA:BB:CC:DD:EE:FF`, or `all` | `ble.forget.error` / `ble.forget.done` |

### Runtime readouts (ephemeral)

| Key | Meaning |
|---|---|
| `ble.state` / `ble.state_text` | `disabled`, `idle` (nothing has asked), `starting`, `up` |
| `ble.up` | `1` while the host is synced |
| `ble.peers` | Attached connections, as one finished line |
| `ble.bonds` | Bonds currently held, as one finished line |
| `ble.pairing_text` | `closed`, or `open, Ns left` |
| `ble.pairing_open` | `1` while the pairing window is open |

### Secrets

None in the secrets store. Bonds — long-term keys included — live in
`/state/ble/bonds.bin`, so they ride `/state` backups and restores, migrate
with the device, and are wiped by a factory reset. Nothing Bluetooth touches
NVS.

## CLI

| Command | What it does |
|---|---|
| `ble` | Status: state, address, transmit power, connections, reservations, advertising and scan state, bonds, free internal/PSRAM heap |
| `ble up` / `ble down` | Set `s.ble.enable` and reconcile |
| `ble peers` | One line per attached connection: address, role, MTU, bonded |
| `ble bonds` | How many of the bond slots are used |
| `ble pair [seconds]` | Open the pairing window (default 60; `0` closes it) |
| `ble forget <addr\|all>` | Drop one bond, or every bond |
| `ble scan` | Show what the scanner has heard; starts a debug scan if nobody else is scanning. `ble scan off` stops it and clears the table |

`ble` prints free internal DRAM as well as PSRAM because internal DRAM, not
flash, is what Bluetooth is scarce in: the controller's activities are
internal-only whatever the host's allocation mode says.

## Pairing a phone

1. `ble up` (or switch Bluetooth on in the settings pane).
2. `ble pair 60`, or press **Pair for 60 s**.
3. Pair from the phone's Bluetooth settings within that window. Pairing is Just
   Works — there is no passkey to compare.
4. `ble bonds` confirms it landed.

Outside the window no bond is written, and a peer that pairs anyway is unpaired
and dropped. `ble forget <addr>` makes a phone pair again; `ble forget all`
clears the store.

## Read next

- [INTERNALS.md](INTERNALS.md) — the task boundaries, why the host starts
  lazily and what that costs, the advertising rotation, the security split, the
  connection budget, and Wi-Fi coexistence.
