/**
 * ble — the Bluetooth Low Energy stack owner.
 *
 * NimBLE initialises once, advertises from one legacy instance, holds one bond
 * store and one connection budget. Those are single resources, so one straddle
 * owns them and every feature that wants Bluetooth is a consumer of this API —
 * the same relation spangap-net has with the Wi-Fi radio.
 *
 * Nothing here knows what rides the link. A consumer:
 *
 *   1. `requires: spangap/spangap-ble`, so this straddle's onInit() has run
 *      before its own,
 *   2. queues its GATT service with bleGattAdd() from onInit(),
 *   3. calls bleRegister() from its own task, so the events land there,
 *   4. asks for the radio with bleUp() and for a slice of the advertising
 *      instance with bleAdvRequest().
 *
 * TASKS. There are three, and the boundary between them is the contract:
 *
 *   - the NimBLE host task, which NimBLE owns. Every GAP/GATT callback runs
 *     here, including a consumer's own `access_cb` and any ble_gattc_*
 *     completion it registers.
 *   - the `ble` task, which owns this straddle's state: the host lifecycle,
 *     the advertising round-robin, the scan, the bond store and the connection
 *     table. Every call below that touches the radio posts to it and returns.
 *   - the consumer's task, where its bleRegister() callbacks are dispatched
 *     (via ITS aux, so the callback runs under the consumer's own itsPoll).
 *
 * A consumer's callbacks from this API therefore NEVER run in host context.
 * Consumer code that talks to NimBLE directly — a characteristic access_cb, a
 * ble_gattc_* completion — does run there, and must be a copy-and-post and
 * nothing else: no storage, no ITS, no logging, no teardown.
 */
#pragma once

#include "service.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct ble_gatt_svc_def;   /* host/ble_gatt.h — consumers include it themselves */

/** Boot-registered service: onInit() seeds the runtime readouts, registers the
 *  `ble` CLI verbs and spawns the ble task. The Bluetooth host itself is NOT
 *  started here — it starts on the first bleUp() from a consumer, so a build
 *  that stages this straddle without using it costs flash only. */
class BleService : public Service {
public:
    void onInit() override;
};

/* ─────────────── events ─────────────── */

enum {
    BLE_EV_UP,           /* host started and synced; the radio is live */
    BLE_EV_DOWN,         /* host stopped */
    BLE_EV_CONNECT,      /* a connection was established */
    BLE_EV_DISCONNECT,   /* a connection ended */
    BLE_EV_MTU,          /* ATT MTU exchanged on a connection */
    BLE_EV_SUBSCRIBE,    /* a central wrote a CCCD on one of our characteristics */
    BLE_EV_CFG_CHANGED,  /* an s.ble.* setting this straddle owns changed */
    BLE_EV_SCAN,         /* a scan result matched the active filter */
    BLE_EV_NOTIFY_TX,    /* an armed notification completed — see bleNotifyArm */
    BLE_EV_COUNT
};

/** One event. Every field is filled for every event; the ones an event has no
 *  answer for read zero. Valid for the duration of the callback only. */
typedef struct {
    uint8_t  event;        /* BLE_EV_* — one callback may serve several */
    uint8_t  addr[6];      /* peer address, NimBLE byte order (LSB first) */
    uint8_t  addrType;     /* BLE_ADDR_PUBLIC / _RANDOM / … */
    uint8_t  central;      /* 1 = we are the central on this connection */
    uint16_t conn;         /* NimBLE connection handle */
    uint16_t attr;         /* SUBSCRIBE: the characteristic's value handle */
    uint16_t mtu;          /* CONNECT / MTU: the ATT MTU in force */
    uint8_t  notify;       /* SUBSCRIBE: 1 = notifications now enabled */
    uint8_t  reason;       /* DISCONNECT: the BLE_HS_* / HCI reason */
    int8_t   rssi;         /* SCAN: advertisement RSSI in dBm */
    uint8_t  uuid128[16];  /* SCAN: the service UUID that matched */
    char     name[20];     /* SCAN: the advertised name, "" when there is none */
} ble_event_t;

typedef void (*ble_event_cb_t)(const ble_event_t* ev);

/** Register `cb` for `event`. Call from the task that should handle it, once
 *  per process — the registry is a fixed append-only array with no unregister,
 *  so re-registering on every start piles up duplicates.
 *
 *  Events are delivered to the registering task as ITS aux messages, so that
 *  task must already have an ITS inbox (itsClientInit / itsServerInit) and must
 *  reach itsPoll() for them to arrive.
 *
 *  BLE_EV_UP is LEVEL-REPLAYED: registering while the host is already up
 *  invokes `cb` synchronously, on the calling task, before this returns. UP
 *  handlers must therefore be idempotent. Every other event is edge-only. */
void bleRegister(int event, ble_event_cb_t cb);

/* ─────────────── lifecycle ─────────────── */

/** Ask for the Bluetooth host. `owner` is a static string naming the consumer
 *  (the same one handed to bleSlotReserve); wants are per owner, so one
 *  consumer standing down cannot take the radio from another. Posts to the ble
 *  task and returns; the host is brought up on the next reconcile pass.
 *
 *  `s.ble.enable` is the master gate: while it is 0 the want is RECORDED but
 *  the host stays down, and flipping the switch on brings the radio up with no
 *  further action from the consumer. Idempotent; BLE_EV_UP fires once the host
 *  has synced. */
void bleUp(const char* owner);

/** Release `owner`'s interest in the host. Posts to the ble task and returns.
 *  The host is torn down once no owner wants it. */
void bleDown(const char* owner);

/** True between BLE_EV_UP and BLE_EV_DOWN — i.e. the host is synced and NimBLE
 *  calls are legal. Cheap; call it, don't cache it. */
bool bleIsUp(void);

/** The adapter's current address: a NON-RESOLVABLE PRIVATE address, fresh at
 *  every host start, after every disconnect, and at latest every 15 minutes
 *  idle — so stale per-address state on a peer can never match this device
 *  twice, and its 00 top bits sort below every phone's (01), handing an
 *  address-comparison role election the dialling side against phones. Stable
 *  between rotations, but re-read your copy on every BLE_EV_UP — one fires
 *  after each rotation. Live connections survive a rotation; bonds do not
 *  (the peer recorded the old address), which stands until this straddle
 *  grows resolvable-private-address privacy. Returns false before the host
 *  has synced once. */
bool bleOwnAddr(uint8_t out[6], uint8_t* type);

/* ─────────────── GATT server ─────────────── */

/** Queue a GATT service definition for registration.
 *
 *  It is QUEUED, not registered: NimBLE's GATT registry only exists between
 *  nimble_port_init() and the host start, and the host starts lazily, so this
 *  records the pointer and the ble task feeds every recorded definition to
 *  ble_gatts_count_cfg()/ble_gatts_add_svcs() at bring-up. `svcs` must
 *  therefore be `static const` and outlive the process.
 *
 *  Call from onInit(). Adding after the host has started warns and forces a
 *  GATT reset plus a host restart, which drops every live connection.
 *
 *  Access flags on the consumer's own characteristics are what enforce
 *  security: the pairing POLICY is global (below), the DEMAND is per
 *  characteristic. A service whose characteristics carry no ENC/AUTHEN flag
 *  never triggers pairing. */
bool bleGattAdd(const struct ble_gatt_svc_def* svcs);

/* ─────────────── advertising ─────────────── */

#define BLE_ADV_NAME_MAX 16

/** One consumer's claim on the advertising instance. */
typedef struct {
    char    name[BLE_ADV_NAME_MAX];  /* "" => no name */
    uint8_t uuid128[16];             /* all-zero => no service UUID */
    uint8_t includeName;             /* 1 => name in the scan response */
    uint8_t connectable;             /* 1 => connectable undirected */
    uint8_t priority;                /* higher gets more of the rotation */
} ble_adv_req_t;

/** Claim a slice of the single legacy advertising instance. Returns a slot
 *  index (>= 0) or -1 when the table is full.
 *
 *  There is exactly one legacy instance and one 31-byte payload, so claims are
 *  ROUND-ROBINED rather than merged: each slot advertises alone for a period
 *  before the next takes over. A central dials an ADDRESS, not a payload, so a
 *  peer that already knows this device connects during whichever slot happens
 *  to be up, as long as that slot is connectable; only DISCOVERY of a
 *  not-currently-advertised payload pays the rotation. With no slots claimed,
 *  advertising stops. */
int  bleAdvRequest(const ble_adv_req_t* req);

/** Drop a claim. The rotation closes up around it. */
void bleAdvRelease(int slot);

/* ─────────────── scanning ─────────────── */

typedef struct {
    uint8_t  uuid128[16];  /* filter; all-zero => report every advertiser */
    int8_t   minRssi;      /* floor in dBm; 0 => no floor */
    uint16_t intervalMs;   /* scan interval (0 => 100) */
    uint16_t windowMs;     /* scan window — the duty cycle (0 => 30) */
    uint8_t  activeScan;   /* 1 => send scan requests, so names arrive too */
} ble_scan_req_t;

/** Start (or re-parameterise) the scan. There is one scanner; the last caller
 *  wins. Results are delivered as BLE_EV_SCAN to every task registered for it,
 *  with the 128-bit-UUID parse over the raw advertisement bytes already done
 *  here. The window is halved automatically while any connection is attached,
 *  because scanning is the expensive half of the 2.4 GHz coexistence. */
bool bleScanStart(const ble_scan_req_t* req);

/** Stop the scan. */
void bleScanStop(void);

/* ─────────────── connections ─────────────── */

/** Dial `addr` as a central. Posts to the ble task and returns true if the
 *  attempt was queued; refused while the host is down, another dial is in
 *  flight, or the connection table is full. Success arrives as BLE_EV_CONNECT;
 *  a refused or timed-out dial arrives as BLE_EV_DISCONNECT whose conn is
 *  0xFFFF (none) and whose addr is the dialled address. */
bool bleConnect(const uint8_t addr[6], uint8_t addrType, uint32_t timeoutMs);

/** Terminate a connection. Safe from any task. */
void bleDisconnect(uint16_t conn);

/** The ATT MTU in force on `conn`, or 0 if it is not ours. */
uint16_t bleConnMtu(uint16_t conn);

/** Ask for a connection interval, in units of 1.25 ms. A sleepy interval kills
 *  a session whose peer runs an inactivity watchdog, so a latency-sensitive
 *  consumer states its needs once, on connect. Safe from any task. */
bool bleConnParams(uint16_t conn, uint16_t minItvl, uint16_t maxItvl,
                   uint16_t latency, uint16_t supervisionMs);

/** A notification or indication arrived from a peer, on a connection this
 *  device dialled. The connection's GAP callback belongs to this straddle, so
 *  a central-role consumer cannot see BLE_GAP_EVENT_NOTIFY_RX itself and takes
 *  delivery here instead.
 *
 *  RUNS IN NIMBLE HOST CONTEXT, and is the one place a consumer's code does.
 *  Copy the bytes and post them to your own task — no storage, no ITS, no
 *  logging, no teardown. `data` is invalid the moment it returns. */
typedef void (*ble_notify_rx_cb_t)(uint16_t conn, uint16_t attr,
                                   const uint8_t* data, uint16_t len);

/** Install the handler above. One per process; the last registration wins. */
void bleOnNotifyRx(ble_notify_rx_cb_t cb);

/** Ask for one BLE_EV_NOTIFY_TX when a notification on `conn` next completes.
 *
 *  This is the GATT server's flow control. ble_gatts_notify_custom() returns
 *  BLE_HS_ENOMEM when NimBLE's buffers are full, and the answer to that is to
 *  stop producing and arm this — never to drop, because a byte dropped out of
 *  a stream corrupts everything after it and the peer has no way to resync.
 *  One-shot, cleared when it fires; re-arm for the next wake. Deliberately the
 *  same shape as itsSetFreeNotify. */
void bleNotifyArm(uint16_t conn);

/** Reserve `n` of the connection budget for `owner` (a static string). The
 *  total is CONFIG_BT_NIMBLE_MAX_CONNECTIONS, and the reservation is refused
 *  when the other owners' claims no longer fit beside it.
 *
 *  It is an AGREEMENT, checked here and kept by each consumer capping its own
 *  peers at what it reserved — not a runtime gate. An inbound connection
 *  carries no owner, so there is nothing to charge it to; what actually leaves
 *  a slot for another consumer is that consumer's neighbour not dialling an
 *  eighth peer. Re-reserving for the same owner replaces its count. */
bool bleSlotReserve(const char* owner, int n);

/** Drop `owner`'s reservation. */
void bleSlotRelease(const char* owner);

/** Connections currently attached. */
int  bleConnCount(void);

/* ─────────────── security ─────────────── */

/** Open the pairing window for `seconds` (0 closes it immediately). New bonds
 *  are accepted only while it is open: outside it the stack offers no bonding,
 *  and a peer that pairs anyway is unpaired and dropped. Existing bonds are
 *  unaffected and reconnect at any time. */
void blePairingWindow(uint32_t seconds);

/** Seconds left on the pairing window; 0 when closed. */
uint32_t blePairingLeft(void);

/** Forget one bond. Returns false if that address held none; true once the
 *  forget is queued. Posts to the ble task, which owns the bond store and its
 *  persistence — mutations stay serialized there. */
bool bleForget(const uint8_t addr[6]);

/** Forget every bond. Posts to the ble task, as above. */
void bleForgetAll(void);

/** Bonds currently held (persisted to /state/ble/bonds.bin across reboots). */
int  bleBondCount(void);
