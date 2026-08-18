/**
 * ble_priv.h — shared state of the ble task. Included by ble.cpp, ble_gap.cpp,
 * ble_sec.cpp and ble_cli.cpp; never by a consumer.
 *
 * Everything here is owned by the ble task unless the comment says otherwise.
 * The two exceptions are s_lock, which guards the request tables the public API
 * writes from a caller's task, and the NimBLE host callbacks, which run in host
 * context and may only touch what is marked as safe there.
 */
#pragma once

#include "ble.h"
#include "spangap.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_bt.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_hs_adv.h"
#include "host/ble_hs_id.h"
#include "host/ble_uuid.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

/* NimBLE's bond store ships no header for its entry point — IDF's own
 * examples declare it at the call site, and it is C. With
 * CONFIG_BT_NIMBLE_NVS_PERSIST off it is RAM-only; ble_store.cpp is the
 * persistence around it. */
extern "C" void ble_store_config_init(void);

/* Table sizes. Advertising and scanning are single resources; the numbers here
 * bound the consumer side of them, not the radio's. */
#define BLE_MAX_ADV_SLOTS   4
#define BLE_MAX_GATT_SVCS   4
#define BLE_MAX_EV_CBS      20   /* iface-ble and rnode-ble register 7 each */
#define BLE_MAX_RESERVERS   4
#define BLE_MAX_WANTS       4
#define BLE_MAX_CONNS       CONFIG_BT_NIMBLE_MAX_CONNECTIONS

/** ITS aux port on a consumer's task that carries ble_event_t. Private to this
 *  straddle: bleRegister() installs the trampoline, consumers never see it. */
#define BLE_AUX_PORT_EVENT  0x424C   /* 'BL' */

/* ─────────────── inbox: what a caller's task asks the ble task for ───────────
 *
 * Every public call that touches the radio sets a want under s_lock and wakes
 * the ble task; the ble task reconciles. There is no request queue and no
 * completion — reconciliation against a desired state is idempotent, which is
 * what makes a lost wake harmless. */

struct ble_adv_slot_t {
    bool          used;
    ble_adv_req_t req;
};

struct ble_conn_t {
    bool     used;
    uint16_t conn;
    uint8_t  addr[6];
    uint8_t  addrType;
    bool     central;      /* we dialled it */
    bool     wasBonded;    /* the peer was already bonded when it connected */
    uint16_t mtu;
};

struct ble_reserver_t {
    const char* owner;     /* static string; nullptr = free slot */
    int         n;
};

/* The cross-file state lives in a namespace and is pulled in unqualified
 * below. Plain `s_`-prefixed globals would be external symbols in a flat link
 * of forty straddles, where `s_enabled` and `s_configDirty` are names half of
 * them use for something file-local; a namespace keeps the familiar names
 * without putting them in that shared pool. */
namespace spangap_ble {

extern SemaphoreHandle_t s_lock;

/* Desired state (written under s_lock by any task, read by the ble task). */
extern volatile bool s_configDirty;
extern volatile bool s_advDirty;
extern volatile bool s_scanDirty;

extern ble_adv_slot_t  s_adv[BLE_MAX_ADV_SLOTS];
extern ble_scan_req_t  s_scanReq;
extern bool            s_scanWanted;
extern ble_reserver_t  s_reserve[BLE_MAX_RESERVERS];

/* Live state (ble task + host callbacks). */
extern bool       s_dialInFlight;  /* ble_gap_connect issued, outcome pending —
                                    * the scanner must not restart under it */
extern bool       s_hostUp;        /* nimble_port_init done and synced */
extern bool       s_enabled;       /* s.ble.enable */
extern int        s_txPowerDbm;
extern uint8_t    s_ownAddr[6];
extern uint8_t    s_ownAddrType;
extern ble_conn_t s_conns[BLE_MAX_CONNS];

/* Counters, for the status readouts. */
extern uint32_t s_connTotal, s_advCycles, s_scanSeen, s_pairFails;
/* Host events lost to a full queue. Written in HOST CONTEXT (hevPost),
 * reported by the ble task when it grows. */
extern volatile uint32_t s_hevDrops;

}   /* namespace spangap_ble */
using namespace spangap_ble;

/** Wake the ble task. Safe from any task, and from host context. */
void bleWake(void);

/** True while any owner's bleUp() want stands. */
bool bleWanted(void);

/** A dial failed or timed out; fires the public BLE_EV_DISCONNECT with the
 *  dialled address and no connection handle. Called on the ble task only. */
void bleDialFailed(uint8_t reason);

/** Mark the adapter address for rotation; the task loop performs it on the
 *  same pass. Called on the ble task only (the gap drain, on disconnect). */
void bleAddrRotateDue(void);

/** Dispatch an event to every registered task. Called on the ble task only;
 *  host-context code posts to the ble task first. */
void bleFire(const ble_event_t* ev);

/** Internal event codes, carried in the same queue as the public ones. bleFire
 *  fans out only the codes below BLE_EV_COUNT; these are consumed by the ble
 *  task and never leave this straddle. */
enum {
    BLE_IEV_PAIRED = BLE_EV_COUNT,  /* pairing finished; `reason` is the status */
    BLE_IEV_ADV_DONE,               /* the advertising instance stopped by itself */
    BLE_IEV_DIAL_FAILED,            /* a dial failed; `reason` is the status.
                                     * NimBLE's failure event carries a ZEROED
                                     * handle — 0 is a valid live connection, so
                                     * the handle must never be read from it. */
};

/** One advertiser the scan has seen recently, for the `ble scan` readout. */
#define BLE_MAX_SEEN 16
struct ble_seen_t {
    bool     used;
    uint8_t  addr[6];
    uint8_t  addrType;
    int8_t   rssi;
    uint32_t lastMs;
    uint8_t  uuid128[16];
    char     name[20];
};

/* ── ble_gap.cpp ── */
int  bleGapEvent(struct ble_gap_event* event, void* arg);
bool bleGapQueueInit(void);      /* the host→ble-task marshal queue */
void bleGapDrain(void);          /* run every queued host event on the ble task */
int  bleSeenList(ble_seen_t* out, int max);
void bleSeenClear(void);
void bleAdvReconcile(void);      /* advertising round-robin, one step per call */
void bleAdvStop(void);
void bleScanReconcile(void);
void bleScanStopNow(void);
void bleGapOnHostDown(void);     /* forget every radio-side flag after teardown */
uint32_t bleAdvNextDueMs(void);  /* ms until the rotation wants another step */

/* ── ble_sec.cpp ── */
void bleSecConfigure(void);          /* fill ble_hs_cfg; called before host start */
void blePairingTick(void);           /* close the window when it expires */
bool bleIsBonded(const uint8_t addr[6], uint8_t addrType);
bool bleBondExists(const uint8_t addr[6]);   /* any address type; RAM read only */
/* BLE TASK ONLY: bond mutations stay serialized on the owner of the bond
 * store's persistence, so an unpair can never race the snapshot that
 * ble_store.cpp writes. The public bleForget/bleForgetAll post here. */
bool bleForgetNow(const uint8_t addr[6]);
void bleForgetAllNow(void);
void bleSecOnPairingComplete(uint16_t conn, int status);
void bleSecPublish(void);

/* ── ble_store.cpp (all of it runs on the ble task) ── */
void     bleStoreLoad(void);    /* restore bonds from /state; after ble_store_config_init */
void     bleStoreTick(void);    /* persist after the debounce; every task pass */
void     bleStoreFlush(void);   /* persist NOW if dirty; before a host restart's init */
void     bleStoreDirty(void);   /* a bond or CCCD changed */
uint32_t bleStoreDueMs(void);   /* ms until a pending persist wants the task awake; 0 = clean */

/* ── ble_cli.cpp ── */
void bleCliRegister(void);

/* ── shared helpers ── */
ble_conn_t* bleConnFind(uint16_t conn);
int  bleSlotsCommitted(const char* exceptOwner);
void bleFmtAddr(const uint8_t addr[6], char* out, size_t len);
