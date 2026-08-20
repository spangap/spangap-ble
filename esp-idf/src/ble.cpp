/**
 * ble — the Bluetooth Low Energy stack owner: host lifecycle, the GATT
 * registry, the event fan-out, the connection budget and the settings
 * reconcile. Advertising and scanning are in ble_gap.cpp, the security policy
 * in ble_sec.cpp, the CLI in ble_cli.cpp.
 *
 * The host starts LAZILY — on the first reconcile pass in which a consumer has
 * called bleUp() and s.ble.enable is 1 — not at boot. NimBLE's GATT registry
 * only exists between nimble_port_init() and the host start, which is why
 * bleGattAdd() records a pointer here and this file feeds the whole list to
 * ble_gatts_add_svcs() during bring-up.
 */
#include "ble_priv.h"

#include "esp_heap_caps.h"
#include "esp_mac.h"

#include <cstdio>
#include <cstring>

static const char* TAG = "ble";

static TaskHandle_t s_task = nullptr;

/* ─────────────── state ─────────────── */

namespace spangap_ble {

SemaphoreHandle_t s_lock = nullptr;

volatile bool s_configDirty = true;
volatile bool s_advDirty    = false;
volatile bool s_scanDirty   = false;

ble_adv_slot_t s_adv[BLE_MAX_ADV_SLOTS] = {};
ble_scan_req_t s_scanReq = {};
bool           s_scanWanted = false;
ble_reserver_t s_reserve[BLE_MAX_RESERVERS] = {};

bool       s_dialInFlight = false;
bool       s_hostUp       = false;
bool       s_enabled      = false;
int        s_txPowerDbm   = 9;
uint8_t    s_ownAddr[6]   = {};
uint8_t    s_ownAddrType  = BLE_OWN_ADDR_PUBLIC;
ble_conn_t s_conns[BLE_MAX_CONNS] = {};

uint32_t s_connTotal = 0, s_advCycles = 0, s_scanSeen = 0, s_pairFails = 0;
volatile uint32_t s_hevDrops = 0;

}   /* namespace spangap_ble */

/* Set in host context by the sync callback; consumed by the ble task. */
static volatile bool s_synced      = false;
/* True between nimble_port_init and nimble_port_deinit — the flag hostStop
 * keys on, because s_hostUp only becomes true at sync and a host stopped in
 * the started-but-unsynced window must still be torn down. */
static bool          s_hostStarted = false;
/* When to try hostStart again after a failure (0 = no retry due). The
 * controller wants a large contiguous internal block, and at the boot peak —
 * Wi-Fi, LCD and USB all initialising at once — it may not get one. That
 * pressure passes, so a failed start is retried rather than final. */
static uint32_t      s_startRetryMs = 0;

/* The controller's internal DRAM, earmarked at onInit — BEFORE the boot storm
 * fragments the heap — and released into the controller's hands at hostStart.
 * Sized to the whole controller need, so the contiguous exchange-memory arena
 * is guaranteed by construction rather than hoped for. Re-grabbed after a
 * hostStop for the same reason. Held only when s.ble.enable was set at boot;
 * a build running with Bluetooth off keeps the RAM. */
#define BLE_CTRL_RESERVE (44 * 1024)
static void* s_ctrlReserve = nullptr;

static void ctrlReserveGrab(void) {
    if (!s_ctrlReserve)
        s_ctrlReserve = heap_caps_malloc(BLE_CTRL_RESERVE,
                                         MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
}
/* A dial the ble task owes the radio, parked here by bleConnect(). */
static volatile bool s_dialPending = false;
static uint8_t       s_dialAddr[6] = {};
static uint8_t       s_dialType    = 0;
static uint32_t      s_dialTimeout = 0;
/* The dial currently on the radio (ble task only) — what a failure names,
 * since a consumer may park the next dial before this one's outcome arrives. */
static uint8_t       s_curDialAddr[6] = {};
static uint8_t       s_curDialType    = 0;
static uint32_t      s_dialForcedMs   = 0;   /* dial watchdog deadline */

/* A bond forget parked for the ble task (one slot; the operator action this
 * carries does not queue up). Bond mutations run on the ble task only, which
 * owns the store's persistence — see bleForgetNow in ble_sec.cpp. */
static volatile bool s_forgetPending  = false;
static bool          s_forgetAll      = false;
static uint8_t       s_forgetAddr[6]  = {};

/* The address rotates after EVERY disconnect (marked here by the gap drain,
 * executed by the task loop), with a 15-minute timer as the idle floor. See
 * addrGenerate for what rotation is for and what it costs. */
#define BLE_ADDR_ROTATE_MS (15 * 60 * 1000)
static uint32_t s_lastRotateMs = 0;
static bool     s_rotateDue    = false;

void bleAddrRotateDue(void) { s_rotateDue = true; }

/* Owners that want the host — the same per-owner shape as s_reserve, so one
 * consumer standing down cannot take the radio from another. Written under
 * s_lock; a want survives s.ble.enable=0 so the master switch restores the
 * radio on its own. */
static const char* s_wants[BLE_MAX_WANTS] = {};

bool bleWanted(void) {
    for (auto& w : s_wants) if (w) return true;
    return false;
}

/* Queued GATT service definitions (bleGattAdd). Pointers only — the consumer
 * owns the storage and must keep it alive forever. */
static const struct ble_gatt_svc_def* s_svcs[BLE_MAX_GATT_SVCS] = {};
static int s_svcCount = 0;
/* How many of them the running host was built with. A mismatch is what tells
 * the reconcile pass that a late bleGattAdd needs the host rebuilt. */
static int s_svcRegistered = 0;

/* ─────────────── event registry ───────────────
 *
 * Fixed, append-only, no unregister — netRegister's contract, for the same
 * reason: a straddle that re-registered on every start would pile up
 * duplicates. Each entry remembers the task that registered it, and delivery
 * is an ITS aux message to that task, so a consumer's handler runs under its
 * own itsPoll and never in host or ble-task context. */

static struct {
    TaskHandle_t   task;
    ble_event_cb_t cb;
    uint8_t        event;
} s_evs[BLE_MAX_EV_CBS];
static int s_evCount = 0;

/* Runs on the consumer's task, from its itsPoll. */
static void bleAuxTrampoline(TaskHandle_t /*sender*/, const void* data, size_t len) {
    if (len < sizeof(ble_event_t)) return;
    ble_event_t ev;
    std::memcpy(&ev, data, sizeof(ev));
    TaskHandle_t me = xTaskGetCurrentTaskHandle();
    for (int i = 0; i < s_evCount; i++)
        if (s_evs[i].task == me && s_evs[i].event == ev.event) s_evs[i].cb(&ev);
}

void bleRegister(int event, ble_event_cb_t cb) {
    if (event < 0 || event >= BLE_EV_COUNT || !cb) return;
    /* Consumers register from their own tasks, concurrently — the append is
     * what needs the lock, the replay below must run outside it. */
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool full = s_evCount >= BLE_MAX_EV_CBS;
    if (!full) {
        s_evs[s_evCount].task  = xTaskGetCurrentTaskHandle();
        s_evs[s_evCount].cb    = cb;
        s_evs[s_evCount].event = (uint8_t)event;
        s_evCount++;
    }
    xSemaphoreGive(s_lock);
    if (full) { warn("event registry full"); return; }
    /* Idempotent per port — installing it again for a second registration on
     * the same task simply replaces the same function pointer. */
    itsOnAux(BLE_AUX_PORT_EVENT, bleAuxTrampoline);

    /* Level-replay UP to a late registrant, synchronously on its own task, so
     * registration order against bring-up does not matter. DOWN and the edges
     * stay edge-only: replaying a teardown to a handler that never set up is
     * wrong. */
    if (event == BLE_EV_UP && s_hostUp) {
        ble_event_t ev = {};
        ev.event = BLE_EV_UP;
        cb(&ev);
    }
}

void bleFire(const ble_event_t* ev) {
    /* One message per distinct task, however many callbacks it registered —
     * the trampoline fans out locally. */
    TaskHandle_t sent[BLE_MAX_EV_CBS];
    int nSent = 0;
    for (int i = 0; i < s_evCount; i++) {
        if (s_evs[i].event != ev->event) continue;
        bool dup = false;
        for (int j = 0; j < nSent; j++) if (sent[j] == s_evs[i].task) { dup = true; break; }
        if (dup) continue;
        sent[nSent++] = s_evs[i].task;
        if (!itsSendAuxByTaskHandle(s_evs[i].task, BLE_AUX_PORT_EVENT,
                                    ev, sizeof(*ev), pdMS_TO_TICKS(50)))
            warn("event %u undelivered (consumer inbox full)", (unsigned)ev->event);
    }
}

void bleWake(void) { if (s_task) xTaskNotifyGive(s_task); }

/* ─────────────── small helpers ─────────────── */

void bleFmtAddr(const uint8_t addr[6], char* out, size_t len) {
    /* NimBLE carries addresses least-significant byte first; print them the way
     * every phone and scanner shows them, most significant first. */
    snprintf(out, len, "%02X:%02X:%02X:%02X:%02X:%02X",
             addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
}

ble_conn_t* bleConnFind(uint16_t conn) {
    for (auto& c : s_conns) if (c.used && c.conn == conn) return &c;
    return nullptr;
}

int bleConnCount(void) {
    int n = 0;
    for (auto& c : s_conns) if (c.used) n++;
    return n;
}

int bleSlotsCommitted(const char* exceptOwner) {
    int n = 0;
    for (auto& r : s_reserve)
        if (r.owner && (!exceptOwner || strcmp(r.owner, exceptOwner) != 0)) n += r.n;
    return n;
}

/* ─────────────── publish ─────────────── */

static const char* stateWord(void) {
    if (!s_enabled)  return "disabled";
    if (s_hostUp)    return "up";
    if (bleWanted()) return "starting";
    return "idle";
}

static void publishState(void) {
    storageBegin();
    storageSet("ble.state", stateWord());
    storageSet("ble.state_text", stateWord());
    storageSet("ble.up", s_hostUp ? 1 : 0);
    storageEnd();
}

static void publishStats(void) {
    char buf[128];
    int n = bleConnCount();
    storageBegin();
    if (n == 0) safeStrncpy(buf, "none", sizeof(buf));
    else {
        int o = snprintf(buf, sizeof(buf), "%d connected", n);
        for (auto& c : s_conns) {
            if (!c.used || o >= (int)sizeof(buf) - 20) continue;
            char a[20]; bleFmtAddr(c.addr, a, sizeof(a));
            o += snprintf(buf + o, sizeof(buf) - o, " \xC2\xB7 %s", a);
        }
    }
    storageSet("ble.peers", buf);
    snprintf(buf, sizeof(buf), "%d bonded", bleBondCount());
    storageSet("ble.bonds", buf);
    uint32_t left = blePairingLeft();
    if (left) snprintf(buf, sizeof(buf), "open, %us left", (unsigned)left);
    else      safeStrncpy(buf, "closed", sizeof(buf));
    storageSet("ble.pairing_text", buf);
    storageSet("ble.pairing_open", left ? 1 : 0);
    storageEnd();
}

/* ─────────────── NimBLE host callbacks (host context) ─────────────── */

static void onHostSync(void) {
    /* The controller is up and the identity address is settled. Nothing but a
     * flag here — the ble task does the work. */
    s_synced = true;
    bleWake();
}

static void onHostReset(int /*reason*/) {
    /* The controller reset under us. NimBLE re-syncs on its own and calls
     * onHostSync again; the ble task re-applies advertising and the scan from
     * its desired state, so there is nothing to remember across it. */
    s_advDirty = true;
    s_scanDirty = true;
    bleWake();
}

static void hostTaskFn(void*) {
    nimble_port_run();              /* returns only after nimble_port_stop() */
    nimble_port_freertos_deinit();
}

/* ─────────────── host bring-up / tear-down (ble task) ─────────────── */

static bool hostStart(void) {
    if (s_hostStarted) return true;

    /* Hand the earmarked block back to the heap; the controller's own mallocs
     * land in the hole. Then verify — the controller does not fail cleanly (a
     * malloc failure mid-init asserts inside the library and the interrupt
     * watchdog reboots the chip), so a short heap must become a deferred
     * retry here, never an attempt. */
    if (s_ctrlReserve) { heap_caps_free(s_ctrlReserve); s_ctrlReserve = nullptr; }
    size_t freeNow = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    size_t biggest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (freeNow < 42 * 1024 || biggest < 26 * 1024) {
        warn("host start deferred: %u B internal free, largest block %u B — "
             "the controller needs ~42 KB with one 26 KB block",
             (unsigned)freeNow, (unsigned)biggest);
        ctrlReserveGrab();       /* hold what there is against fragmentation */
        return false;
    }

    esp_err_t e = nimble_port_init();
    if (e != ESP_OK) {
        err("nimble_port_init: %s", esp_err_to_name(e));
        ctrlReserveGrab();
        return false;
    }
    s_hostStarted = true;

    ble_hs_cfg.sync_cb  = onHostSync;
    ble_hs_cfg.reset_cb = onHostReset;
    bleSecConfigure();

    ble_svc_gap_init();
    ble_svc_gatt_init();

    /* Every queued consumer service, in registration order. count_cfg sizes
     * NimBLE's attribute tables; add_svcs records the definition for the
     * ble_gatts_start() that ble_hs_start() runs when the host task comes up. */
    for (int i = 0; i < s_svcCount; i++) {
        int rc = ble_gatts_count_cfg(s_svcs[i]);
        if (rc == 0) rc = ble_gatts_add_svcs(s_svcs[i]);
        if (rc != 0) err("gatt service %d rejected (rc=%d)", i, rc);
    }
    s_svcRegistered = s_svcCount;

    /* The GAP device name is what a phone's bond list shows forever after
     * pairing, so it must tell devices apart: the operator's hostname when it
     * says something, otherwise the project name suffixed with the adapter
     * address's tail — the Wi-Fi AP SSID's shape. A bare shared project name
     * would make every unnamed node in a drawer indistinguishable. */
    char name[32];
    storageGetStr("sys.hostname", name, sizeof(name), "");
    if (!name[0] || strcmp(name, CONFIG_SPANGAP_PROJECT_NAME) == 0) {
        uint8_t mac[6] = {};
        esp_read_mac(mac, ESP_MAC_BT);
        snprintf(name, sizeof(name), "%s_%02x%02x",
                 CONFIG_SPANGAP_PROJECT_NAME, mac[4], mac[5]);
    }
    ble_svc_gap_device_name_set(name);

    /* Flush any bond change still inside its debounce, because config_init
     * ZEROES the store's RAM counts on every call — the file must be current
     * before it does, and bleStoreLoad() restores from it after. */
    bleStoreFlush();
    ble_store_config_init();
    bleStoreLoad();
    nimble_port_freertos_init(hostTaskFn);

    /* What the controller and host actually took — the number BLE_CTRL_RESERVE
     * is sized from, so it is kept visible. */
    size_t freeAfter = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    info("controller+host took %u B internal, %u B left",
         (unsigned)(freeNow - freeAfter), (unsigned)freeAfter);

    /* Transmit power is a controller setting and outlives the host task, but it
     * is reset by nimble_port_deinit, so it is re-applied on every start. */
    int lvl = (s_txPowerDbm + 24) / 3;
    if (lvl < 0) lvl = 0;
    if (lvl > ESP_PWR_LVL_P20) lvl = ESP_PWR_LVL_P20;
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, (esp_power_level_t)lvl);

    info("host starting (%d gatt service%s, tx %d dBm)",
         s_svcCount, s_svcCount == 1 ? "" : "s", s_txPowerDbm);
    return true;
}

/* esp-nimble keeps this in a private header (ble_hs_resolv_priv.h); the
 * symbol is exported from the archive. Regenerates the local RPA and sets it
 * in the controller — the host-based-privacy analogue of set_rnd. */
extern "C" int ble_hs_gen_own_private_rnd(void);

/* True once ble_hs_pvcy_rpa_config() succeeded for the current host run;
 * cleared at host stop, since a restart reinitialises the privacy machinery. */
static bool s_pvcyOn = false;

/* A fresh RESOLVABLE private address at every host start, after every
 * disconnect, and at latest every BLE_ADDR_ROTATE_MS idle — the cadence
 * non-resolvable rotation had, because peers key per-address state by our
 * address and clean it unreliably; a fresh address per connection — what
 * phones' radios do natively — means stale entries on the far side can never
 * match us twice. The difference privacy makes is that a BONDED peer holds
 * the identity resolving key (NimBLE distributes it at SMP key exchange, and
 * it persists with the bonds) and resolves every rotation back to one
 * identity, so bonds survive what used to strand them; strangers still see
 * an unlinkable fresh address. esp-nimble's host-based privacy keeps the
 * generation in the host — the RPA rides the ordinary set-random-address
 * route, own_addr_type stays BLE_OWN_ADDR_RANDOM everywhere, and this code
 * always knows the current value. The identity a bond records is the chip's
 * public address, stable across reboots for free.
 *
 * Cost, accepted: an RPA's top bits are 01, the same as every phone's, so a
 * consumer electing who-dials by address comparison gets a coin flip per
 * rotation window instead of the old always-we-dial (non-resolvable 00 sat
 * below everything). Falls back to the old non-resolvable rotation — fresh
 * addresses, stranded bonds — if privacy fails to start. */
static bool addrGenerate(void) {
    if (!s_pvcyOn && ble_hs_pvcy_rpa_config(NIMBLE_HOST_ENABLE_RPA) == 0) {
        s_pvcyOn = true;
        /* A first boot just minted the IRK through the store callbacks; the
         * persist path diffs, so this is free when it already existed. */
        bleStoreDirty();
    } else if (s_pvcyOn && ble_hs_gen_own_private_rnd() != 0) {
        return false;
    }
    if (!s_pvcyOn) {
        ble_addr_t rnd;
        if (ble_hs_id_gen_rnd(1, &rnd) != 0 || ble_hs_id_set_rnd(rnd.val) != 0)
            return false;
    }
    if (ble_hs_id_copy_addr(BLE_ADDR_RANDOM, s_ownAddr, nullptr) != 0)
        return false;
    s_ownAddrType = BLE_OWN_ADDR_RANDOM;
    return true;
}

/* Called once the host has synced — the first moment NimBLE calls are legal. */
static void hostSynced(void) {
    s_hostUp = true;
    if (!addrGenerate()) {
        warn("random address rejected — falling back to the public address");
        ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, s_ownAddr, nullptr);
        s_ownAddrType = BLE_OWN_ADDR_PUBLIC;
    }
    s_lastRotateMs = millis();
    char a[20]; bleFmtAddr(s_ownAddr, a, sizeof(a));
    info("up: %s, %d bond%s", a, bleBondCount(), bleBondCount() == 1 ? "" : "s");
    s_advDirty = true;
    s_scanDirty = true;
    publishState();
    ble_event_t ev = {};
    ev.event = BLE_EV_UP;
    std::memcpy(ev.addr, s_ownAddr, 6);
    ev.addrType = s_ownAddrType;
    bleFire(&ev);
}

static void hostStop(void) {
    if (!s_hostStarted) return;

    bleAdvStop();
    bleScanStopNow();
    for (auto& c : s_conns) if (c.used) ble_gap_terminate(c.conn, BLE_ERR_REM_USER_CONN_TERM);
    std::memset(s_conns, 0, sizeof(s_conns));

    int rc = nimble_port_stop();
    if (rc == 0) nimble_port_deinit();
    else         warn("nimble_port_stop: rc=%d", rc);

    s_hostStarted = false;
    s_hostUp = false;
    s_synced = false;
    s_pvcyOn = false;        /* a restart reinitialises the privacy machinery */
    ctrlReserveGrab();       /* earmark the freed RAM for the next start */
    bleGapOnHostDown();
    info("stopped");
    publishState();

    ble_event_t ev = {};
    ev.event = BLE_EV_DOWN;
    bleFire(&ev);
}

/* Registering a GATT service after the host has started means tearing the
 * registry down and building it again, which drops every connection. Legal,
 * but only the reconcile path should ever reach it. */
static void hostRestartForGatt(void) {
    warn("gatt service added after host start — restarting the host, "
         "connections will drop");
    hostStop();
    hostStart();
}

/* ─────────────── public API ─────────────── */

/* bleUp/bleDown express a want, post it, and return. No radio work ever runs
 * on a caller's task. The want is recorded even while s.ble.enable is 0 — the
 * master switch is applied at reconcile time, so flipping it on restores the
 * radio with no action from the consumer. */
void bleUp(const char* owner) {
    if (!owner) return;
    bool full = true;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const char** free_ = nullptr;
    for (auto& w : s_wants) {
        if (w && strcmp(w, owner) == 0) { full = false; break; }
        if (!w && !free_) free_ = &w;
    }
    if (full && free_) { *free_ = owner; full = false; }
    xSemaphoreGive(s_lock);
    if (full) { warn("%s: want table full", owner); return; }
    s_configDirty = true;
    bleWake();
}

void bleDown(const char* owner) {
    if (!owner) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (auto& w : s_wants)
        if (w && strcmp(w, owner) == 0) w = nullptr;
    xSemaphoreGive(s_lock);
    s_configDirty = true;
    bleWake();
}

bool bleIsUp(void) { return s_hostUp; }

bool bleOwnAddr(uint8_t out[6], uint8_t* type) {
    if (!s_hostUp) return false;
    std::memcpy(out, s_ownAddr, 6);
    if (type) *type = s_ownAddrType;
    return true;
}

bool bleGattAdd(const struct ble_gatt_svc_def* svcs) {
    if (!svcs) return false;
    if (s_svcCount >= BLE_MAX_GATT_SVCS) { err("gatt registry full"); return false; }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_svcs[s_svcCount++] = svcs;
    bool late = s_hostUp;
    xSemaphoreGive(s_lock);
    if (late) { s_configDirty = true; bleWake(); }   /* forces the restart below */
    return true;
}

int bleAdvRequest(const ble_adv_req_t* req) {
    if (!req) return -1;
    int slot = -1;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < BLE_MAX_ADV_SLOTS; i++)
        if (!s_adv[i].used) { slot = i; s_adv[i].used = true; s_adv[i].req = *req; break; }
    xSemaphoreGive(s_lock);
    if (slot < 0) { warn("advertising table full"); return -1; }
    s_advDirty = true;
    bleWake();
    return slot;
}

void bleAdvRelease(int slot) {
    if (slot < 0 || slot >= BLE_MAX_ADV_SLOTS) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_adv[slot].used = false;
    xSemaphoreGive(s_lock);
    s_advDirty = true;
    bleWake();
}

bool bleScanStart(const ble_scan_req_t* req) {
    if (!req) return false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_scanReq    = *req;
    s_scanWanted = true;
    xSemaphoreGive(s_lock);
    s_scanDirty = true;
    bleWake();
    return true;
}

void bleScanStop(void) {
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_scanWanted = false;
    xSemaphoreGive(s_lock);
    s_scanDirty = true;
    bleWake();
}

bool bleConnect(const uint8_t addr[6], uint8_t addrType, uint32_t timeoutMs) {
    if (!s_hostUp) return false;
    if (bleConnCount() >= BLE_MAX_CONNS) return false;
    bool queued = false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (!s_dialPending) {
        std::memcpy(s_dialAddr, addr, 6);
        s_dialType    = addrType;
        s_dialTimeout = timeoutMs ? timeoutMs : 7000;
        s_dialPending = true;
        queued = true;
    }
    xSemaphoreGive(s_lock);
    if (queued) bleWake();
    return queued;
}

/* Bond mutations stay on the ble task — see s_forgetPending. The return
 * answers "did that address hold a bond", from the RAM store. */
bool bleForget(const uint8_t addr[6]) {
    if (!s_hostUp || !bleBondExists(addr)) return false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    std::memcpy(s_forgetAddr, addr, 6);
    s_forgetAll     = false;
    s_forgetPending = true;
    xSemaphoreGive(s_lock);
    bleWake();
    return true;
}

void bleForgetAll(void) {
    if (!s_hostUp) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_forgetAll     = true;
    s_forgetPending = true;
    xSemaphoreGive(s_lock);
    bleWake();
}

void bleDialFailed(uint8_t reason) {
    char a[20]; bleFmtAddr(s_curDialAddr, a, sizeof(a));
    warn("dial %s failed (status 0x%02x)", a, (unsigned)reason);
    ble_event_t ev = {};
    ev.event    = BLE_EV_DISCONNECT;
    ev.conn     = BLE_HS_CONN_HANDLE_NONE;
    ev.reason   = reason;
    ev.addrType = s_curDialType;
    std::memcpy(ev.addr, s_curDialAddr, 6);
    bleFire(&ev);
    s_scanDirty = true;      /* the dial cancelled the scan; bring it back */
}

void bleDisconnect(uint16_t conn) {
    if (!s_hostUp) return;
    ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
}

uint16_t bleConnMtu(uint16_t conn) {
    ble_conn_t* c = bleConnFind(conn);
    return c ? c->mtu : 0;
}

bool bleConnParams(uint16_t conn, uint16_t minItvl, uint16_t maxItvl,
                   uint16_t latency, uint16_t supervisionMs) {
    if (!s_hostUp) return false;
    struct ble_gap_upd_params p = {};
    p.itvl_min            = minItvl;
    p.itvl_max            = maxItvl;
    p.latency             = latency;
    p.supervision_timeout = (uint16_t)(supervisionMs / 10);   /* units of 10 ms */
    p.min_ce_len          = 0;
    p.max_ce_len          = 0;
    return ble_gap_update_params(conn, &p) == 0;
}

bool bleSlotReserve(const char* owner, int n) {
    if (!owner || n < 0) return false;
    bool ok = false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    ble_reserver_t* free_ = nullptr;
    ble_reserver_t* mine  = nullptr;
    for (auto& r : s_reserve) {
        if (r.owner && strcmp(r.owner, owner) == 0) { mine = &r; break; }
        if (!r.owner && !free_) free_ = &r;
    }
    ble_reserver_t* slot = mine ? mine : free_;
    if (slot && bleSlotsCommitted(owner) + n <= BLE_MAX_CONNS) {
        slot->owner = owner;
        slot->n     = n;
        ok = true;
    }
    xSemaphoreGive(s_lock);
    if (!ok) warn("%s: cannot reserve %d of %d connections", owner, n, BLE_MAX_CONNS);
    return ok;
}

void bleSlotRelease(const char* owner) {
    if (!owner) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (auto& r : s_reserve)
        if (r.owner && strcmp(r.owner, owner) == 0) { r.owner = nullptr; r.n = 0; }
    xSemaphoreGive(s_lock);
}

/* ─────────────── config ─────────────── */

static void applyConfig(void) {
    bool en  = storageGetInt("s.ble.enable", 0) != 0;
    int  txp = storageGetInt("s.ble.txpower", 9);
    if (txp < -24) txp = -24;
    if (txp >  20) txp =  20;

    bool changed = (txp != s_txPowerDbm);
    s_enabled    = en;
    s_txPowerDbm = txp;

    if (!en) {
        /* The master switch. With it off the host never starts, whatever a
         * consumer asked for — the want survives, so flipping it back on
         * restores the radio without the consumer doing anything. */
        hostStop();
        publishState();
        return;
    }
    if (s_hostUp && changed) {
        int lvl = (s_txPowerDbm + 24) / 3;
        if (lvl < 0) lvl = 0;
        if (lvl > ESP_PWR_LVL_P20) lvl = ESP_PWR_LVL_P20;
        esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, (esp_power_level_t)lvl);
    }
    if (!bleWanted()) { hostStop(); publishState(); return; }
    if (!s_hostStarted) {
        s_startRetryMs = hostStart() ? 0 : millis() + 10000;
        publishState();
    }

    ble_event_t ev = {};
    ev.event = BLE_EV_CFG_CHANGED;
    bleFire(&ev);
}

static void onCfgChange(const char*, const char*) {
    s_configDirty = true;
    bleWake();
}

/* The pairing window as a command sentinel: an ephemeral key carrying the
 * number of seconds, answered on ble.pair.error / ble.pair.done. */
static void onPairCmd(const char* key, const char* val) {
    if (strcmp(key, "ble.pair") != 0 || !val || !*val) return;
    int secs = atoi(val);
    storageUnset(key);
    if (secs < 0 || secs > 3600) {
        storageSet("ble.pair.error", "Pairing window must be 0-3600 seconds.");
        return;
    }
    if (!s_enabled) {
        storageSet("ble.pair.error", "Bluetooth is disabled — enable it first.");
        return;
    }
    blePairingWindow((uint32_t)secs);
    static int ack = 0;
    storageSet("ble.pair.done", ++ack);
}

/* Dropping bonds, same shape: an ephemeral key carrying "all" or one address. */
static void onForgetCmd(const char* key, const char* val) {
    if (strcmp(key, "ble.forget") != 0 || !val || !*val) return;
    char want[24];
    safeStrncpy(want, val, sizeof(want));
    storageUnset(key);
    if (!s_hostUp) {
        storageSet("ble.forget.error", "Bluetooth is not running.");
        return;
    }
    /* This handler runs on the ble task (the subscription was made here), so
     * the Now variants are legal and the error answer stays synchronous. */
    if (strcmp(want, "all") == 0) {
        bleForgetAllNow();
    } else {
        unsigned b[6];
        if (sscanf(want, "%2x:%2x:%2x:%2x:%2x:%2x",
                   &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6) {
            storageSet("ble.forget.error",
                       "Expected a Bluetooth address (AA:BB:CC:DD:EE:FF) or \"all\".");
            return;
        }
        uint8_t addr[6];
        for (int i = 0; i < 6; i++) addr[i] = (uint8_t)b[5 - i];
        if (!bleForgetNow(addr)) {
            storageSet("ble.forget.error", "That address is not bonded.");
            return;
        }
    }
    static int ack = 0;
    storageSet("ble.forget.done", ++ack);
}

/* ─────────────── the ble task ───────────────
 *
 * Priority 2 on the primary core, the spangap-net precedent for a radio owner:
 * a consumer sits at 1, so the owner is never starved behind the traffic it
 * carries. The canonical ITS loop, with the poll timeout set by whichever of
 * the advertising rotation and the pairing window wants attention first. */

static void bleTaskMain(void*) {
    info("[%s] task up", TAG);

    itsClientInit(2);
    bleGapQueueInit();
    storageSubscribeChanges("s.ble.enable",  onCfgChange);
    storageSubscribeChanges("s.ble.txpower", onCfgChange);
    storageSubscribeChanges("ble.pair",      onPairCmd);
    storageSubscribeChanges("ble.forget",    onForgetCmd);

    for (;;) {
        while (itsPoll(0)) {}

        if (s_startRetryMs && (int32_t)(millis() - s_startRetryMs) >= 0) {
            s_startRetryMs = 0;
            s_configDirty  = true;     /* try hostStart again */
        }

        if (s_configDirty) {
            s_configDirty = false;
            /* A service queued after the host came up is the one case that
             * cannot be reconciled in place: NimBLE's registry is only writable
             * before the host starts. */
            if (s_hostUp && s_svcCount != s_svcRegistered) hostRestartForGatt();
            applyConfig();
        }

        if (s_synced && !s_hostUp) hostSynced();

        /* Everything the host task saw, run here instead. */
        bleGapDrain();

        bleStoreTick();

        if (s_hostUp) {
            bool dial = false;
            ble_addr_t peer = {};
            uint32_t dialTimeout = 0;
            xSemaphoreTake(s_lock, portMAX_DELAY);
            if (s_dialPending) {
                s_dialPending = false;
                dial = true;
                peer.type   = s_dialType;
                dialTimeout = s_dialTimeout;
                std::memcpy(peer.val, s_dialAddr, 6);
            }
            xSemaphoreGive(s_lock);
            if (dial) {
                /* What a failure event will name — the parked s_dialAddr may
                 * already hold the consumer's next dial by then. */
                std::memcpy(s_curDialAddr, peer.val, 6);
                s_curDialType = peer.type;
                /* Scanning and connecting cannot run at once on this
                 * controller, so the scan yields for the dial and the reconcile
                 * below restarts it. */
                bleScanStopNow();
                int rc = ble_gap_connect(s_ownAddrType, &peer, (int32_t)dialTimeout,
                                         nullptr, bleGapEvent, nullptr);
                if (rc != 0) { warn("connect: rc=%d", rc); bleDialFailed(0); }
                else {
                    s_dialInFlight  = true;
                    s_dialForcedMs  = millis() + dialTimeout + 3000;
                }
            }
            /* The dial watchdog. The outcome of a dial arrives as a host
             * event, and host events can be lost — a full queue, a
             * connection that dies between the event and its conn lookup. A
             * dial whose outcome never arrives would otherwise park the
             * scan, every future dial and the address rotation FOREVER, so
             * past the dial's own timeout plus margin it is declared failed
             * from here. */
            if (s_dialInFlight && (int32_t)(millis() - s_dialForcedMs) >= 0) {
                warn("dial outcome lost — forcing failure");
                s_dialInFlight = false;
                bleDialFailed(0);
                s_scanDirty = true;
            }
            /* Host-event queue overflow is silent in host context; report it
             * from here, where logging is legal. */
            {
                static uint32_t lastDrops = 0;
                if (s_hevDrops != lastDrops) {
                    warn("host event queue dropped %u events",
                         (unsigned)(s_hevDrops - lastDrops));
                    lastDrops = s_hevDrops;
                }
            }
            bool fAll = false, fOne = false;
            uint8_t fAddr[6];
            xSemaphoreTake(s_lock, portMAX_DELAY);
            if (s_forgetPending) {
                s_forgetPending = false;
                fAll = s_forgetAll;
                fOne = !fAll;
                std::memcpy(fAddr, s_forgetAddr, 6);
            }
            xSemaphoreGive(s_lock);
            if (fAll)      bleForgetAllNow();
            else if (fOne) bleForgetNow(fAddr);

            /* NimBLE's own privacy timer regenerates the RPA underneath us
             * (preempting and restoring adv+scan itself). Adopt the new value
             * and refire BLE_EV_UP so consumers' copies follow. */
            uint8_t curAddr[6];
            if (s_pvcyOn
                    && ble_hs_id_copy_addr(BLE_ADDR_RANDOM, curAddr, nullptr) == 0
                    && std::memcmp(curAddr, s_ownAddr, 6) != 0) {
                std::memcpy(s_ownAddr, curAddr, 6);
                s_lastRotateMs = millis();
                char a[20]; bleFmtAddr(s_ownAddr, a, sizeof(a));
                info("address rotated (privacy timer): now %s", a);
                ble_event_t ev = {};
                ev.event = BLE_EV_UP;
                std::memcpy(ev.addr, s_ownAddr, 6);
                ev.addrType = s_ownAddrType;
                bleFire(&ev);
            }

            /* The controller refuses LE Set Random Address while advertising,
             * scanning or a pending connect uses it, so those stop first and
             * the reconcile below brings them back under the new address. A
             * dial in flight defers the rotation to the pass its completion
             * wakes; live connections keep their address and keep running. */
            if ((s_rotateDue || millis() - s_lastRotateMs >= BLE_ADDR_ROTATE_MS)
                    && !s_dialInFlight) {
                s_rotateDue    = false;
                s_lastRotateMs = millis();
                bleAdvStop();
                bleScanStopNow();
                if (addrGenerate()) {
                    char a[20]; bleFmtAddr(s_ownAddr, a, sizeof(a));
                    info("address rotated: now %s", a);
                } else warn("address rotation rejected — keeping the old one");
                s_advDirty  = true;
                s_scanDirty = true;
                ble_event_t ev = {};
                ev.event = BLE_EV_UP;      /* consumers re-read their copy */
                std::memcpy(ev.addr, s_ownAddr, 6);
                ev.addrType = s_ownAddrType;
                bleFire(&ev);
            }

            bleAdvReconcile();
            bleScanReconcile();
            blePairingTick();
        }

        /* At most 1 Hz: the loop wakes per host event, and a storage batch per
         * scan report would churn the subscribers for nothing. */
        static uint32_t lastStatsMs = 0;
        if (uiTelemetryWanted() && millis() - lastStatsMs >= 1000) {
            lastStatsMs = millis();
            publishStats();
        }

        /* While the host is up the rotation sets the cadence; while it is down
         * there is nothing to tick — except a start retry — so park until
         * something wakes us. */
        TickType_t wait = portMAX_DELAY;
        if (!s_hostUp) {
            uint32_t d = 0;
            if (s_startRetryMs) {
                int32_t left = (int32_t)(s_startRetryMs - millis());
                d = left > 1 ? (uint32_t)left : 1;
            }
            uint32_t sd = bleStoreDueMs();   /* a persist pending past hostStop */
            if (sd && (!d || sd < d)) d = sd;
            if (d) wait = pdMS_TO_TICKS(d);
        }
        if (s_hostUp) {
            uint32_t due = bleAdvNextDueMs();
            if (blePairingLeft() && due > 500) due = 500;
            if (due > 1000) due = 1000;      /* the stats publish cap */
            wait = pdMS_TO_TICKS(due ? due : 1);
        }
        itsPoll(wait);
    }
}

/* ─────────────── boot ─────────────── */

void BleService::onInit() {
    s_lock = xSemaphoreCreateMutex();

    /* Earmark the controller's DRAM now, while the heap is still one large
     * block — the boot storm behind this onInit fragments it, and the
     * controller's exchange-memory arena needs its space contiguous. Only
     * when Bluetooth is switched on: a build running with it off keeps the
     * RAM, and a later enable falls back to the best-effort path. */
    if (storageGetInt("s.ble.enable", 0) != 0) {
        ctrlReserveGrab();
        if (!s_ctrlReserve) warn("could not earmark %u B for the controller",
                                 (unsigned)BLE_CTRL_RESERVE);
    }

    /* Ephemeral readouts exist from boot so a settings pane never shows a blank
     * row before the first publish. */
    storageBegin();
    storageSet("ble.state_text", "disabled");
    storageSet("ble.peers", "none");
    storageSet("ble.bonds", "0 bonded");
    storageSet("ble.pairing_text", "closed");
    storageEnd();

    /* NimBLE narrates every GAP/GATT procedure at INFO — three lines per
     * advertising rotation, one per keepalive write, one per outbound
     * notification, plus parameter continuation lines. They all carry the
     * ESP tag "NimBLE", and a logRule prefix with a colon matches tag-first,
     * so this one rule re-levels the whole family; BLE_INIT keeps its own. */
    logRule("NimBLE: ", 'D');

    bleCliRegister();
    /* PSRAM stack like the rest of the platform: nothing on this task writes
     * flash any more — the bond store persists through /state (ble_store.cpp)
     * and PHY calibration persistence is off, so the NVS writers that once
     * forced a DRAM stack here are gone. */
    s_task = spawnTask(bleTaskMain, TAG, 6144, nullptr, 2, CORE_PRIMARY, STACK_PSRAM);
}
