/**
 * ble_store — bond persistence in /state, not NVS.
 *
 * NimBLE's stock config store keeps every record in RAM and, with
 * CONFIG_BT_NIMBLE_NVS_PERSIST off, persists nothing — exactly the split this
 * platform wants: its NVS backend wrote internal flash synchronously on
 * whatever task paired or forgot, outside the platform's wear policy and its
 * backups. So the RAM half and all its semantics stay Espressif's, and
 * persistence is this file: the store's arrays are snapshotted into
 * /state/ble/bonds.bin after a quiet debounce and loaded back before the host
 * first starts. Bonds therefore ride /state backups, migrate with the device,
 * and are wiped by a factory reset like every other durable thing here.
 *
 * The store's arrays are reached through NimBLE's own private header (pulled
 * in via an include path in CMakeLists.txt — it is not exported), which maps
 * them correctly whether the store is built with static arrays or behind the
 * heap-allocated ble_store_config_vars. Only bonds and CCCDs are persisted:
 * privacy records (RPA/IRK), CSF and encrypted-advertising data belong to
 * features this straddle never enables.
 *
 * Change detection is event-driven, not hooked: every mutation of the store
 * already surfaces on the ble task as an event this straddle handles —
 * pairing completion, a CCCD write, a forget — and those mark the store
 * dirty. A crash inside the debounce costs one pairing at worst, and the
 * peer re-pairs.
 */
#include "ble_priv.h"

#include "host/ble_store.h"

#include <cstdio>
#include <cstring>

extern "C" {
#include "ble_store_config_priv.h"
}

/* In the dynamic build the arrays live behind this pointer, allocated by
 * ble_store_config_init(); before the first host start there is nothing to
 * read or write. The macro-mapped array names below fail hard at compile
 * time if NimBLE restructures the store — the desired failure mode. */
static bool storeReady(void) {
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    return ble_store_config_vars != nullptr;
#else
    return true;
#endif
}

#define BLE_STORE_DIR    "/state/ble"
#define BLE_STORE_FILE   BLE_STORE_DIR "/bonds.bin"
#define BLE_STORE_MAGIC  0x424C4253u   /* "SBLB" little-endian on disk */
#define BLE_STORE_QUIET_MS 2000

/* The layout stamp: NimBLE's structs are dumped raw, so a NimBLE upgrade that
 * resizes them must invalidate the file rather than misread it. Bonds are
 * re-creatable by pairing again; honesty beats migration code. */
struct store_hdr_t {
    uint32_t magic;
    uint16_t secSize;
    uint16_t cccdSize;
    uint8_t  nOur, nPeer, nCccd, pad;
};

static uint32_t s_dirtyMs = 0;         /* 0 = clean; else millis() of the change */

/* Dirty is marked by EVENTS (every subscribe, every pairing outcome), not by
 * diffs — so the diff lives here: a hash of the meaningful snapshot, checked
 * before writing. A re-subscribe that changes nothing costs neither a flash
 * write nor a log line. */
static uint32_t s_lastHash = 0;

static uint32_t snapshotHash(void) {
    uint32_t h = 2166136261u;                      /* FNV-1a */
    auto mix = [&h](const void* p, size_t n) {
        const uint8_t* b = (const uint8_t*)p;
        while (n--) { h ^= *b++; h *= 16777619u; }
    };
    int no = ble_store_config_num_our_secs;
    int np = ble_store_config_num_peer_secs;
    int nc = ble_store_config_num_cccds;
    mix(&no, sizeof(no)); mix(&np, sizeof(np)); mix(&nc, sizeof(nc));
    mix(ble_store_config_our_secs,  sizeof(struct ble_store_value_sec)  * no);
    mix(ble_store_config_peer_secs, sizeof(struct ble_store_value_sec)  * np);
    mix(ble_store_config_cccds,     sizeof(struct ble_store_value_cccd) * nc);
    return h;
}

void bleStoreDirty(void) {
    uint32_t now = millis();
    s_dirtyMs = now ? now : 1;
}

uint32_t bleStoreDueMs(void) {
    if (!s_dirtyMs) return 0;
    uint32_t age = millis() - s_dirtyMs;
    return age >= BLE_STORE_QUIET_MS ? 1 : BLE_STORE_QUIET_MS - age;
}

/* Called at every host start, right AFTER ble_store_config_init() — that init
 * zeroes the RAM counts each time, so the file is the truth to restore from.
 * bleStoreFlush() ran just before it, so the file holds any pending change. */
/* File I/O is the fs_* handle API, never plain stdio: stdio runs LittleFS on
 * the CALLING task, whose cache-off window asserts under this task's PSRAM
 * stack — the fs worker's DRAM stack is the whole point of the proxy. */
void bleStoreLoad(void) {
    if (!storeReady()) return;
    int f = fs_open(BLE_STORE_FILE, "rb");
    if (f < 0) {                       /* nothing ever bonded: the normal case */
        s_lastHash = snapshotHash();
        return;
    }
    store_hdr_t h = {};
    bool ok = fs_read(&h, sizeof(h), 1, f) == 1 &&
              h.magic    == BLE_STORE_MAGIC &&
              h.secSize  == sizeof(struct ble_store_value_sec) &&
              h.cccdSize == sizeof(struct ble_store_value_cccd) &&
              h.nOur  <= CONFIG_BT_NIMBLE_MAX_BONDS &&
              h.nPeer <= CONFIG_BT_NIMBLE_MAX_BONDS &&
              h.nCccd <= CONFIG_BT_NIMBLE_MAX_CCCDS;
    if (ok) {
        ok = fs_read(ble_store_config_our_secs,  sizeof(struct ble_store_value_sec),  h.nOur,  f) == h.nOur
          && fs_read(ble_store_config_peer_secs, sizeof(struct ble_store_value_sec),  h.nPeer, f) == h.nPeer
          && fs_read(ble_store_config_cccds,     sizeof(struct ble_store_value_cccd), h.nCccd, f) == h.nCccd;
    }
    fs_close(f);
    if (!ok) {
        warn("bond store: %s stale or torn — starting empty", BLE_STORE_FILE);
        fs_remove(BLE_STORE_FILE);
        s_lastHash = snapshotHash();
        return;
    }
    ble_store_config_num_our_secs  = h.nOur;
    ble_store_config_num_peer_secs = h.nPeer;
    ble_store_config_num_cccds     = h.nCccd;
    s_lastHash = snapshotHash();
    if (h.nOur)
        info("bond store: %u bond%s restored", h.nOur, h.nOur == 1 ? "" : "s");
}

static void persistNow(void) {
    s_dirtyMs = 0;
    if (!storeReady()) return;
    uint32_t hnow = snapshotHash();
    if (hnow == s_lastHash) return;    /* the event changed nothing */

    fs_mkdirp(BLE_STORE_DIR);
    store_hdr_t h = {};
    h.magic    = BLE_STORE_MAGIC;
    h.secSize  = (uint16_t)sizeof(struct ble_store_value_sec);
    h.cccdSize = (uint16_t)sizeof(struct ble_store_value_cccd);
    h.nOur     = (uint8_t)ble_store_config_num_our_secs;
    h.nPeer    = (uint8_t)ble_store_config_num_peer_secs;
    h.nCccd    = (uint8_t)ble_store_config_num_cccds;

    /* Write-then-rename, so a crash mid-write leaves the previous file. */
    int f = fs_open(BLE_STORE_FILE ".new", "wb");
    if (f < 0) { warn("bond store: cannot write %s", BLE_STORE_FILE ".new"); return; }
    bool ok = fs_write(&h, sizeof(h), 1, f) == 1
           && fs_write(ble_store_config_our_secs,  sizeof(struct ble_store_value_sec),  h.nOur,  f) == h.nOur
           && fs_write(ble_store_config_peer_secs, sizeof(struct ble_store_value_sec),  h.nPeer, f) == h.nPeer
           && fs_write(ble_store_config_cccds,     sizeof(struct ble_store_value_cccd), h.nCccd, f) == h.nCccd;
    fs_close(f);
    if (!ok || fs_rename(BLE_STORE_FILE ".new", BLE_STORE_FILE) != 0) {
        /* s_lastHash stays stale, so the next change persists everything. */
        warn("bond store: persist failed");
        fs_remove(BLE_STORE_FILE ".new");
        return;
    }
    s_lastHash = hnow;
    info("bond store: %u bond%s persisted", h.nOur, h.nOur == 1 ? "" : "s");
}

void bleStoreTick(void) {
    if (s_dirtyMs && millis() - s_dirtyMs >= BLE_STORE_QUIET_MS) persistNow();
}

void bleStoreFlush(void) {
    if (s_dirtyMs) persistNow();
}
