/**
 * ble_cli — the `ble` verbs. Each subcommand is its own whole-string
 * registration, because dispatch is longest-prefix: `ble peers` must be
 * registered beside `ble`, not parsed out of its arguments.
 *
 * Bare `ble` prints status — the platform has no separate `status` verb.
 */
#include "ble_priv.h"

#include "esp_heap_caps.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

/* "AA:BB:CC:DD:EE:FF" (display order, most significant byte first) into
 * NimBLE's least-significant-first array. Returns false on anything else. */
static bool parseAddr(const char* s, uint8_t out[6]) {
    unsigned b[6];
    if (!s || sscanf(s, "%2x:%2x:%2x:%2x:%2x:%2x",
                     &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6) return false;
    for (int i = 0; i < 6; i++) out[i] = (uint8_t)b[5 - i];
    return true;
}

static void cliBle(const char* args) {
    if (args && strcmp(args, "help") == 0) {
        cliPrintf("%-*s Bluetooth status; up/down\n", CLI_HELP_COL, "ble [up|down]");
        return;
    }
    if (args && cliWantsHelp(args)) {
        cliPrintf("%-*s Bluetooth status\n",              CLI_HELP_COL, "ble");
        cliPrintf("%-*s enable/disable Bluetooth\n",      CLI_HELP_COL, "ble up|down");
        cliPrintf("%-*s attached connections\n",          CLI_HELP_COL, "ble peers");
        cliPrintf("%-*s stored bonds\n",                  CLI_HELP_COL, "ble bonds");
        cliPrintf("%-*s open the pairing window\n",       CLI_HELP_COL, "ble pair [seconds]");
        cliPrintf("%-*s drop a bond\n",                   CLI_HELP_COL, "ble forget <addr|all>");
        cliPrintf("%-*s one-shot scan for advertisers\n", CLI_HELP_COL, "ble scan [seconds]");
        return;
    }
    /* The verbs flip the master switch only; the wants stay each consumer's
     * own, and the reconcile pass follows the setting change. */
    if (args && strcmp(args, "up") == 0)   { storageSet("s.ble.enable", 1); return; }
    if (args && strcmp(args, "down") == 0) { storageSet("s.ble.enable", 0); return; }

    char a[20];
    cliPrintf("state:    %s\n", s_hostUp ? "up"
                              : !s_enabled ? "disabled"
                              : bleWanted() ? "starting" : "idle (nothing has asked)");
    if (s_hostUp) {
        bleFmtAddr(s_ownAddr, a, sizeof(a));
        cliPrintf("address:  %s (public)\n", a);
    }
    cliPrintf("tx power: %d dBm\n", s_txPowerDbm);
    cliPrintf("conns:    %d of %d attached, %u since boot\n",
              bleConnCount(), BLE_MAX_CONNS, (unsigned)s_connTotal);
    int committed = bleSlotsCommitted(nullptr);
    if (committed) {
        cliPrintf("reserved: ");
        for (auto& r : s_reserve) if (r.owner) cliPrintf("%s=%d ", r.owner, r.n);
        cliPrintf("(%d of %d)\n", committed, BLE_MAX_CONNS);
    }
    int adv = 0;
    for (auto& s : s_adv) if (s.used) adv++;
    cliPrintf("adv:      %d claim%s, %u rotation%s\n", adv, adv == 1 ? "" : "s",
              (unsigned)s_advCycles, s_advCycles == 1 ? "" : "s");
    cliPrintf("scan:     %s, %u report%s\n", s_scanWanted ? "on" : "off",
              (unsigned)s_scanSeen, s_scanSeen == 1 ? "" : "s");
    cliPrintf("bonds:    %d stored, pairing %s\n", bleBondCount(),
              blePairingLeft() ? "OPEN" : "closed");
    if (s_pairFails) cliPrintf("refused:  %u pairing attempt%s\n",
                               (unsigned)s_pairFails, s_pairFails == 1 ? "" : "s");
    /* Internal DRAM, not flash, is what BLE is scarce in: the controller's
     * activities are internal-only whatever the host's allocation mode says. */
    cliPrintf("heap:     %u B internal, %u B PSRAM free\n",
              (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
              (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

static void cliBlePeers(const char* args) {
    if (args && cliWantsHelp(args)) {
        cliPrintf("%-*s attached connections\n", CLI_HELP_COL, "ble peers");
        return;
    }
    if (!s_hostUp) { cliPrintf("Bluetooth is down\n"); return; }
    int n = 0;
    for (auto& c : s_conns) {
        if (!c.used) continue;
        char a[20]; bleFmtAddr(c.addr, a, sizeof(a));
        cliPrintf("%-18s %-10s mtu %-4u %s\n", a,
                  c.central ? "central" : "peripheral",
                  (unsigned)c.mtu, c.wasBonded ? "bonded" : "open");
        n++;
    }
    if (!n) cliPrintf("no connections\n");
}

static void cliBleBonds(const char* args) {
    if (args && cliWantsHelp(args)) {
        cliPrintf("%-*s stored bonds\n", CLI_HELP_COL, "ble bonds");
        return;
    }
    if (!s_hostUp) { cliPrintf("Bluetooth is down\n"); return; }
    cliPrintf("%d of %d bond slots used\n", bleBondCount(), CONFIG_BT_NIMBLE_MAX_BONDS);
}

static void cliBlePair(const char* args) {
    if (args && cliWantsHelp(args)) {
        cliPrintf("%-*s open the pairing window (default 60 s, 0 closes it)\n",
                  CLI_HELP_COL, "ble pair [seconds]");
        return;
    }
    int secs = (args && *args) ? atoi(args) : 60;
    if (secs < 0 || secs > 3600) { cliPrintf("seconds must be 0-3600\n"); return; }
    if (!s_enabled) { cliPrintf("Bluetooth is disabled — `ble up` first\n"); return; }
    blePairingWindow((uint32_t)secs);
    cliPrintf(secs ? "pairing open for %ds\n" : "pairing closed\n", secs);
}

static void cliBleForget(const char* args) {
    if (!args || !*args || cliWantsHelp(args)) {
        cliPrintf("%-*s drop a bond, or every bond\n",
                  CLI_HELP_COL, "ble forget <addr|all>");
        return;
    }
    if (!s_hostUp) { cliPrintf("Bluetooth is down\n"); return; }
    if (strcmp(args, "all") == 0) { bleForgetAll(); cliPrintf("all bonds dropped\n"); return; }
    uint8_t addr[6];
    if (!parseAddr(args, addr)) { cliPrintf("expected AA:BB:CC:DD:EE:FF or `all`\n"); return; }
    cliPrintf(bleForget(addr) ? "dropped\n" : "no such bond\n");
}

/* The debug scan reads the table the ble task keeps of everything the scanner
 * has heard, whoever asked for the scan. So `ble scan` starts the observer if
 * nobody else has, and is run again to read the results — no CLI command ever
 * blocks waiting for the radio, and running it alongside a consumer's own scan
 * shows exactly what that consumer is seeing. */
static void cliBleScan(const char* args) {
    if (args && cliWantsHelp(args)) {
        cliPrintf("%-*s show what the scanner has heard (starts it if idle)\n",
                  CLI_HELP_COL, "ble scan");
        cliPrintf("%-*s stop the debug scan and clear the table\n",
                  CLI_HELP_COL, "ble scan off");
        return;
    }
    if (!s_hostUp) { cliPrintf("Bluetooth is down — `ble up` first\n"); return; }

    /* Whether a consumer wanted the scan before we did. Re-asserting a
     * consumer's request is the reconcile pass's job, not ours. */
    static bool s_cliWanted = false;

    if (args && strcmp(args, "off") == 0) {
        if (s_cliWanted) { s_cliWanted = false; bleScanStop(); }
        bleSeenClear();
        cliPrintf("debug scan off\n");
        return;
    }

    if (!s_scanWanted) {
        ble_scan_req_t r = {};
        r.activeScan = 1;          /* ask for scan responses, so names arrive */
        r.intervalMs = 100;
        r.windowMs   = 90;
        bleScanStart(&r);
        s_cliWanted = true;
        cliPrintf("scanning — run `ble scan` again for results, "
                  "`ble scan off` to stop\n");
        return;
    }

    ble_seen_t seen[BLE_MAX_SEEN];
    int n = bleSeenList(seen, BLE_MAX_SEEN);
    if (!n) { cliPrintf("nothing heard yet\n"); return; }
    for (int i = 0; i < n; i++) {
        char a[20]; bleFmtAddr(seen[i].addr, a, sizeof(a));
        char u[40] = "-";
        bool any = false;
        for (int k = 0; k < 16; k++) if (seen[i].uuid128[k]) { any = true; break; }
        if (any) {
            /* 128-bit UUIDs travel least-significant byte first; print the
             * canonical form so it can be compared with a spec by eye. */
            int o = 0;
            for (int k = 15; k >= 0; k--) {
                o += snprintf(u + o, sizeof(u) - o, "%02x", seen[i].uuid128[k]);
                if (k == 12 || k == 10 || k == 8 || k == 6)
                    o += snprintf(u + o, sizeof(u) - o, "-");
            }
        }
        cliPrintf("%-18s %4d dBm  %-38s %s\n", a, (int)seen[i].rssi, u, seen[i].name);
    }
}

void bleCliRegister(void) {
    cliRegisterCmd("ble",        cliBle);
    cliRegisterCmd("ble peers",  cliBlePeers);
    cliRegisterCmd("ble bonds",  cliBleBonds);
    cliRegisterCmd("ble pair",   cliBlePair);
    cliRegisterCmd("ble forget", cliBleForget);
    cliRegisterCmd("ble scan",   cliBleScan);
}
