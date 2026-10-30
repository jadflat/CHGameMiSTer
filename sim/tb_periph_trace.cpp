// The record tb_periph_log.cpp writes and tb_periph_replay.cpp reads
// (included by both, not built on its own).
#pragma once
#include <cstdint>

enum { TR_READ = 0, TR_WRITE = 1, TR_BUTTONS = 2, TR_RESET = 3, TR_END = 4 };

struct TraceRec {
    uint64_t t;         // board cycle (m->cycles) of the access
    uint32_t addr;
    uint32_t val;       // read: what the C returned; write: the value; buttons: CHG_BTN_*
    uint32_t dt;        // t minus the instruction boundary before it (the top of the chg_run loop)
    uint32_t dn;        // the next boundary minus t
    uint8_t  kind;
    uint8_t  size;
    uint8_t  pad[6];
};
