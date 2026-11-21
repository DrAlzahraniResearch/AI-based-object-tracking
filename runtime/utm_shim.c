/* Lean runtime start-up + array wrappers around the Lean-exported checker. */
#include <lean/lean.h>
#include "utm_checker.h"

/* Exported from Lean (proofs/UtmChecker.lean, @[export ...]); Int64 crosses as its uint64_t bit pattern. */
extern uint8_t  utm_separated(uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t,
                              uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t,
                              uint64_t);
extern uint8_t  utm_separated_hv(uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t,
                                 uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t,
                                 uint64_t, uint64_t);
extern uint8_t  utm_inside(uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t,
                           uint64_t, uint64_t, uint64_t, uint64_t, uint64_t);
extern uint8_t  utm_separated_dev(uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t,
                                  uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t,
                                  uint64_t, uint64_t, uint64_t, uint64_t, uint64_t);
extern uint32_t utm_abi_version(lean_object *);
extern lean_object *initialize_UtmChecker(uint8_t builtin);
extern void lean_initialize_runtime_module(void);   /* in libleanrt; not declared in lean.h */

static int g_ready = 0;

int utm_init(void) {
    if (g_ready) return 0;
    lean_initialize_runtime_module();
    lean_object *res = initialize_UtmChecker(1);
    if (!lean_io_result_is_ok(res)) {
        lean_io_result_show_error(res);
        lean_dec_ref(res);
        return 1;
    }
    lean_dec_ref(res);
    lean_io_mark_end_initialization();
    if (utm_abi_version(lean_box(0)) != UTM_ABI_VERSION) return 2;
    g_ready = 1;
    return 0;
}

uint32_t utm_version(void) { return g_ready ? utm_abi_version(lean_box(0)) : 0; }

#define U(v) ((uint64_t)(v))

int utm_check_separated(const int64_t a[8], const int64_t b[8], int64_t buf) {
    if (!g_ready) return -1;
    return utm_separated(U(a[0]), U(a[1]), U(a[2]), U(a[3]), U(a[4]), U(a[5]), U(a[6]), U(a[7]),
                         U(b[0]), U(b[1]), U(b[2]), U(b[3]), U(b[4]), U(b[5]), U(b[6]), U(b[7]), U(buf));
}

int utm_check_separated_hv(const int64_t a[8], const int64_t b[8], int64_t h, int64_t v) {
    if (!g_ready) return -1;
    return utm_separated_hv(U(a[0]), U(a[1]), U(a[2]), U(a[3]), U(a[4]), U(a[5]), U(a[6]), U(a[7]),
                            U(b[0]), U(b[1]), U(b[2]), U(b[3]), U(b[4]), U(b[5]), U(b[6]), U(b[7]), U(h), U(v));
}

int utm_check_inside(const int64_t box[8], int64_t m, int64_t x, int64_t y, int64_t z, int64_t t) {
    if (!g_ready) return -1;
    return utm_inside(U(box[0]), U(box[1]), U(box[2]), U(box[3]), U(box[4]), U(box[5]), U(box[6]), U(box[7]),
                      U(m), U(x), U(y), U(z), U(t));
}

int64_t utm_count_conflicts(const int64_t *boxes, const int64_t *group, int64_t n, int64_t buf) {
    if (!g_ready) return -1;
    int64_t bad = 0;
    for (int64_t i = 0; i < n; i++)
        for (int64_t j = i + 1; j < n; j++)
            if (group[i] != group[j] && !utm_check_separated(boxes + 8 * i, boxes + 8 * j, buf)) bad++;
    return bad;
}

int64_t utm_count_conflicts_hv(const int64_t *boxes, const int64_t *group, int64_t n, int64_t h, int64_t v) {
    if (!g_ready) return -1;
    int64_t bad = 0;
    for (int64_t i = 0; i < n; i++)
        for (int64_t j = i + 1; j < n; j++)
            if (group[i] != group[j] && !utm_check_separated_hv(boxes + 8 * i, boxes + 8 * j, h, v)) bad++;
    return bad;
}

int64_t utm_count_against_hv(const int64_t *cand, int64_t m, const int64_t *others, int64_t n, int64_t h, int64_t v) {
    if (!g_ready) return -1;
    int64_t bad = 0;
    for (int64_t i = 0; i < m; i++)
        for (int64_t j = 0; j < n; j++)
            if (!utm_check_separated_hv(cand + 8 * i, others + 8 * j, h, v)) bad++;
    return bad;
}

int utm_check_separated_dev(const int64_t a[8], const int64_t b[8], int64_t h, int64_t v,
                            int64_t eh, int64_t ev, int64_t dt) {
    if (!g_ready) return -1;
    return utm_separated_dev(U(a[0]), U(a[1]), U(a[2]), U(a[3]), U(a[4]), U(a[5]), U(a[6]), U(a[7]),
                             U(b[0]), U(b[1]), U(b[2]), U(b[3]), U(b[4]), U(b[5]), U(b[6]), U(b[7]),
                             U(h), U(v), U(eh), U(ev), U(dt));
}

int64_t utm_count_against_dev(const int64_t *cand, int64_t m, const int64_t *others, int64_t n, int64_t h, int64_t v,
                              int64_t eh, int64_t ev, int64_t dt) {
    if (!g_ready) return -1;
    int64_t bad = 0;
    for (int64_t i = 0; i < m; i++)
        for (int64_t j = 0; j < n; j++)
            if (!utm_check_separated_dev(cand + 8 * i, others + 8 * j, h, v, eh, ev, dt)) bad++;
    return bad;
}
