/* Verified separation checker: C API.
 *
 * The decisions are made by Lean code (proofs/UtmChecker.lean) whose soundness
 * is proved in proofs/UtmProofs/Refinement.lean:
 *   utm_check_separated(a, b, buf) == 1  and both aircraft inside their own
 *   volume at the same time  ==>  their Euclidean distance >= buf          (Theorem 1)
 *   utm_check_separated_hv(a, b, h, v) == 1  and both inside at the same time
 *   ==>  horizontal distance >= h  or  vertical distance >= v (no LoS)      (Theorem 2)
 *
 * Units: millimetres (x, y, z, local ENU) and milliseconds (t); integers only.
 * A box is int64_t[8] = {x0, x1, y0, y1, z0, z1, t0, t1}, closed intervals.
 * Call utm_init() once (any thread) before anything else; not re-entrant during init.
 */
#ifndef UTM_CHECKER_H
#define UTM_CHECKER_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define UTM_ABI_VERSION 2
/* leanc compiles with -fvisibility=hidden; the public API must opt back in. */
#define UTM_API __attribute__((visibility("default")))

UTM_API int      utm_init(void);                       /* 0 = ok; idempotent */
UTM_API uint32_t utm_version(void);                    /* == UTM_ABI_VERSION once initialised */
UTM_API int      utm_check_separated(const int64_t a[8], const int64_t b[8], int64_t buf);          /* 1/0, -1 if not initialised */
UTM_API int      utm_check_inside(const int64_t box[8], int64_t margin,
                          int64_t x, int64_t y, int64_t z, int64_t t);                     /* 1/0, -1 if not initialised */
UTM_API int      utm_check_separated_hv(const int64_t a[8], const int64_t b[8], int64_t h, int64_t v); /* 1/0, -1 */
/* Count pairs (i<j, different groups) whose boxes are NOT separated. boxes: n x 8, group: n ids. */
UTM_API int64_t  utm_count_conflicts(const int64_t *boxes, const int64_t *group, int64_t n, int64_t buf);
UTM_API int64_t  utm_count_conflicts_hv(const int64_t *boxes, const int64_t *group, int64_t n, int64_t h, int64_t v);
/* Count pairs (candidate i, other j) NOT cylinder-separated. cand: m x 8, others: n x 8. */
UTM_API int64_t  utm_count_against_hv(const int64_t *cand, int64_t m, const int64_t *others, int64_t n,
                                      int64_t h, int64_t v);

#ifdef __cplusplus
}
#endif
#endif
