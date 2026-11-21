#include <stdio.h>
#include "utm_checker.h"
int main(void) {
    if (utm_init() != 0) { puts("init failed"); return 1; }
    int64_t a[8] = {0, 10, 0, 10, 0, 10, 0, 10};
    int64_t b[8] = {15, 20, 0, 10, 0, 10, 0, 10}, c[8] = {14, 20, 0, 10, 0, 10, 0, 10};
    int64_t d[8] = {0, 10, 0, 10, 0, 10, 11, 20};
    int ok = utm_version() == UTM_ABI_VERSION && utm_check_separated(a, b, 5) == 1 &&
             utm_check_separated(a, c, 5) == 0 && utm_check_separated(a, d, 5) == 1 &&
             utm_check_inside(a, 2, 5, 5, 5, 5) == 1 && utm_check_inside(a, 2, 9, 5, 5, 5) == 0;
    /* negative coordinates and large values cross the ABI intact */
    int64_t e[8] = {-5000000000LL, -4000000000LL, -1, 1, -1, 1, -1, 1}, f[8] = {-3999999000LL, 0, -1, 1, -1, 1, -1, 1};
    ok = ok && utm_check_separated(e, f, 1000) == 1 && utm_check_separated(e, f, 1001) == 0;
    /* cylinder form: z gap 5 counts against v, x gap 5 against h */
    int64_t g[8] = {0, 10, 0, 10, 15, 20, 0, 10}, h2[8] = {15, 20, 0, 10, 0, 10, 0, 10};
    ok = ok && utm_check_separated_hv(a, g, 100, 5) == 1 && utm_check_separated_hv(a, g, 100, 6) == 0 &&
         utm_check_separated_hv(a, h2, 5, 100) == 1 && utm_check_separated_hv(a, h2, 6, 100) == 0;
    int64_t pair[16] = {0, 10, 0, 10, 0, 10, 0, 10, 15, 20, 0, 10, 0, 10, 0, 10};
    ok = ok && utm_count_against_hv(a, 1, pair, 2, 5, 5) == 1 && utm_count_against_hv(a, 1, pair, 2, 4, 5) == 1;
    /* Theorem 3: x gap 5 = h 1 + 2 x eh 2 exactly; time gap 1 is lost to dt 1 */
    ok = ok && utm_check_separated_dev(a, h2, 1, 100, 2, 0, 0) == 1 && utm_check_separated_dev(a, h2, 1, 100, 3, 0, 0) == 0 &&
         utm_check_separated_dev(a, d, 5, 5, 0, 0, 0) == 1 && utm_check_separated_dev(a, d, 5, 5, 0, 0, 1) == 0 &&
         utm_count_against_dev(a, 1, pair, 2, 1, 100, 2, 0, 0) == 1;
    printf("utm_checker abi %u: %s\n", utm_version(), ok ? "OK" : "FAIL");
    return ok ? 0 : 1;
}
