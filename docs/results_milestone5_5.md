# Milestone 5.5 results: landing reservations (contingent landings)

Same setup as milestone 5 (`docs/results_milestone5.md`): `mini_town`, digital twin with the Tier A plant,
ORCA + FCFS, seeds 1-3, demand x1 / x2 / x5, identical missions in every configuration, 30 m / 10 m separation
everywhere, 5 m terminal / 20 m en-route lateral buffers. The milestone 5 rows are the milestone 5 runs; the
`sequence` mode still reproduces them bit for bit (tracks, events and report, checked on compact x2 seed 1).

* **sequenced** (milestone 5): arrivals are certified only tactically, ~20 s before the approach fix.
* **reserved** (`--hubs reserve`): the planned arrival is certified at admission as a landing reservation;
  a request with no such plan within the 900 s ground-delay limit is rejected. Before the fix the sequencer
  still looks for a better tactical slot; if none certifies, the flight keeps its reservation. Emergencies get
  bounded priority (S43).
* **reserved or tactical** (`--hubs reserve --reserve-fallback`): as reserved, but a request with no
  reservation is admitted without one (milestone 5 handling) instead of being rejected. x2 and x5 only.

## Why arrivals went contingent

Every failed final attempt at x2 (compact seed 1, spread seed 2: 20 contingent arrivals) was logged with what
blocked each candidate plan. None failed after holding. All 20 went contingent at the first attempt at the fix,
with no landing slot and no free holding box on the ring. The blockers were departures climbing out through the
arrival's corridor and flights inbound on the same bearing. Those flights were all certified at admission, but
against plans whose landings were not certified. The terminal area was already full when the arrival asked for
a slot, so holding stacks or a longer lead would not have helped (S40).

## Totals over seeds 1-3

| scale | hubs | runs | flown / requests | rejected | terminal LoS | en-route LoS | arrivals certified | contingent | emergencies uncertified | holding mean / p95 s | ground delay mean s | departures moved |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| x1 | off | 3 | 106 / 106 | 0 | 59 | 0 | - | - | 0 | - | 35 | 0 |
| x1 | sequenced, compact hubs | 3 | 106 / 106 | 0 | 3 | 0 | 103 / 106 | 3 | 2 | 13 / 53 | 129 | 22 |
| x1 | sequenced, spread hubs (40 m) | 3 | 106 / 106 | 0 | 1 | 0 | 104 / 106 | 2 | 1 | 2 / 8 | 41 | 0 |
| x1 | reserved, compact hubs | 3 | 103 / 106 | 3 | 1 | 0 | 103 / 103 | 0 | 1 | 2 / 1 | 320 | 2 |
| x1 | reserved, spread hubs (40 m) | 3 | 106 / 106 | 0 | 0 | 0 | 106 / 106 | 0 | 0 | 0 / 0 | 74 | 0 |
| x2 | off | 3 | 205 / 205 | 0 | 264 | 0 | - | - | 8 | - | 90 | 0 |
| x2 | sequenced, compact hubs | 3 | 178 / 205 | 27 | 29 | 0 | 155 / 178 | 23 | 10 | 18 / 61 | 319 | 126 |
| x2 | sequenced, spread hubs (40 m) | 3 | 205 / 205 | 0 | 10 | 4 | 179 / 205 | 26 | 9 | 6 / 30 | 109 | 5 |
| x2 | sequenced, mismatched plant | 3 | 176 / 205 | 29 | 44 | 0 | 148 / 176 | 28 | 10 | 12 / 54 | 322 | 128 |
| x2 | reserved, compact hubs | 3 | 144 / 205 | 61 | 2 | 0 | 143 / 144 | 1 | 6 | 1 / 2 | 554 | 4 |
| x2 | reserved, spread hubs (40 m) | 3 | 201 / 205 | 4 | 1 | 0 | 197 / 201 | 4 | 2 | 1 / 0 | 289 | 1 |
| x2 | reserved, mismatched plant | 3 | 127 / 205 | 78 | 2 | 0 | 126 / 127 | 1 | 5 | 6 / 24 | 538 | 11 |
| x2 | reserved or tactical, compact | 3 | 149 / 205 | 56 | 3 | 0 | 145 / 149 | 4 | 5 | 10 / 75 | 492 | 36 |
| x2 | reserved or tactical, spread | 3 | 204 / 205 | 1 | 1 | 0 | 200 / 204 | 4 | 2 | 1 / 0 | 294 | 1 |
| x5 | off | 3 | 424 / 515 | 91 | 907 | 2 | - | - | 24 | - | 459 | 0 |
| x5 | sequenced, compact hubs | 3 | 225 / 515 | 290 | 63 | 3 | 185 / 225 | 40 | 23 | 19 / 73 | 464 | 236 |
| x5 | sequenced, spread hubs (40 m) | 3 | 383 / 515 | 132 | 40 | 18 | 319 / 383 | 64 | 23 | 10 / 48 | 462 | 70 |
| x5 | reserved, compact hubs | 3 | 157 / 515 | 358 | 12 | 2 | 152 / 157 | 5 | 15 | 7 / 48 | 530 | 16 |
| x5 | reserved, spread hubs (40 m) | 3 | 279 / 515 | 236 | 5 | 5 | 273 / 279 | 6 | 12 | 0 / 0 | 527 | 1 |
| x5 | reserved or tactical, compact | 3 | 187 / 515 | 328 | 23 | 2 | 175 / 187 | 12 | 20 | 22 / 123 | 434 | 96 |
| x5 | reserved or tactical, spread | 3 | 318 / 515 | 197 | 4 | 3 | 306 / 318 | 12 | 10 | 4 / 28 | 531 | 11 |
-> docs/figures/milestone5_5_hubs.png

Figure: `docs/figures/milestone5_5_hubs.png`. Contingent arrivals = landings with no certified plan (no slot
and no reservation).

* **Contingent arrivals drop by an order of magnitude.** Compact: 23 -> 1 at x2 and 40 -> 5 at x5.
  Spread: 26 -> 4 at x2 and 64 -> 6 at x5. Mismatched plant: 28 -> 1. At x1 there are none (5 before).
  Every remaining contingent arrival traces to an emergency:
  * about three quarters are emergencies that flew uncertified (S19);
  * the rest are mostly flights whose reservation an emergency displaced;
  * two lost certification in a failed conformance replan.
  No flight that was admitted with a reservation and kept it landed uncertified.
* **Terminal loss of separation falls further.** Compact: 29 -> 2 at x2 and 63 -> 12 at x5. Spread: 10 -> 1 at
  x2 and 40 -> 5 at x5. Mismatched plant: 44 -> 2. Total terminal plus en-route LoS over all 21 reserved runs: 30,
  against 215 in the matching 21 sequenced runs.
* **No loss of separation between two certified flights.** The verified audit passes on all 33 reservation runs
  (0 claim violations, 0 counterexamples), and every LoS event is classified "uncertified".
* **Holding nearly disappears.** Mean / p95 holding is 0-7 / 0-48 s, against 2-19 / 8-73 s. Flights fly their
  reservation, and up to 7 per configuration keep it when the tactical search finds nothing better.
* **The price is capacity, and the hub layout sets it.**
  * Spread hubs: x1 and x2 cost almost nothing (x2: 201 of 205 flown, against 205), but mean ground delay goes
    from 109 to 289 s.
  * Compact hubs (one runway) at x2: 144 of 205 flown, against 178. At x5: 157 of 515, against 225.
  * Spread hubs at x5: 279 of 515 flown, against 383.
  * Why: a reservation carries the slack of the whole flight (5 s + 2 s/min, so roughly ±25 s at the pad after
    a 10-minute mission). It blocks the hub for longer than the tactical slot that eventually replaces it.
* **The fallback recovers part of it.** Admitting requests without a reservation instead of rejecting them flies
  149 / 204 (compact / spread) at x2 and 187 / 318 at x5. Contingent arrivals stay low (4 / 4 at x2, 12 / 12 at
  x5), but flights admitted without a reservation can go contingent again.
* **Emergencies.** Priority admission certified 1-12 emergencies per configuration that would otherwise have
  flown uncertified. Uncertified emergencies fall from 10 to 6 (compact x2), 9 to 2 (spread x2), 23 to 15 and
  23 to 12 (x5).

### Reading it

Reservations make the "no certified landing" case rare. The remaining cases come from emergencies, a policy
choice rather than a capacity shortfall. With a spread hub and ×2 demand the cost is ground delay rather than
rejected requests. With a compact hub it costs throughput, because each reservation's time slack occupies a
single runway. The open lever is that slack: tighter arrival timing models, re-certification in flight, or
reservations sized from measured arrival-time error rather than the 2 s/min planning assumption. Re-certifying
in flight every 60 s was tried at x5 (one seed) and added only 2-8 % throughput (S44).

## Theorem 4, extended

`proofs/UtmProofs/Registry.lean` gains `Op.drop f`, which removes any subset of entries. This covers:

* a displaced reservation;
* the rows an emergency's priority admission displaces. The implementation checks against the registry without
  those rows, then removes the ones it conflicts with; the result equals a `drop` followed by an ordinary
  checked `admit`.

`inv_steps` covers sequences that include it. A flight that keeps its reservation leaves the registry unchanged.
The proofs use standard axioms only.

## Per-run table (reservation runs)

| scale | seed | hubs | flown / requests | rejected | departures moved | ground delay mean / p95 s | arrivals sequenced + kept reservation (contingent) | holding mean / p95 / max s | terminal LoS | en-route LoS | emergencies uncertified (priority) | conformance (truth) | audit |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| x1 | 1 | reserved, compact hubs | 31 / 31 | 0 | 0 | 380 / 855 | 31 + 0 (0) | 0 / 0 / 0 | 0 | 0 | 0 (0) | 1.0000 | 0 viol, 0 cex |
| x1 | 2 | reserved, compact hubs | 36 / 37 | 1 | 0 | 311 / 658 | 36 + 0 (0) | 0 / 0 / 0 | 0 | 0 | 0 (1) | 1.0000 | 0 viol, 0 cex |
| x1 | 3 | reserved, compact hubs | 36 / 38 | 2 | 2 | 268 / 802 | 36 + 0 (0) | 5 / 4 / 170 | 1 | 0 | 1 (2) | 1.0000 | 0 viol, 0 cex |
| x1 | 1 | reserved, spread hubs (40 m) | 31 / 31 | 0 | 0 | 81 / 220 | 31 + 0 (0) | 0 / 0 / 0 | 0 | 0 | 0 (0) | 1.0000 | 0 viol, 0 cex |
| x1 | 2 | reserved, spread hubs (40 m) | 37 / 37 | 0 | 0 | 68 / 200 | 37 + 0 (0) | 0 / 0 / 0 | 0 | 0 | 0 (0) | 1.0000 | 0 viol, 0 cex |
| x1 | 3 | reserved, spread hubs (40 m) | 38 / 38 | 0 | 0 | 73 / 262 | 38 + 0 (0) | 0 / 0 / 0 | 0 | 0 | 0 (1) | 1.0000 | 0 viol, 0 cex |
| x2 | 1 | reserved, compact hubs | 48 / 65 | 17 | 0 | 545 / 890 | 45 + 2 (1) | 2 / 0 / 90 | 1 | 0 | 2 (3) | 1.0000 | 0 viol, 0 cex |
| x2 | 2 | reserved, compact hubs | 50 / 76 | 26 | 4 | 496 / 890 | 50 + 0 (0) | 1 / 5 / 35 | 1 | 0 | 4 (0) | 1.0000 | 0 viol, 0 cex |
| x2 | 3 | reserved, compact hubs | 46 / 64 | 18 | 0 | 621 / 900 | 46 + 0 (0) | 0 / 0 / 0 | 0 | 0 | 0 (1) | 1.0000 | 0 viol, 0 cex |
| x2 | 1 | reserved, spread hubs (40 m) | 64 / 65 | 1 | 1 | 302 / 611 | 58 + 3 (3) | 2 / 2 / 70 | 0 | 0 | 2 (3) | 0.9993 | 0 viol, 0 cex |
| x2 | 2 | reserved, spread hubs (40 m) | 73 / 76 | 3 | 0 | 333 / 774 | 72 + 1 (0) | 0 / 0 / 0 | 0 | 0 | 0 (4) | 0.9997 | 0 viol, 0 cex |
| x2 | 3 | reserved, spread hubs (40 m) | 64 / 64 | 0 | 0 | 232 / 495 | 62 + 1 (1) | 0 / 0 / 0 | 1 | 0 | 0 (1) | 0.9999 | 0 viol, 0 cex |
| x2 | 1 | reserved, mismatched plant | 43 / 65 | 22 | 3 | 539 / 890 | 41 + 1 (1) | 5 / 30 / 140 | 1 | 0 | 3 (2) | 0.9998 | 0 viol, 0 cex |
| x2 | 2 | reserved, mismatched plant | 42 / 76 | 34 | 4 | 497 / 890 | 42 + 0 (0) | 9 / 0 / 250 | 0 | 0 | 1 (3) | 1.0000 | 0 viol, 0 cex |
| x2 | 3 | reserved, mismatched plant | 42 / 64 | 22 | 4 | 578 / 879 | 42 + 0 (0) | 5 / 43 / 100 | 1 | 0 | 1 (0) | 1.0000 | 0 viol, 0 cex |
| x2 | 1 | reserved or tactical, compact | 49 / 65 | 16 | 6 | 491 / 890 | 47 + 1 (1) | 14 / 110 / 120 | 1 | 0 | 2 (3) | 1.0000 | 0 viol, 0 cex |
| x2 | 2 | reserved or tactical, compact | 53 / 76 | 23 | 21 | 401 / 874 | 52 + 0 (1) | 8 / 50 / 110 | 2 | 0 | 3 (1) | 1.0000 | 0 viol, 0 cex |
| x2 | 3 | reserved or tactical, compact | 47 / 64 | 17 | 9 | 584 / 897 | 45 + 0 (2) | 8 / 64 / 110 | 0 | 0 | 0 (1) | 1.0000 | 0 viol, 0 cex |
| x2 | 1 | reserved or tactical, spread | 65 / 65 | 0 | 1 | 307 / 652 | 59 + 3 (3) | 1 / 0 / 70 | 0 | 0 | 2 (3) | 0.9993 | 0 viol, 0 cex |
| x2 | 2 | reserved or tactical, spread | 75 / 76 | 1 | 0 | 342 / 773 | 73 + 2 (0) | 1 / 0 / 50 | 0 | 0 | 0 (4) | 1.0000 | 0 viol, 0 cex |
| x2 | 3 | reserved or tactical, spread | 64 / 64 | 0 | 0 | 232 / 495 | 62 + 1 (1) | 0 / 0 / 0 | 1 | 0 | 0 (1) | 0.9999 | 0 viol, 0 cex |
| x5 | 1 | reserved, compact hubs | 51 / 172 | 121 | 6 | 523 / 900 | 49 + 1 (1) | 10 / 68 / 100 | 5 | 1 | 4 (5) | 0.9996 | 0 viol, 0 cex |
| x5 | 2 | reserved, compact hubs | 53 / 183 | 130 | 9 | 515 / 900 | 52 + 0 (1) | 8 / 74 / 110 | 4 | 1 | 7 (3) | 1.0000 | 0 viol, 0 cex |
| x5 | 3 | reserved, compact hubs | 53 / 160 | 107 | 1 | 553 / 900 | 50 + 0 (3) | 2 / 0 / 80 | 3 | 0 | 4 (2) | 1.0000 | 0 viol, 0 cex |
| x5 | 1 | reserved, spread hubs (40 m) | 89 / 172 | 83 | 0 | 486 / 890 | 85 + 2 (2) | 0 / 0 / 35 | 2 | 0 | 3 (5) | 0.9998 | 0 viol, 0 cex |
| x5 | 2 | reserved, spread hubs (40 m) | 99 / 183 | 84 | 1 | 542 / 900 | 93 + 2 (4) | 1 / 0 / 30 | 3 | 4 | 7 (3) | 0.9983 | 0 viol, 0 cex |
| x5 | 3 | reserved, spread hubs (40 m) | 91 / 160 | 69 | 0 | 553 / 890 | 90 + 1 (0) | 0 / 0 / 0 | 0 | 1 | 2 (3) | 0.9994 | 0 viol, 0 cex |
| x5 | 1 | reserved or tactical, compact | 62 / 172 | 110 | 43 | 385 / 890 | 59 + 0 (3) | 18 / 72 / 120 | 5 | 0 | 6 (3) | 0.9997 | 0 viol, 0 cex |
| x5 | 2 | reserved or tactical, compact | 62 / 183 | 121 | 33 | 439 / 890 | 58 + 0 (4) | 23 / 143 / 250 | 8 | 2 | 9 (1) | 0.9999 | 0 viol, 0 cex |
| x5 | 3 | reserved or tactical, compact | 63 / 160 | 97 | 20 | 479 / 899 | 58 + 0 (5) | 23 / 153 / 200 | 10 | 0 | 5 (1) | 1.0000 | 0 viol, 0 cex |
| x5 | 1 | reserved or tactical, spread | 101 / 172 | 71 | 4 | 502 / 900 | 92 + 4 (5) | 4 / 30 / 70 | 2 | 0 | 3 (5) | 0.9980 | 0 viol, 0 cex |
| x5 | 2 | reserved or tactical, spread | 115 / 183 | 68 | 6 | 537 / 890 | 108 + 1 (6) | 4 / 30 / 55 | 2 | 2 | 7 (3) | 0.9988 | 0 viol, 0 cex |
| x5 | 3 | reserved or tactical, spread | 102 / 160 | 58 | 1 | 554 / 890 | 99 + 2 (1) | 3 / 26 / 150 | 0 | 1 | 0 (4) | 0.9998 | 0 viol, 0 cex |
