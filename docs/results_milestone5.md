# Milestone 5 results: hub sequencing

All runs: `mini_town`, the digital twin with the Tier A plant (matched unless noted), ORCA + FCFS scheduler,
seeds 1-3, demand x1 / x2 / x5. Every configuration flies the same missions: inbound flights end their cruise
at an approach fix 100 m from the hub centre on the inbound bearing, then approach the pad. Separation
standard everywhere, the terminal area included: 30 m horizontal or 10 m vertical (an assumption, as in every
milestone). Terminal boxes use a 5 m lateral buffer, en-route boxes 20 m.

* **off**: no hub sequencing; terminal boxes are not certified (milestone 4 semantics).
* **sequenced, compact hubs**: the mini_town hubs as built (pads 8 m apart, so every pad is within 30 m of
  every other: each hub is one runway).
* **sequenced, spread hubs**: the same hubs with 40 m pad spacing (30 m + 2 x 5 m buffers): every pad
  column certifiable against every other.
* **sequenced, mismatched plant**: compact hubs, with the milestone 4.5 plant (fleet 15 % slower than
  planned, one vehicle degraded mid-flight); the twin calibrates and replans as before.

## Totals over seeds 1-3

| scale | hubs | runs | flown / requests | rejected | terminal LoS | en-route LoS | arrivals certified | holding mean / p95 s | ground delay mean s | departures moved |
|---|---|---|---|---|---|---|---|---|---|---|
| x1 | off | 3 | 106 / 106 | 0 | 59 | 0 | - | - | 35 | 0 |
| x1 | sequenced, compact hubs | 3 | 106 / 106 | 0 | 3 | 0 | 103 / 106 | 13 / 53 | 129 | 22 |
| x1 | sequenced, spread hubs (40 m) | 3 | 106 / 106 | 0 | 1 | 0 | 104 / 106 | 2 / 8 | 41 | 0 |
| x2 | off | 3 | 205 / 205 | 0 | 264 | 0 | - | - | 90 | 0 |
| x2 | sequenced, compact hubs | 3 | 178 / 205 | 27 | 29 | 0 | 155 / 178 | 18 / 61 | 319 | 126 |
| x2 | sequenced, spread hubs (40 m) | 3 | 205 / 205 | 0 | 10 | 4 | 179 / 205 | 6 / 30 | 109 | 5 |
| x2 | sequenced, mismatched plant | 3 | 176 / 205 | 29 | 44 | 0 | 148 / 176 | 12 / 54 | 322 | 128 |
| x5 | off | 3 | 424 / 515 | 91 | 907 | 2 | - | - | 459 | 0 |
| x5 | sequenced, compact hubs | 3 | 225 / 515 | 290 | 63 | 3 | 185 / 225 | 19 / 73 | 464 | 236 |
| x5 | sequenced, spread hubs (40 m) | 3 | 383 / 515 | 132 | 40 | 18 | 319 / 383 | 10 / 48 | 462 | 70 |

Figure: `docs/figures/milestone5_hubs.png`.

* **Terminal-area loss of separation falls 94-98 %.** x1: 59 -> 3 (compact) / 1 (spread); x2: 264 -> 29 / 10;
  x5: 907 -> 63 / 40.
* **No loss of separation between two certified flights, anywhere.** Every remaining event, terminal or en
  route, in all 21 sequenced runs involves a contingent landing (no certified slot within 300 s of holding) or
  an emergency that flew uncertified by policy (S19). The verified audit of all 30 runs: 0 claim violations,
  0 counterexamples; every LoS event is classified "uncertified". This is what Theorem 4 + Theorem 2 predict.
* **Under 30 m / 10 m, the hub layout sets the capacity.** A compact hub is one runway: at x2 it cannot keep
  up (27 of 205 requests rejected, 126 departures moved for arrivals, mean ground delay 319 s against 90 s);
  at x5 it flies 225 of 515 requests. The spread layout flies everything at x2 (mean ground delay 109 s, 5
  departures moved, arrivals hold 6 s on average) and 383 of 515 at x5, against 424 of 515 for strategic
  deconfliction alone (which then has 907 terminal conflicts). The trade is footprint: the spread west hub
  covers 80 x 40 m instead of 16 x 8 m.
* **Holding stays short.** Mean / p95 arrival holding: compact 13-19 / 53-73 s, spread 2-10 / 8-48 s; never
  above 190 s.
* **Residual risk = contingent landings.** Share of arrivals that found no certified slot: x1 3 % / 2 %, x2 13 %
  / 13 %, x5 18 % / 17 % (compact / spread), 16 % on the mismatched plant. About a quarter (14-33 %) are uncertified
  emergencies; most of the rest are inbound streams that share an approach fix, where any plan for the
  leading flight (holding included) blocks the flight behind it. That is the next thing to fix (holding
  stacks per bearing, or sequencing the stream as a whole).
  *Milestone 5.5 found the actual cause (arrivals were never promised a landing) and fixed it with landing
  reservations: `docs/results_milestone5_5.md`.*
* **Model mismatch.** On the mismatched plant the twin's calibration and replanning still hold
  conformance at 0.9995-0.9998; terminal LoS is 44 against 29 on the matched plant, all involving contingent
  or uncertified flights.

## Theorem 4

`proofs/UtmProofs/Registry.lean`: the registry protocol that admission, replanning and the hub sequencer all
follow (withdraw, check against every other flight, register as certified only if all checks pass; force-
register what must fly anyway; prune) keeps every pair of certified boxes of different flights cylinder-
separated (`inv_steps`), so two flights inside certified boxes are never in loss of separation
(`registry_no_los`). Standard axioms only.

## Design notes from building it

* A 20 m lateral buffer in the terminal area made a pad column 40 m wide: no two pads could ever operate
  together and holding points 36 deg apart conflicted. 5 m terminal buffers, sized from PX4's measured
  tracking (3.5 m p99, milestone 4.5), fix that; Theorem 3 is what licenses it.
* Descending from the cruise layers inside the hub column took 30-80 s per landing (1.5 m/s). Descending on
  the holding ring and crossing to the pad at 25 m cuts the column time to ~17 s.
* A 70 m ring worked for compact hubs but not spread ones: the ring must keep holding boxes 30 m clear of
  every pad column, i.e. farthest pad + 55 m. With 70 m, a quarter of spread-layout arrivals failed.
* Retrying only every 2 s let flights slip past their fix between attempts and fly into the hub uncertified;
  the final attempt now always comes before the fix. A first "hold at the fix" fallback registered holds
  without checking them and produced 151 en-route conflicts in one run; holds are now certified like
  everything else.

## Per-run table

| scale | seed | hubs | flown / requests | rejected | departures moved | ground delay mean / p95 s | arrivals certified (contingent) | holding mean / p95 / max s | terminal LoS | en-route LoS | conformance (truth) | audit |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| x1 | 1 | off | 31 / 31 | 0 | 0 | 31 / 150 | - | - | 21 | 0 | 1.0000 | 0 viol, 0 cex |
| x1 | 2 | off | 37 / 37 | 0 | 0 | 36 / 170 | - | - | 15 | 0 | 1.0000 | 0 viol, 0 cex |
| x1 | 3 | off | 38 / 38 | 0 | 0 | 37 / 180 | - | - | 23 | 0 | 1.0000 | 0 viol, 0 cex |
| x1 | 1 | sequenced, compact hubs | 31 / 31 | 0 | 10 | 168 / 450 | 31 (0) | 11 / 48 / 55 | 0 | 0 | 1.0000 | 0 viol, 0 cex |
| x1 | 2 | sequenced, compact hubs | 37 / 37 | 0 | 5 | 79 / 222 | 35 (2) | 14 / 60 / 110 | 0 | 0 | 1.0000 | 0 viol, 0 cex |
| x1 | 3 | sequenced, compact hubs | 38 / 38 | 0 | 7 | 138 / 464 | 37 (1) | 13 / 52 / 90 | 3 | 0 | 1.0000 | 0 viol, 0 cex |
| x1 | 1 | sequenced, spread hubs (40 m) | 31 / 31 | 0 | 0 | 34 / 140 | 30 (1) | 4 / 19 / 90 | 0 | 0 | 1.0000 | 0 viol, 0 cex |
| x1 | 2 | sequenced, spread hubs (40 m) | 37 / 37 | 0 | 0 | 38 / 114 | 36 (1) | 0 / 0 / 0 | 1 | 0 | 1.0000 | 0 viol, 0 cex |
| x1 | 3 | sequenced, spread hubs (40 m) | 38 / 38 | 0 | 0 | 52 / 173 | 38 (0) | 2 / 5 / 50 | 0 | 0 | 1.0000 | 0 viol, 0 cex |
| x2 | 1 | off | 65 / 65 | 0 | 0 | 85 / 258 | - | - | 75 | 0 | 0.9994 | 0 viol, 0 cex |
| x2 | 2 | off | 76 / 76 | 0 | 0 | 98 / 298 | - | - | 111 | 0 | 0.9999 | 0 viol, 0 cex |
| x2 | 3 | off | 64 / 64 | 0 | 0 | 86 / 228 | - | - | 78 | 0 | 1.0000 | 0 viol, 0 cex |
| x2 | 1 | sequenced, compact hubs | 56 / 65 | 9 | 37 | 287 / 710 | 46 (10) | 14 / 49 / 60 | 14 | 0 | 0.9999 | 0 viol, 0 cex |
| x2 | 2 | sequenced, compact hubs | 64 / 76 | 12 | 47 | 329 / 828 | 57 (7) | 21 / 72 / 150 | 11 | 0 | 1.0000 | 0 viol, 0 cex |
| x2 | 3 | sequenced, compact hubs | 58 / 64 | 6 | 42 | 341 / 675 | 52 (6) | 21 / 62 / 120 | 4 | 0 | 1.0000 | 0 viol, 0 cex |
| x2 | 1 | sequenced, spread hubs (40 m) | 65 / 65 | 0 | 1 | 123 / 306 | 55 (10) | 7 / 35 / 45 | 2 | 1 | 0.9981 | 0 viol, 0 cex |
| x2 | 2 | sequenced, spread hubs (40 m) | 76 / 76 | 0 | 3 | 109 / 300 | 66 (10) | 7 / 34 / 130 | 7 | 3 | 0.9959 | 0 viol, 0 cex |
| x2 | 3 | sequenced, spread hubs (40 m) | 64 / 64 | 0 | 1 | 94 / 242 | 58 (6) | 4 / 22 / 55 | 1 | 0 | 1.0000 | 0 viol, 0 cex |
| x2 | 1 | sequenced, mismatched plant | 57 / 65 | 8 | 34 | 305 / 728 | 42 (15) | 9 / 49 / 55 | 20 | 0 | 0.9998 | 0 viol, 0 cex |
| x2 | 2 | sequenced, mismatched plant | 64 / 76 | 12 | 48 | 295 / 757 | 55 (9) | 13 / 56 / 90 | 20 | 0 | 0.9998 | 0 viol, 0 cex |
| x2 | 3 | sequenced, mismatched plant | 55 / 64 | 9 | 46 | 367 / 805 | 51 (4) | 13 / 58 / 150 | 4 | 0 | 0.9995 | 0 viol, 0 cex |
| x5 | 1 | off | 136 / 172 | 36 | 0 | 452 / 870 | - | - | 288 | 0 | 0.9999 | 0 viol, 0 cex |
| x5 | 2 | off | 156 / 183 | 27 | 0 | 469 / 890 | - | - | 349 | 2 | 0.9989 | 0 viol, 0 cex |
| x5 | 3 | off | 132 / 160 | 28 | 0 | 454 / 874 | - | - | 270 | 0 | 1.0000 | 0 viol, 0 cex |
| x5 | 1 | sequenced, compact hubs | 74 / 172 | 98 | 68 | 458 / 877 | 61 (13) | 21 / 80 / 130 | 24 | 1 | 0.9999 | 0 viol, 0 cex |
| x5 | 2 | sequenced, compact hubs | 77 / 183 | 106 | 95 | 453 / 872 | 66 (11) | 21 / 78 / 190 | 20 | 0 | 0.9999 | 0 viol, 0 cex |
| x5 | 3 | sequenced, compact hubs | 74 / 160 | 86 | 73 | 481 / 880 | 58 (16) | 14 / 62 / 110 | 19 | 2 | 0.9997 | 0 viol, 0 cex |
| x5 | 1 | sequenced, spread hubs (40 m) | 125 / 172 | 47 | 16 | 494 / 880 | 104 (21) | 11 / 54 / 70 | 13 | 5 | 0.9966 | 0 viol, 0 cex |
| x5 | 2 | sequenced, spread hubs (40 m) | 137 / 183 | 46 | 32 | 444 / 882 | 108 (29) | 10 / 50 / 60 | 19 | 8 | 0.9947 | 0 viol, 0 cex |
| x5 | 3 | sequenced, spread hubs (40 m) | 121 / 160 | 39 | 22 | 447 / 880 | 107 (14) | 9 / 40 / 55 | 8 | 5 | 0.9963 | 0 viol, 0 cex |
