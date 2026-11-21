"""Digital twin core: the closed loop between a plant (Tier A stand-in, replayed log, or PX4) and the
traffic-management model.

Every tick (plant clock):
  1. ingest telemetry -> state estimates (`TwinState`);
  2. admit requests as they arrive (incremental strategic scheduler, Theorem 2 certification) and
     dispatch the scheduled plan to the plant, or cancel;
  3. conformance: is each vehicle inside its declared intent boxes? (Lean-verified `inside`)
     -> on confirmed non-conformance, replan it from its measured position and re-certify;
  4. calibration: measured cruise / climb / descent speeds vs the planning model -> re-calibrate;
  5. prediction: every `every_s`, fork a Tier A rollout from the estimated state for `horizon_s` and
     record predicted positions and predicted loss of separation; scored against what happens.
"""
from __future__ import annotations

import copy
import time
from collections import defaultdict
from dataclasses import asdict, dataclass, replace

import numpy as np

from ..intents import IntentParams, _leg_time, mission_intents
from ..missions import Mission
from ..osm import Airspace
from ..scheduler import Decision, FcfsScheduler
from ..sim import SimParams, Simulation, leg_of
from ..vehicle import VehicleParams
from .state import ConformanceMonitor, SpeedCalibrator, TwinState
from .telemetry import Plant


LIFTOFF_PROBE_M = 5.0


@dataclass(frozen=True)
class TwinParams:
    tick_s: float = 0.2                 # twin update period (plant clock)
    strategic: bool = True              # False: admit every request as filed, uncertified (prediction benchmark)
    replan: bool = True                 # replan non-conforming flights
    replan_cooldown_s: float = 20.0
    max_hover_s: float = 120.0          # airborne delay allowed when replanning
    calibrate: bool = True              # adopt re-calibrated speeds for new plans and predictions
    calibrate_every_s: float = 30.0
    predict_every_s: float = 10.0
    horizon_s: float = 60.0
    rollout_dt: float = 0.2
    sample_h: tuple = (10.0, 30.0, 60.0)
    prune_lag_s: float = 60.0
    log_state_hz: float = 1.0


class Predictor:
    def __init__(self, air: Airspace, sp: SimParams, policy_factory, tp: TwinParams):
        self.air, self.policy_factory, self.tp = air, policy_factory, tp
        self.sp = replace(sp, dt=tp.rollout_dt, log_hz=1.0 / tp.rollout_dt)

    def predict(self, t: float, vids: list[int], routes: dict, state: TwinState, waiting: list[int],
                vp: VehicleParams, caps: dict | None = None) -> dict:
        ids = list(vids) + [w for w in waiting if routes[w].release_t <= t + self.tp.horizon_s]
        if not ids:
            return {"t": t, "pos": {}, "los": []}
        missions = [routes[i] for i in ids]
        pos = np.array([state.estimate(i, t) if i in vids else routes[i].meta["start"] for i in ids], float)
        vel = np.array([state.tracks[i].vel if i in vids else np.zeros(3) for i in ids], float)
        st = np.array(["flying" if i in vids else "waiting" for i in ids], dtype=object)
        wpi = np.array([min(state.tracks[i].wp_idx, len(routes[i].waypoints) - 1) if i in vids else 0 for i in ids], int)
        hold = np.array([state.tracks[i].hold_until if i in vids else -1.0 for i in ids], float)
        sim = Simulation.from_state(self.air, missions, vp, self.sp, self.policy_factory(self.sp.dt), t, pos, vel, st,
                                    wpi, hold)
        for j, i in enumerate(ids):
            if caps and i in caps:
                sim.fleet.v_cap[j] = caps[i]
        out_pos = {}
        marks = sorted(self.tp.sample_h)
        k = 0
        while sim.fleet.t < t + self.tp.horizon_s - 1e-9:
            if not sim.step():
                break
            while k < len(marks) and sim.fleet.t >= t + marks[k] - 1e-9:
                on = sim.fleet.on
                out_pos[marks[k]] = {int(ids[j]): sim.fleet.pos[j].round(2).tolist() for j in np.flatnonzero(on)}
                k += 1
        los = [{"a": min(int(ids[e["a"]]), int(ids[e["b"]])), "b": max(int(ids[e["a"]]), int(ids[e["b"]])),
                "t_onset": e["t_start"], "min_dist_m": round(e["min_dist_m"], 1)}
               for e in sim.los + list(sim.open_pairs.values()) if not e["terminal"]]
        return {"t": round(t, 2), "pos": out_pos, "los": los}


class DigitalTwin:
    def __init__(self, air: Airspace, plant: Plant, requests: list[Mission], scheduler: FcfsScheduler,
                 vp: VehicleParams, ip: IntentParams, sp: SimParams, policy_factory, tp: TwinParams = TwinParams(),
                 checker=None, log=None, echo=print, sequencer=None, vp_cfg: VehicleParams | None = None):
        self.air, self.plant, self.sched, self.ip, self.sp, self.tp = air, plant, scheduler, ip, sp, tp
        self.vp_plan = vp
        self.requests = sorted(requests, key=lambda m: (m.release_t, m.id))
        self.next_req = 0
        self.state = TwinState()
        self.monitor = ConformanceMonitor(checker)
        self.calib = SpeedCalibrator(vp, cfg=vp_cfg)
        self.seq = sequencer                      # swarm_sim.hubs.HubSequencer (milestone 5) or None
        self.seq_retry: dict[int, float] = {}
        self.arrivals: list = []
        self.predictor = Predictor(air, sp, policy_factory, tp)
        self.log, self.echo = log, echo
        self.routes: dict[int, Mission] = {}
        self.status: dict[int, str] = {}          # waiting | flying | landed | cancelled
        self.decisions: list = []
        self.replans: list = []
        self.alerts: list = []
        self.intent_versions: list = []           # every registered plan version, for the offline audit
        self.last_replan: dict[int, float] = {}
        self.leg_start: dict[int, tuple] = {}     # vid -> (wp_idx, t) of the leg in progress
        self.legs: list = []                      # completed legs: measured vs planned durations
        self.predictions: list = []
        self.pos_err = defaultdict(list)          # horizon -> errors (m)
        self.pending_checks: list = []            # (due_t, horizon, {vid: predicted pos})
        self.t_last = {"cal": 0.0, "pred": -1e9, "prune": 0.0, "log": -1e9}
        self.caps: dict[int, float] = {}         # per-vehicle speed caps the twin has measured (degraded vehicles)
        self.liftoff: dict[int, float] = {}      # release -> lift-off lag per flight (s)
        self.wall0 = time.time()

    # ------------------------------------------------------------------ helpers
    def _vehicle_vp(self, vid: int) -> VehicleParams | None:
        """This vehicle's own model if it is measurably slower than the planning model (degraded, or the
        fleet model is not calibrated yet)."""
        v = self.calib.vehicle_speed(vid)
        if v is None or v >= 0.97 * self.vp_plan.cruise_xy:
            return None
        return replace(self.vp_plan, cruise_xy=round(v, 2), v_xy_max=round(v, 2))

    def _register_version(self, vid: int, m: Mission, status: str, now: float, reason: str,
                          vp: VehicleParams | None = None):
        boxes = mission_intents(m, vp or self.sched.vp, self.ip)
        self.monitor.set_intents(vid, boxes)
        for v in self.intent_versions:
            if v["vid"] == vid and v["valid_to"] is None:
                v["valid_to"] = round(now, 3)
        self.intent_versions.append({"vid": vid, "valid_from": round(now, 3), "valid_to": None, "status": status,
                                     "reason": reason, "int": [b.as_int() for b in boxes],
                                     "phase": [b.phase for b in boxes], "flow": [b.flow for b in boxes],
                                     "cert": self.sched.certified(boxes, m), "mission": m.to_json(),
                                     "vehicle": asdict(vp or self.sched.vp), "takeoff_s": self.ip.takeoff_s})

    def _emit(self, topic: str, now: float, msg: dict):
        if self.log is not None:
            self.log.write(topic, now, msg)

    # ------------------------------------------------------------------ the loop
    def tick(self):
        now = self.plant.now()
        # 1. telemetry
        frame = []
        for z in self.plant.poll():
            self.state.ingest(z)
            frame.append({"vid": z.vid, "t": round(z.t, 3), "pos": [round(float(v), 3) for v in z.pos],
                          "vel": [round(float(v), 3) for v in z.vel], "state": z.state, "wp_idx": z.wp_idx,
                          "hold_until": round(z.hold_until, 2), "route_ver": z.route_ver})
            if z.state == "flying" and self.status.get(z.vid) == "waiting":
                self.status[z.vid] = "flying"
            if (z.state == "flying" and z.vid in self.routes and z.vid not in self.liftoff
                    and z.pos[2] > self.routes[z.vid].meta["pad"][2] + LIFTOFF_PROBE_M):
                # lag = time to pass 5 m above the pad minus the model's climb time to 5 m (an estimator's
                # altitude on the pad wanders by ~0.5 m, so a lower threshold measures noise)
                self.liftoff[z.vid] = round(z.t - self.routes[z.vid].release_t - LIFTOFF_PROBE_M / self.vp_plan.v_up_max, 2)
            self._legs(z)
            if z.state == "landed" and self.status.get(z.vid) != "landed":
                self.status[z.vid] = "landed"
                self.monitor.drop(z.vid)
                self.sched.withdraw(z.vid)
                for v in self.intent_versions:
                    if v["vid"] == z.vid and v["valid_to"] is None:
                        v["valid_to"] = round(now, 3)
        if frame:
            self._emit("/twin/telemetry", now, {"samples": frame})
        # 2. admission
        while self.next_req < len(self.requests) and self.requests[self.next_req].release_t <= now + self.tp.tick_s:
            m = self.requests[self.next_req]
            self.next_req += 1
            if self.tp.strategic:
                v, dec, _ = self.sched.admit(m, key=m.id)
            else:
                v = copy.deepcopy(m)
                v.meta = dict(v.meta) | {"schedule_status": "unscheduled"}
                self.sched.register(m.id, self.sched._boxes(v))
                dec = Decision(m.id, m.kind, m.release_t, m.release_t, 0.0, 0, 0, "unscheduled", 0)
            d = asdict(dec)
            self.decisions.append(d)
            self._emit("/twin/decision", now, d)
            if v is None:
                self.plant.cancel(m.id)
                self.status[m.id] = "cancelled"
                continue
            v.meta = dict(v.meta) | {"pad": list(v.meta["start"])}
            self.routes[m.id] = v
            self.status[m.id] = "waiting"
            self.plant.dispatch(m.id, v)
            self._register_version(m.id, v, dec.status, now, "admit")
        # 3. arrival sequencing (hub slots), conformance (+ replans) and 4. calibration samples
        flying = [k for k in self.state.flying() if k in self.routes]
        if self.seq is not None:
            for vid in flying:
                tr = self.state.tracks[vid]
                est = self.state.estimate(vid, now)
                eta = self.seq.eta(self.routes[vid], tr.wp_idx, est, tr.vel)
                if eta is not None and eta <= self.seq.hp.lead_s and now >= self.seq_retry.get(vid, -1e9):
                    # final within one retry of the last moment, so the final attempt always comes before the fix
                    self._sequence(vid, est, now, final=eta <= self.seq.hp.final_eta_s + self.seq.hp.retry_s)
        for vid in flying:
            tr = self.state.tracks[vid]
            est = self.state.estimate(vid, now)
            route = self.routes[vid]
            if tr.wp_idx < len(route.waypoints):
                self.calib.observe(tr, *leg_of(route, tr.wp_idx), vid=vid)
            alert = self.monitor.check(vid, est, now)
            if alert is None:
                if (not self.monitor.ok.get(vid, True) and self.tp.replan and tr.wp_idx < len(route.waypoints) - 1
                        and now - self.last_replan.get(vid, -1e9) >= self.tp.replan_cooldown_s):
                    self._replan(vid, tr, est, now)            # still non-conforming after the cooldown: try again
                continue
            landing = (alert["event"] == "nonconforming" and tr.wp_idx >= len(route.waypoints) - 1
                       and alert["phase"] in ("terminal", "late"))
            if landing:
                alert["event_detail"] = "landing_overrun"      # final descent at the pad: hub sequencing (step 7)
            self.alerts.append(alert)
            self._emit("/twin/alert", now, alert)
            if alert["event"] != "nonconforming" or not self.tp.replan or landing:
                continue
            if now - self.last_replan.get(vid, -1e9) >= self.tp.replan_cooldown_s:
                self._replan(vid, tr, est, now)
        if self.tp.calibrate and now - self.t_last["cal"] >= self.tp.calibrate_every_s:
            self.t_last["cal"] = now
            self._calibrate_takeoff(now)
            vp = self.calib.maybe_update(now)
            if vp is not None:
                self.sched.vp = vp
                self.vp_plan = vp
                self._emit("/twin/calibration", now, self.calib.updates[-1])
                self.echo(f"  t={now:7.1f}  calibration: {self.calib.updates[-1]}")
        # 5. prediction and its scoring
        self._score_positions(now)
        if now - self.t_last["pred"] >= self.tp.predict_every_s and flying:
            self.t_last["pred"] = now
            waiting = [k for k, s in self.status.items() if s == "waiting"]
            p = self.predictor.predict(now, flying, self.routes, self.state, waiting, self.vp_plan, self.caps)
            self.predictions.append({"t": p["t"], "los": p["los"]})
            for h, pos in p["pos"].items():
                self.pending_checks.append((now + h, h, pos))
            self._emit("/twin/prediction", now, p)
        if now - self.t_last["prune"] >= 10.0:
            self.t_last["prune"] = now
            self.sched.prune(now - self.tp.prune_lag_s)
        if now - self.t_last["log"] >= 1.0 / self.tp.log_state_hz:
            self.t_last["log"] = now
            self._emit("/twin/state", now, {"vehicles": {
                str(k): {"pos": [round(float(v), 2) for v in self.state.estimate(k, now)],
                         "conforming": bool(self.monitor.ok.get(k, True)), "wp_idx": self.state.tracks[k].wp_idx}
                for k in flying}})
            tru = self.plant.truth()
            if tru:
                on = np.flatnonzero(tru["on"])
                self._emit("/plant/truth", now, {"vehicles": {str(int(i)): tru["pos"][i].round(2).tolist() for i in on}})

    def _calibrate_takeoff(self, now):
        """Fleet lift-off lag (median of the last 20 flights, at least 3); adopted when it is over 1 s and
        differs from the model by more than 0.5 s. Measured as the time to pass 5 m above the pad minus the
        model's climb time to 5 m."""
        lags = list(self.liftoff.values())[-20:]
        if len(lags) < 3:
            return
        med = float(np.median(lags))
        cur = self.ip.takeoff_s
        target = round(med, 2) if med > 1.0 else 0.0
        if abs(target - cur) > 0.5:
            self.ip = replace(self.ip, takeoff_s=target)
            self.sched.ip = self.ip
            upd = {"t": round(now, 2), "takeoff_s": target, "flights": len(lags)}
            self.calib.updates.append(upd)
            self._emit("/twin/calibration", now, upd)
            self.echo(f"  t={now:7.1f}  calibration: {upd}")

    def _sequence(self, vid, est, now, final=True):
        vvp = self._vehicle_vp(vid)
        bumpable = frozenset(k for k, st in self.status.items()
                             if st == "waiting" and k in self.routes and self.routes[k].kind != "emergency"
                             and self.routes[k].release_t > now + 5.0)
        res = self.seq.sequence(vid, self.routes[vid], est, now, vvp, final, bumpable)
        if res is None:
            self.seq_retry[vid] = now + self.seq.hp.retry_s
            return
        v, status, hold = res
        for k in self.seq.decisions[-1].bumped:
            self._readmit(k, now)
        if status == "awaiting":
            self.seq_retry[vid] = now + self.seq.hp.await_retry_s
        self.routes[vid] = v
        self.plant.set_route(vid, v)
        self.state.rerouted(vid)
        self.leg_start.pop(vid, None)
        self._register_version(vid, v, status, now, "arrival", vvp)
        d = self.seq.decisions[-1]
        rec = {"t": round(now, 2), "vid": vid, "hub": d.hub, "status": status, "hold_s": d.hold_s, "pre_hold_s": d.pre_hold_s,
               "slot": list(d.slot), "pad": d.pad, "candidates": d.candidates}
        self.arrivals.append(rec)
        self._emit("/twin/arrival", now, rec)

    def _readmit(self, k, now):
        """A ground departure displaced by an arrival: admit its request again (later slot) and re-dispatch."""
        req = next(r for r in self.requests if r.id == k)
        self.sched.withdraw(k)
        v, dec, _ = self.sched.admit(req, key=k, now=now)
        d = asdict(dec) | {"rescheduled": True}
        self.decisions.append(d)
        self._emit("/twin/decision", now, d)
        if v is None:
            self.plant.cancel(k)
            self.status[k] = "cancelled"
            for x in self.intent_versions:
                if x["vid"] == k and x["valid_to"] is None:
                    x["valid_to"] = round(now, 3)
            return
        v.meta = dict(v.meta) | {"pad": list(v.meta["start"])}
        self.routes[k] = v
        self.plant.dispatch(k, v)
        self._register_version(k, v, dec.status, now, "rescheduled")

    def _replan(self, vid, tr, est, now):
        route = self.routes[vid]
        wi = min(tr.wp_idx, len(route.waypoints) - 1)
        rest = copy.deepcopy(route.waypoints[wi:])
        if tr.hold_until >= 0:
            rest[0].hold_s = max(0.0, tr.hold_until - now)
        m = copy.deepcopy(route)
        m.waypoints = rest
        m.meta = dict(route.meta) | {"start": [float(v) for v in est]}
        vvp = self._vehicle_vp(vid)
        v, status, hover, _ = self.sched.replan(vid, m, now, self.tp.max_hover_s, vvp)
        if vvp is not None:
            v.meta = dict(v.meta) | {"vehicle_cruise_xy": vvp.cruise_xy}
            self.caps[vid] = vvp.cruise_xy
        self.routes[vid] = v
        self.plant.set_route(vid, v)
        self.state.rerouted(vid)
        self.last_replan[vid] = now
        self.leg_start.pop(vid, None)
        self._register_version(vid, v, status, now, "replan", vvp)
        rec = {"t": round(now, 2), "vid": vid, "status": status, "hover_s": hover, "from_wp": wi,
               "vehicle_cruise_xy": vvp.cruise_xy if vvp else None,
               "pos": [round(float(x), 1) for x in est]}
        self.replans.append(rec)
        self._emit("/twin/replan", now, rec)

    def _legs(self, z):
        """Completed legs: measured duration vs the planning model's (model at admission and current)."""
        if z.vid not in self.routes:
            return
        tr = self.state.tracks.get(z.vid)
        if tr is not None and z.route_ver < tr.route_ver:
            return
        cur = self.leg_start.get(z.vid)
        if cur is None:
            self.leg_start[z.vid] = (z.wp_idx, z.t)
            return
        wi, t0 = cur
        if z.wp_idx == wi:
            return
        route = self.routes[z.vid]
        if z.wp_idx == wi + 1 and wi < len(route.waypoints) and wi > 0:
            a, b = leg_of(route, wi)
            w = route.waypoints[wi]
            dur = z.t - t0 - w.hold_s
            if dur > 0:
                self.legs.append({"vid": z.vid, "tag": w.tag, "dxy": round(float(np.hypot(b[0] - a[0], b[1] - a[1])), 1),
                                  "dz": round(b[2] - a[2], 1), "measured_s": round(dur, 2),
                                  "planned_s": round(_leg_time(a, b, self.calib.vp0), 2),
                                  "planned_cal_s": round(_leg_time(a, b, self.vp_plan), 2)})
        self.leg_start[z.vid] = (z.wp_idx, z.t)

    def _score_positions(self, now):
        if not self.pending_checks:
            return
        due = [c for c in self.pending_checks if c[0] <= now]
        if not due:
            return
        self.pending_checks = [c for c in self.pending_checks if c[0] > now]
        tru = self.plant.truth()
        for _, h, pos in due:
            for vid, p in pos.items():
                if tru and tru["on"][vid]:
                    actual = tru["pos"][vid]
                elif vid in self.state.tracks and self.state.tracks[vid].state == "flying":
                    actual = self.state.estimate(vid, now)
                else:
                    continue
                self.pos_err[h].append(float(np.linalg.norm(np.asarray(p) - actual)))

    # ------------------------------------------------------------------ run + report
    def run(self, max_t: float) -> dict:
        nxt = self.plant.now()
        while self.plant.now() < max_t:
            alive = self.plant.advance()
            if self.plant.now() >= nxt - 1e-9:
                nxt = self.plant.now() + self.tp.tick_s
                self.tick()
            if not alive and self.next_req >= len(self.requests):
                break
        for v in self.intent_versions:
            if v["valid_to"] is None:
                v["valid_to"] = round(self.plant.now(), 3)
        return self.report()

    def report(self) -> dict:
        final = {}
        for d in self.decisions:                  # a moved departure has a second decision: count the last one
            final[d["mission"]] = d
        st = defaultdict(int)
        for d in final.values():
            st[d["status"]] += 1
        st["rescheduled"] = sum(1 for d in self.decisions if d.get("rescheduled"))
        if not st["rescheduled"]:
            del st["rescheduled"]
        rp = defaultdict(int)
        for r in self.replans:
            rp[r["status"]] += 1
        legs = {}
        for tag in sorted({x["tag"] for x in self.legs}):
            L = [x for x in self.legs if x["tag"] == tag]
            e0 = np.array([x["measured_s"] - x["planned_s"] for x in L])
            e1 = np.array([x["measured_s"] - x["planned_cal_s"] for x in L])
            legs[tag] = {"n": len(L), "err_vs_model_s": {"median": round(float(np.median(e0)), 2),
                                                         "p95_abs": round(float(np.percentile(abs(e0), 95)), 2)},
                         "err_vs_calibrated_s": {"median": round(float(np.median(e1)), 2),
                                                 "p95_abs": round(float(np.percentile(abs(e1), 95)), 2)}}
        sim_s = self.plant.now()
        wall = time.time() - self.wall0
        return {
            "plant": self.plant.name, "sim_s": round(sim_s, 1), "wall_s": round(wall, 1),
            "realtime_factor": round(sim_s / wall, 2) if wall > 0 else None,
            "requests": len(self.requests), "decisions": dict(st),
            "conformance": self.monitor.summary(),
            "alerts": {"nonconforming": sum(a["event"] == "nonconforming" for a in self.alerts),
                       "landing_overrun": sum(a.get("event_detail") == "landing_overrun" for a in self.alerts),
                       "recovered": sum(a["event"] == "conforming" for a in self.alerts)},
            "replans": {"total": len(self.replans), **dict(rp),
                        "hover_s_mean": round(float(np.mean([r["hover_s"] for r in self.replans])), 1) if self.replans else None},
            "calibration": {"model": asdict(self.calib.vp0), "updates": self.calib.updates,
                            "final": {k: getattr(self.vp_plan, k) for k in ("cruise_xy", "v_xy_max", "v_up_max", "v_down_max")}
                            | {"takeoff_s": self.ip.takeoff_s},
                            "liftoff_lag_s": {"n": len(self.liftoff),
                                              "median": round(float(np.median(list(self.liftoff.values()))), 2)
                                              if self.liftoff else None}},
            "legs": legs,
            "hubs": self.seq.report() if self.seq is not None else None,
            "prediction": {"issued": len(self.predictions),
                           "position_error_m": {f"{h:g}s": {"n": len(v), "median": round(float(np.median(v)), 2),
                                                             "p95": round(float(np.percentile(v, 95)), 2)}
                                                for h, v in sorted(self.pos_err.items()) if v}},
        }


def score_los_predictions(predictions: list, events: list, match_s: float = 15.0) -> dict:
    """Predicted en-route LoS vs actual en-route LoS events (plant ground truth).

    An actual event (a, b, t_start) is predicted with lead L if some prediction issued at t_issue <= t_start
    contains the pair with onset within `match_s` of t_start; L = t_start - earliest such t_issue. A predicted
    event (pair, onset) is a true alarm if it matches an actual event within `match_s`."""
    def key(a, b):
        return (min(a, b), max(a, b))

    actual = [e for e in events if not e["terminal"]]
    leads = []
    for e in actual:
        pair = key(e["a"], e["b"])
        issued = [p["t"] for p in predictions if p["t"] <= e["t_start"] and any(
            key(x["a"], x["b"]) == pair and abs(x["t_onset"] - e["t_start"]) <= match_s for x in p["los"])]
        leads.append(e["t_start"] - min(issued) if issued else None)
    clusters = []
    for p in predictions:
        for x in p["los"]:
            pair = key(x["a"], x["b"])
            for c in clusters:
                if c["pair"] == pair and abs(c["t_onset"] - x["t_onset"]) <= 20.0:
                    break
            else:
                clusters.append({"pair": pair, "t_onset": x["t_onset"]})
    true = sum(any(key(e["a"], e["b"]) == c["pair"] and abs(e["t_start"] - c["t_onset"]) <= match_s for e in actual)
               for c in clusters)
    got = [x for x in leads if x is not None]
    return {"actual_en_route": len(actual), "predicted_events": len(clusters), "true_alarms": true,
            "precision": round(true / len(clusters), 3) if clusters else None,
            "recall_any_lead": round(len(got) / len(actual), 3) if actual else None,
            "recall_lead_10s": round(sum(x >= 10 for x in got) / len(actual), 3) if actual else None,
            "recall_lead_30s": round(sum(x >= 30 for x in got) / len(actual), 3) if actual else None,
            "lead_s_median": round(float(np.median(got)), 1) if got else None}
