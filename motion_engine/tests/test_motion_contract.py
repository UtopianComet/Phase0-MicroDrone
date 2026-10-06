"""MotionStub against the shared contract (INTEGRATION_README.md §3-§5, §8).

These are the Motion Engine deliverables from the integration README:
approach/descent, cancellation, ordinary disarm, emergency-latch interaction,
command expiry and renewal -- all on the simulation clock, no hardware.

Each test drives Motion the way the coordinator will: one `submit()` per
command, exactly one `step(dt, now_ns)` per 0.1 s tick, then `feedback()`.
"""

import json
import math
import sys
from pathlib import Path
from typing import List, Optional

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import pytest

from motion_engine.contracts import (
    SCHEMA_VERSION,
    Command,
    CommandStatus,
    MemoryLogSink,
    MotionCommandType as K,
    Pose,
    Reason,
    SimulationClock,
)
from motion_engine.motion_stubs import MotionStub

DT = 0.1
DT_NS = 100_000_000
#: README §5: command validity 0.2 s, renewed by the coordinator.
VALIDITY_NS = 200_000_000
PAD = Pose(0.0, 0.0, 0.0, 0.0)


class Rig:
    """A minimal stand-in for the coordinator's tick loop."""

    def __init__(self, **motion_kwargs) -> None:
        self.clock = SimulationClock()
        self.log = MemoryLogSink()
        self.motion = MotionStub(logger=self.log, clock=self.clock, **motion_kwargs)
        self._n = 0

    @property
    def now(self) -> int:
        return self.clock.timestamp_ns

    def cmd(self, kind, target: Optional[Pose] = None, *, cid: Optional[str] = None,
            expires: Optional[int] = -1, issued: Optional[int] = None, **kw) -> Command:
        """Build a command issued now. `expires=-1` means now + validity."""
        if cid is None:
            self._n += 1
            cid = f"c{self._n}"
        issued = self.now if issued is None else issued
        if expires == -1:
            expires = issued + VALIDITY_NS
        return Command(
            command_id=cid, kind=kind, mission_state=kw.pop("mission_state", "HOVERING"),
            issued_ns=issued, expires_ns=expires, source=kw.pop("source", "test"),
            reason=kw.pop("reason", "test"), target=target, **kw,
        )

    def send(self, kind, target: Optional[Pose] = None, **kw):
        return self.motion.submit(self.cmd(kind, target, **kw), self.now)

    def tick(self, renew: Optional[Command] = None):
        """One coordinator tick: optional renewal, one step, advance time."""
        ack = None
        if renew is not None:
            ack = self.motion.submit(_renewed(renew, self.now), self.now)
        self.motion.step(DT, now_ns=self.now)
        fb = self.motion.feedback(self.now)
        self.clock.advance(DT)
        return fb, ack

    def run(self, cmd: Command, until, max_ticks: int = 2000) -> List:
        """Renew `cmd` every tick until `until(feedback)` is true."""
        trace = []
        for _ in range(max_ticks):
            fb, ack = self.tick(renew=cmd)
            assert ack.accepted, ack
            trace.append(fb)
            if until(fb):
                return trace
        raise AssertionError(f"condition not met; last feedback={trace[-1]}")

    def launch(self, altitude: float = 5.0) -> None:
        assert self.send(K.ARM, expires=None).accepted
        takeoff = self.cmd(K.TAKEOFF, Pose(0, 0, altitude, 0), expires=None)
        assert self.motion.submit(takeoff, self.now).accepted
        while not self.motion.at_target():
            self.tick()
        while self.motion.pose != self.motion.target:  # settle exactly
            self.tick()


def _renewed(cmd: Command, now: int) -> Command:
    if cmd.expires_ns is None:
        return cmd
    return Command(**{**cmd.__dict__, "expires_ns": now + VALIDITY_NS})


def records(log: MemoryLogSink, event=None):
    return [r for r in log.records if event is None or r.event == event]


# -- acknowledgement vs progress ----------------------------------------------


def test_accepted_means_executing_not_done():
    rig = Rig()
    assert rig.send(K.ARM, expires=None).accepted
    ack = rig.send(K.TAKEOFF, Pose(0, 0, 5.0, 0), expires=None)
    assert ack.accepted and ack.reason == Reason.ACCEPTED
    assert ack.active_command_id == ack.command_id
    fb = rig.motion.feedback(rig.now)
    assert fb.status == CommandStatus.IN_PROGRESS
    assert not fb.arrived and not fb.airborne and fb.armed
    assert fb.active_command_id == ack.command_id and fb.active_kind == "TAKEOFF"


def test_launch_reports_arrival_and_airborne():
    rig = Rig()
    rig.launch(5.0)
    fb = rig.motion.feedback(rig.now)
    assert fb.status == CommandStatus.ARRIVED
    assert fb.airborne and not fb.grounded and fb.armed
    assert fb.pose.z == 5.0 and fb.vertical_error_m == 0.0
    assert not fb.landing_settled


# -- approach / descent / ordinary disarm (README §8) ------------------------------


def test_approach_and_descent_land_settled_then_disarm():
    rig = Rig()
    rig.launch(5.0)
    move = rig.cmd(K.MOVE_TO, Pose(20.0, 10.0, 5.0, 0.0))
    assert rig.motion.submit(move, rig.now).accepted
    rig.run(move, lambda fb: fb.pose == fb.target)

    approach = rig.cmd(K.GOTO_DOCK_APPROACH, Pose(PAD.x, PAD.y, 5.0, 90.0))
    assert rig.motion.submit(approach, rig.now).accepted
    trace = rig.run(approach, lambda fb: fb.pose == fb.target)
    assert trace[-1].status == CommandStatus.ARRIVED
    assert trace[-1].airborne and not trace[-1].landing_settled

    descend = rig.cmd(K.DESCEND_TO_DOCK, Pose(PAD.x, PAD.y, 0.0, 90.0), mission_state="DOCKING_APPROACH")
    assert rig.motion.submit(descend, rig.now).accepted
    trace = rig.run(descend, lambda fb: fb.landing_settled)

    # Arrival tolerance fires while still in the air: that must NOT read as landed.
    early = [fb for fb in trace if fb.arrived and fb.airborne]
    assert early, "expected at least one tick that is 'arrived' but still airborne"
    assert all(not fb.landing_settled and fb.status == CommandStatus.ARRIVED for fb in early)

    landed = trace[-1]
    assert landed.status == CommandStatus.LANDED
    assert landed.grounded and landed.pose.z == 0.0
    assert landed.pose.horizontal_distance_to(PAD) <= 0.25
    assert landed.armed  # ordinary landing does not auto-disarm

    ack = rig.send(K.DISARM, expires=None, mission_state="DOCKED")
    assert ack.accepted
    fb = rig.motion.feedback(rig.now)
    assert not fb.armed and fb.landing_settled and fb.status == CommandStatus.LANDED


def test_disarm_rejected_while_airborne():
    rig = Rig()
    rig.launch(5.0)
    ack = rig.send(K.DISARM, expires=None)
    assert not ack.accepted and ack.reason == Reason.AIRBORNE
    assert rig.motion.armed


def test_flight_commands_rejected_when_not_armed():
    rig = Rig()
    ack = rig.send(K.TAKEOFF, Pose(0, 0, 5, 0), expires=None)
    assert not ack.accepted and ack.reason == Reason.NOT_ARMED
    assert records(rig.log, "takeoff")[-1].decision == "rejected_not_armed"
    rig.tick()
    assert rig.motion.pose == PAD


# -- cancellation / replacement before movement (README §3) -------------------------


def test_new_command_replaces_target_before_next_step():
    rig = Rig()
    rig.launch(5.0)
    far = rig.cmd(K.MOVE_TO, Pose(100.0, 0.0, 5.0, 0.0))
    assert rig.motion.submit(far, rig.now).accepted
    for _ in range(5):
        rig.tick(renew=far)
    here = rig.motion.pose
    assert here.x > 0.0

    hold = rig.cmd(K.HOVER, mission_state="HOVERING", reason="bird_lost")
    ack = rig.motion.submit(hold, rig.now)
    assert ack.accepted and ack.active_command_id == hold.command_id
    fb, _ = rig.tick(renew=hold)
    assert fb.pose == here  # did not take even one more step toward x=100
    assert fb.status == CommandStatus.ARRIVED and fb.active_kind == "HOVER"


def test_rejected_command_leaves_previous_active_and_reports_it():
    rig = Rig()
    rig.launch(5.0)
    move = rig.cmd(K.MOVE_TO, Pose(10.0, 0.0, 5.0, 0.0))
    assert rig.motion.submit(move, rig.now).accepted
    bad = rig.send(K.MOVE_TO, Pose(0.0, 0.0, 500.0, 0.0))
    assert not bad.accepted and bad.reason == Reason.TARGET_OUTSIDE_LIMITS
    assert bad.active_command_id == move.command_id
    assert rig.motion.target.z == 5.0  # not clamped-and-flown


@pytest.mark.parametrize("target", [
    Pose(0.0, 0.0, 31.0, 0.0),     # above the 30 m ceiling
    Pose(200.0, 0.0, 5.0, 0.0),    # outside the 150 m geofence
])
def test_out_of_limit_targets_are_rejected_not_clamped(target):
    rig = Rig()
    rig.launch(5.0)
    ack = rig.send(K.MOVE_TO, target)
    assert not ack.accepted and ack.reason == Reason.TARGET_OUTSIDE_LIMITS
    assert records(rig.log, "move_to")[-1].decision == "rejected_target_outside_limits"


def test_speed_limits_in_command_only_lower():
    rig = Rig()
    rig.launch(5.0)
    slow = rig.cmd(K.MOVE_TO, Pose(10.0, 0.0, 5.0, 0.0), max_horizontal_speed_mps=1.0)
    assert rig.motion.submit(slow, rig.now).accepted
    trace = rig.run(slow, lambda fb: fb.pose == fb.target)
    assert len(trace) == pytest.approx(100, abs=1)  # 10 m at 1 m/s

    fast = rig.cmd(K.MOVE_TO, Pose(0.0, 0.0, 5.0, 0.0), max_horizontal_speed_mps=50.0)
    assert rig.motion.submit(fast, rig.now).accepted
    trace = rig.run(fast, lambda fb: fb.pose == fb.target)
    assert len(trace) == pytest.approx(20, abs=1)  # capped at 5 m/s


# -- expiry and renewal -------------------------------------------------------------


def test_unrenewed_command_expires_into_hold_before_moving():
    rig = Rig()
    rig.launch(5.0)
    move = rig.cmd(K.MOVE_TO, Pose(50.0, 0.0, 5.0, 0.0))
    assert rig.motion.submit(move, rig.now).accepted
    rig.tick()                    # t+0.0: moves
    fb, _ = rig.tick()            # t+0.1: still valid, moves
    assert fb.status == CommandStatus.IN_PROGRESS
    before = rig.motion.pose
    fb, _ = rig.tick()            # t+0.2 == expiry: hold replaces target first
    assert fb.pose == before
    assert fb.status == CommandStatus.EXPIRED_HOLD
    assert fb.target == before and fb.active_kind == "HOVER"
    rec = records(rig.log, "command_expired")[-1]
    assert rec.fields["command_id"] == move.command_id and rec.decision == "hold_in_place"
    for _ in range(10):
        fb, _ = rig.tick()
    assert fb.pose == before  # stays held


def test_renewal_extends_without_restart_or_extra_log():
    rig = Rig()
    rig.launch(5.0)
    track = rig.cmd(K.TRACK_TARGET, Pose(25.0, 0.0, 5.0, 0.0), mission_state="TRACKING")
    assert rig.motion.submit(track, rig.now).accepted
    n_logs = len(rig.log.records)
    fb, ack = rig.tick(renew=track)
    assert ack.accepted and ack.reason == Reason.RENEWED
    trace = rig.run(track, lambda fb: fb.pose == fb.target)
    assert trace[-1].pose.x == 25.0
    # Only the one-off "arrived" record was added; renewals log nothing.
    assert [r.event for r in rig.log.records[n_logs:]] == ["arrived"]


def test_reusing_an_id_for_a_different_request_is_rejected():
    rig = Rig()
    rig.launch(5.0)
    a = rig.cmd(K.MOVE_TO, Pose(10.0, 0.0, 5.0, 0.0), cid="same")
    assert rig.motion.submit(a, rig.now).accepted
    b = rig.cmd(K.MOVE_TO, Pose(-10.0, 0.0, 5.0, 0.0), cid="same")
    ack = rig.motion.submit(b, rig.now)
    assert not ack.accepted and ack.reason == Reason.COMMAND_ID_REUSED
    assert rig.motion.target.x == 10.0
    # An old, no-longer-active id cannot be recycled either.
    assert rig.send(K.HOVER, cid="h1").accepted
    again = rig.send(K.MOVE_TO, Pose(1.0, 0.0, 5.0, 0.0), cid="same")
    assert not again.accepted and again.reason == Reason.COMMAND_ID_REUSED


# -- malformed / stale input ----------------------------------------------------------


@pytest.mark.parametrize("build,reason", [
    (lambda r: r.cmd(K.MOVE_TO, Pose(1, 0, 5, 0), expires=None), Reason.MISSING_EXPIRY),
    (lambda r: r.cmd(K.MOVE_TO, None), Reason.MISSING_TARGET),
    (lambda r: r.cmd(K.MOVE_TO, Pose(float("nan"), 0, 5, 0)), Reason.INVALID_COMMAND),
    (lambda r: r.cmd(K.MOVE_TO, Pose(math.inf, 0, 5, 0)), Reason.INVALID_COMMAND),
    (lambda r: r.cmd(K.MOVE_TO, Pose(1, 0, 5, 0), frame="local_ned"), Reason.UNSUPPORTED_FRAME),
    (lambda r: r.cmd(K.SET_ORIENTATION, Pose(0, 0, 5, 0)), Reason.UNSUPPORTED_KIND),
    (lambda r: r.cmd("FLY_AWAY", Pose(0, 0, 5, 0)), Reason.UNSUPPORTED_KIND),
    (lambda r: r.cmd(K.MOVE_TO, Pose(1, 0, 5, 0), schema_version=2), Reason.UNSUPPORTED_SCHEMA),
    (lambda r: r.cmd(K.MOVE_TO, Pose(1, 0, 5, 0), cid=""), Reason.INVALID_COMMAND),
    (lambda r: r.cmd(K.MOVE_TO, Pose(1, 0, 5, 0), max_vertical_speed_mps=-1.0), Reason.INVALID_COMMAND),
    (lambda r: r.cmd(K.MOVE_TO, Pose(1, 0, 5, 0), issued=True), Reason.INVALID_COMMAND),
    (lambda r: r.cmd(K.MOVE_TO, Pose(1, 0, 5, 0), issued=r.now + DT_NS), Reason.FUTURE_TIMESTAMP),
    (lambda r: r.cmd(K.MOVE_TO, Pose(1, 0, 5, 0), issued=r.now - VALIDITY_NS), Reason.COMMAND_EXPIRED),
])
def test_malformed_or_stale_commands_are_rejected_with_reason(build, reason):
    rig = Rig()
    rig.launch(5.0)
    before = rig.motion.target
    ack = rig.motion.submit(build(rig), rig.now)
    assert not ack.accepted and ack.reason == reason
    assert rig.motion.target == before


def test_non_command_input_does_not_raise():
    rig = Rig()
    ack = rig.motion.submit({"kind": "ARM"}, rig.now)  # a dict, not a Command
    assert not ack.accepted and ack.reason == Reason.INVALID_COMMAND
    assert not rig.motion.armed


def test_kind_may_be_given_as_its_string_value():
    rig = Rig()
    assert rig.send("ARM", expires=None).accepted
    assert rig.motion.armed


# -- emergency latch interaction (README §6-§7) --------------------------------------------


def test_emergency_descent_latches_rejects_ordinary_commands_and_auto_disarms():
    rig = Rig()
    rig.launch(3.0)
    move = rig.cmd(K.MOVE_TO, Pose(20.0, 0.0, 3.0, 0.0))
    assert rig.motion.submit(move, rig.now).accepted
    for _ in range(5):
        rig.tick(renew=move)
    x0 = rig.motion.pose.x

    # An expiry on an emergency command is ignored: it must never time out.
    emerg = rig.cmd(K.EMERGENCY_LAND, mission_state="EMERGENCY_LAND", reason="critical_battery")
    ack = rig.motion.submit(emerg, rig.now)
    assert ack.accepted and ack.reason == Reason.ACCEPTED
    fb = rig.motion.feedback(rig.now)
    assert fb.emergency_descent and fb.status == CommandStatus.EMERGENCY_DESCENT

    for kind, target in [(K.MOVE_TO, Pose(0, 0, 10, 0)), (K.HOVER, None), (K.ARM, None),
                         (K.DISARM, None), (K.LAND, None)]:
        ack = rig.send(kind, target, expires=None if kind in (K.ARM, K.DISARM, K.LAND) else -1)
        assert not ack.accepted and ack.reason == Reason.EMERGENCY_DESCENT_ACTIVE, kind
        assert ack.active_command_id == emerg.command_id

    # Idempotent: a second emergency (new id) is acknowledged, nothing restarts.
    again = rig.send(K.EMERGENCY_LAND, mission_state="EMERGENCY_LAND")
    assert again.accepted and again.reason == Reason.ALREADY_EMERGENCY_LANDING
    assert len(records(rig.log, "emergency_land")) == 1

    ticks = 0
    while rig.motion.armed:
        fb, _ = rig.tick()  # no renewals -- emergency must keep descending
        ticks += 1
        assert fb.pose.x == x0  # in place
        assert ticks < 100
    assert ticks == pytest.approx(30, abs=1)  # 3 m at 1 m/s
    fb = rig.motion.feedback(rig.now)
    assert fb.grounded and not fb.armed and not fb.emergency_descent
    assert fb.landing_settled and fb.status == CommandStatus.LANDED
    assert records(rig.log, "disarm")[-1].decision == "auto_disarm_on_ground"

    # After touchdown Motion is disarmed; flight commands are refused until re-armed.
    ack = rig.send(K.TAKEOFF, Pose(0, 0, 5, 0), expires=None)
    assert not ack.accepted and ack.reason == Reason.NOT_ARMED


# -- feedback shape and determinism ------------------------------------------------------


def test_feedback_is_read_only_and_json_serialisable():
    rig = Rig()
    rig.launch(5.0)
    a = rig.motion.feedback(rig.now)
    b = rig.motion.feedback(rig.now)
    assert a == b
    d = a.to_dict()
    json.dumps(d)
    assert d["schema_version"] == SCHEMA_VERSION and d["synthetic"] is True
    assert d["timestamp_ns"] == rig.now and isinstance(d["timestamp_ns"], int)
    assert set(d) >= {
        "pose", "armed", "airborne", "grounded", "active_command_id", "target",
        "status", "arrived", "landing_settled", "valid", "timestamp_ns",
    }


def _scripted_run():
    rig = Rig()
    rig.launch(5.0)
    script = [
        (rig.cmd(K.TRACK_TARGET, Pose(30.0, 10.0, 5.0, 45.0)), 40),
        (rig.cmd(K.HOVER), 5),
        (rig.cmd(K.GOTO_DOCK_APPROACH, Pose(0.0, 0.0, 5.0, 0.0)), 80),
        (rig.cmd(K.DESCEND_TO_DOCK, Pose(0.0, 0.0, 0.0, 0.0)), 40),
    ]
    trace, acks = [], []
    for cmd, ticks in script:
        cmd = _renewed(cmd, rig.now)
        acks.append(rig.motion.submit(cmd, rig.now).to_dict())
        for _ in range(ticks):
            fb, ack = rig.tick(renew=cmd)
            trace.append(fb.to_dict())
    return trace, acks, [r.to_dict() for r in rig.log.records]


def test_identical_inputs_reproduce_identical_traces_and_logs():
    first = _scripted_run()
    second = _scripted_run()
    assert first == second
    assert first[0][-1]["landing_settled"] is True
