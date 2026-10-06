# Motion Engine — Phase 0

[Phase 0 scope: Start Here](../START_HERE.md) · [Shared integration contract](../INTEGRATION_README.md)

## Current milestone: the synthetic loop

For the integration milestone there is no drone and no PX4 in the loop.
Motion's job is the kinematic stub, [`motion_stubs.py`](motion_stubs.py),
driven by the central coordinator through the contract in
[INTEGRATION_README.md](../INTEGRATION_README.md) §3-§5. Motion owns the
synthetic vehicle pose and its execution status, and never a mission state.

### Contract (`motion_engine/contracts.py`)

| Type | Direction | What it carries |
|---|---|---|
| `Command` | coordinator → Motion | `command_id`, `kind`, `mission_state`, `target` (`Pose`, local ENU), `frame`, optional speed caps, `issued_ns`, `expires_ns`, `source`, `reason` |
| `CommandAck` | Motion → coordinator, immediately | `command_id`, `accepted`, stable `reason`, `active_command_id` (what is still flying after this) |
| `VehicleFeedback` | Motion → snapshot `vehicle` section | pose, `armed`, `airborne`, `grounded`, `emergency_descent`, `landing_settled`, active command ID/kind/target, `status`, `arrived`, errors, `timestamp_ns`, `valid` |

One coordinator tick:

```python
from motion_engine.contracts import Command, MotionCommandType as K, Pose, SimulationClock
from motion_engine.motion_stubs import MotionStub

clock = SimulationClock()                 # integer ns, starts at 0
motion = MotionStub(logger=shared_logger, clock=clock)

ack = motion.submit(command, clock.timestamp_ns)   # step 8: dispatch, record ack
motion.step(0.1, now_ns=clock.timestamp_ns)        # advance Motion exactly once
vehicle = motion.feedback(clock.timestamp_ns)      # post-motion feedback
clock.advance(0.1)
```

Rules Motion enforces:

- **Accepted is not done.** `status` goes `in_progress` → `arrived`, and for
  landing commands → `landed`. `arrived` uses the 0.25 m arrival tolerance and
  can be true while still airborne, so docking must check `landing_settled`
  (z exactly 0 after a landing command), never `arrived`.
- **Expiry.** Every command except arm/disarm/takeoff/land/emergency needs
  `expires_ns`. Re-sending the active `command_id` with a later expiry renews
  it: no restart, no new log line. If it expires, Motion replaces it with a
  hold at the current pose *before* moving that tick (`status = expired_hold`).
- **Replacement.** An accepted command replaces the active target
  immediately, so a hold sent this tick stops the drone this tick.
- **No silent clamps.** A target above the ceiling or outside the geofence is
  rejected (`target_outside_limits`), not clamped. The direct methods still
  clamp, for scripts.
- **Emergency latch.** After `EMERGENCY_LAND` every other command is rejected
  (`emergency_descent_active`), repeats are idempotent, emergency never
  expires, and touchdown auto-disarms.
- **Bad input is a result, not an exception.** Missing target or expiry, NaN
  or inf, wrong frame or schema, future timestamps and reused IDs each come
  back as a rejected ack with a stable reason (`contracts.Reason`).

Limits default to the INTEGRATION_README §5 table (`MotionLimits`): 5 / 2 m/s,
90 deg/s, 150 m geofence, 30 m ceiling, 0.25 m / 3 deg tolerances, 1 m/s
emergency descent. Pass the same `MotionLimits` that Safety uses.

Logging goes through any object with the agreed
`log(module, event, *, fields, state, truth, decision, t)` method. The shared
logger is planned for `state_machine/logging_format.py`; until it exists,
`contracts.MemoryLogSink` is an in-memory stand-in. Motion no longer imports
the old `mission.logging_format`.

### Runnable checks

```bash
py -3 -m pytest motion_engine/tests/test_motion_stubs.py motion_engine/tests/test_motion_contract.py
py -3 motion_engine/synthetic/generate_motion_data.py      # synthetic CSVs, see synthetic/README.md
```

`test_motion_contract.py` covers the README's Motion deliverables: approach
and descent, cancellation, ordinary disarm, emergency-latch interaction,
expiry and renewal, rejected input, and identical traces from identical inputs.

## Phase-1 (PX4) goals

The original PX4 plan. It comes after the synthetic milestone above.

### Purpose

Translate permitted high-level movement targets into PX4 external-control commands. PX4 supplies low-level flight stabilization in simulation. Building a replacement flight PID or directly driving motors is not required for this semester.

### Inputs and outputs

- Inputs: navigation/docking targets, estimated vehicle state, safety decisions.
- Outputs: bounded PX4 position or velocity targets and yaw commands; command/status logs.
- Define units, coordinate frames, command expiration, and external-control lifecycle before connecting modules.
- Log requested and sent targets separately from observed vehicle motion.

### First tasks

1. Establish a PX4 connection and read status/telemetry.
2. Reproduce takeoff, hold, and landing in the stock simulation.
3. Define a bounded command interface with navigation and safety.
4. Handle rejected commands, stale requests, and loss of the control connection.
5. Execute waypoint and docking targets through the same interface.

Maintain the command stream required by the selected PX4 interface and verify its loss-of-control behavior. Start from the [PX4 offboard example](https://docs.px4.io/main/en/ros2/offboard_control), using documentation matching the pinned release.

### Acceptance evidence

A repeatable flight script takes off, holds, and lands while recording targets and telemetry. Tests demonstrate bounds enforcement, command expiration, and the chosen abort behavior.

A controlled mission abort and motor termination are distinct actions. Do not implement one ambiguous “kill switch” for both.

The PX4 side (offboard connection, SITL flight script) is not part of the
synthetic milestone. Each `MotionStub` method has a `# PHASE-1 HOOK:` comment
naming the PX4 command it becomes; `grep -n "PHASE-1 HOOK" motion_engine/motion_stubs.py`
is the to-do list.
