# Laguna Seca Multicar Rules of Engagement
The goal is to exercise multi-car passing where every vehicle runs active cruise control and follows negotiated speed profiles without base control or race control input.

## Summary of the Rules
- Passing is allowed only on certified straights that provide enough lateral clearance for both cars to hold their lanes.
- The attacker broadcasts `PASS_STATE_REQUESTING` in `pass_state` before entering a pass zone, including its own ID, the defender ID, and the requested zone ID.
- The defender must reply with `PASS_STATE_ACKNOWLEDGED` in `pass_state` before the maneuver starts; until then both cars remain in formation and the attacker uses cruise control to manage spacing.
- After acknowledgment, the attacker and defender may stage for the pass: maintain nominal speed; lane positioning isn’t required until the straight. At the entry to the authorized passing zone, the defender drops to the negotiated yield speed and the attacker moves to the designated passing lane.
- Outside authorized zones, vehicles may change lanes to set up the maneuver but must maintain nominal speed and spacing until they enter a passing zone or receive a race control directive.
- Both cars emit `PASS_STATE_COMPLETED` once the attacker is safely ahead so nominal formation rules resume.
- **If any vehicle broadcasts `STATE_EMERGENCY_STOP`, every car within radio reach must come to an immediate stop until that state is clear.**

## AVLT coordination message
```
std_msgs/Header header        # Standard ROS header (stamp drives relative timing)

uint8   car_id                # Vehicle ID [ - ]
int32   lat_e7                # Latitude * 1e7, rear-axle centre [ signed deg * 1e7 ]
int32   lon_e7                # Longitude * 1e7, rear-axle centre [ signed deg * 1e7 ]
int16   alt_dm                # Altitude (ellipsoid) in decimetres [ dm ]
int16   heading_cdeg          # Heading in centi-degrees, North = 0 [ deg * 100 ]
int16   vel_cms               # Longitudinal speed in centimetres per second [ cm/s ]
uint8   state                 # Vehicle state enum below

# Vehicle state constants
uint8 STATE_UNKNOWN = 0
uint8 STATE_EMERGENCY_STOP = 1
uint8 STATE_CONTROLLED_STOP = 2
uint8 STATE_NOMINAL = 3

uint8   pass_state            # Engagement finite-state machine value [ enum below ]
uint8   pass_sequence         # Monotonic counter to correlate handshakes
uint8   target_car_id         # Defender car ID being overtaken or followed [ - ]
uint8   pass_zone_id          # Identifier for the authorized straight where the pass occurs
uint8   heartbeat             # Rolling heartbeat counter; drop rate exposes link quality [ - ]
uint16  yield_speed_cms       # Defender follow speed for yielding car [ cm/s ]
uint16  request_ttl_ms        # Request time-to-live relative to header.stamp [ ms ]

# Pass state constants
uint8 PASS_STATE_IDLE = 0
uint8 PASS_STATE_REQUESTING = 1
uint8 PASS_STATE_ACKNOWLEDGED = 2
uint8 PASS_STATE_TARGET_READY = 3
uint8 PASS_STATE_EXECUTING = 4
uint8 PASS_STATE_COMPLETED = 5
uint8 PASS_STATE_ABORTED = 6
uint8 PASS_STATE_SUSPENDED = 7
```

### Field guidance
- `lat_e7`/`lon_e7` retain ~1 cm resolution worldwide while using 32-bit integers, and `heading_cdeg`/`vel_cms` deliver 0.01° and 0.01 m/s resolution with 16-bit storage.
- `pass_sequence` increments whenever a fresh pass is requested so acknowledgements and completions match even if packets drop.
- `pass_zone_id` refers to a track configuration table that defines legal straights, lane boundaries, speed profiles, clearance envelopes, and abort plans without changing message semantics.
- `yield_speed_cms` stores the negotiated follow speed with centimetre-per-second resolution so both controllers hold the same target once yield mode begins.
- `request_ttl_ms` is applied against `header.stamp`; receivers compute `deadline = header.stamp + request_ttl_ms` and revert to `PASS_STATE_IDLE` after that time.
- `pass_state` publishes the FSM value using the `PASS_STATE_*` constants, and lane metadata keeps planners from drifting back to the racing line during an abort while confining pass-driven speed changes to the zone interior and defining target lines for defender and attacker.
- `heartbeat` taps the existing AVLT heartbeat so vehicles detect peers falling outside range, quantify link quality from drop rate.

### Autonomous multi-car state machine
Each vehicle runs the same finite-state machine keyed by `pass_state`. The attacker is the car that issued the current request, and the defender is the `target_car_id`. The FSM governs overtaking, yielding, and formation behaviour without manual input and scales to multiple competitors through zone reservations and queued requests.

#### State semantics
- `PASS_STATE_IDLE`: No active request; `target_car_id` is zero or self, and the car maintains nominal race pace subject to global safety rules.
- `PASS_STATE_REQUESTING`: Attacker reserved the next eligible passing slot for the specified defender and zone and is awaiting acknowledgement.
- `PASS_STATE_ACKNOWLEDGED`: Request and acknowledgement matched on `pass_sequence`; both cars maintain nominal speed until crossing the zone entry. Lane staging can occur beforehand, but the defender completes the yield-speed change and locks into the zone's defender line inside the zone. The zone is locked to this pair.
- `PASS_STATE_EXECUTING`: Both cars are inside the zone executing the pass; any remaining lane alignment and all speed adjustments occur within the clearance envelope while the defender holds the safe line and negotiated speed.
- `PASS_STATE_COMPLETED`: Attacker achieved the required gap, both cars broadcast `PASS_STATE_COMPLETED`, and the formation returns to nominal after the cool-down timer.
- `PASS_STATE_ABORTED`: A fault (timeout, hazard, or conflicting reservation) forced the pair onto the abort speed profile while each car stays in its current lane; if the abort occurs before zone entry they maintain nominal speed in that staging alignment, otherwise the deceleration happens inside the active zone until race control releases them. `pass_sequence` increments before the next request.
- `PASS_STATE_SUSPENDED`: Communications dropped below the heartbeat threshold but the TTL window has not expired; both cars hold formation while waiting for reconnection. Active cruise control maintains minimum headway in all pre-pass phases.

#### Attacker transitions
- Idle → Requesting: Requires a faster attacker, an eligible pass zone within horizon, no conflicting lock, and attacker `STATE_NOMINAL`. The attacker fills `target_car_id`, `pass_zone_id`, `yield_speed_cms`, `request_ttl_ms`, increments `pass_sequence`, then publishes `PASS_STATE_REQUESTING`.
- Requesting → Idle: TTL expires or the defender declines by staying in `PASS_STATE_IDLE`. The attacker clears defender metadata and observes a cool-down before reissuing.
- Requesting → Acknowledged: Matching `PASS_STATE_ACKNOWLEDGED` arrives with the same `pass_sequence`, defender ID, and zone ID. The attacker reserves the zone, synchronises approach speed, and republishes state.
- Requesting → Aborted: A hazard is detected before acknowledgement. The attacker broadcasts `PASS_STATE_ABORTED`, keeping its current staging lane at nominal speed when outside the zone or following the abort speed profile once inside, and relays the alert through AVLT.
- Acknowledged → Executing:  The attacker must received defender ready flag before swithing to this state. The attacker crosses the zone entry with the defender still at nominal speed; any remaining lane alignment and the yield-speed change occur inside the zone as they assert `PASS_STATE_EXECUTING`.
- Executing → Completed: Track-specific clear-ahead criteria are satisfied before the zone exit. The attacker broadcasts `PASS_STATE_COMPLETED` and returns to formation logic.
- Executing → Aborted: Clearance violation, defender state downgrade, emergency stop, or heartbeat timeout occurs inside the zone. Both cars follow the abort profile while holding their assigned lanes.
- Completed → Idle: After a short stabilisation interval (about 1–2 s) and restored spacing, attacker returns to idle and clears defender metadata.

#### Defender transitions
- Idle → Acknowledged: A valid `PASS_STATE_REQUESTING` arrives, the defender is free, the zone matches, its vehicle state is `STATE_NOMINAL`, heartbeat from the attacker meets the configured margin, and no higher-priority race control constraint exists. The defender is not tracking another vehicle ahead. The defender publishes `PASS_STATE_ACKNOWLEDGED` and enters that state.
- Acknowledged → Prepping: This is archived by the defender vehicle being in their lane and on the passing straight, this ensure the vehicles don't collide on the same lane. Defender then relays `PASS_STATE_TARGET_READY`. The defender locks into the zone's defender line and reduces to `yield_speed_cms`
- Prepping → Executing: The defender speed the defender's line and awaits for the the attcking vehicle.
- Acknowledged → Suspended: The attacker drops out of radio contact (heartbeat timeout) before entry. The defender holds its current lane, maintains nominal speed, and periodically rebroadcasts metadata until expiry, deferring any speed change until zone entry.
- Executing → Completed: The defender observes `PASS_STATE_COMPLETED`, verifies safe trailing gap, and accelerates back to race pace before returning to idle.
- Any → Aborted: Sensor hazards, conflicting reservations, or race-control downgrades force an abort. The defender broadcasts `PASS_STATE_ABORTED`, keeping its current lane at nominal speed when outside the zone or tracking the abort profile in-zone, and waits for clearance to resume racing.
- Suspended → Idle: TTL expires or heartbeat remains absent beyond the configured window; the defender releases reservations and reverts to formation mode.

#### Multi-vehicle constraints
- Zone reservation: Only one engagement per `pass_zone_id` at a time; the first pair to reach `PASS_STATE_ACKNOWLEDGED` holds the lock.
- Queueing: Each vehicle queues incoming requests by time-to-zone and sequence number; only the queue head may grant acknowledgement.
- Mutual exclusion: A vehicle cannot attack and defend in overlapping zones. If already defending while executing a pass, it declines new requests to avoid cascading manoeuvres.
- Minimum spacing: Before issuing a request the attacker ensures any third vehicle between attacker and defender is at least one car length outside the zone to avoid three-wide conflicts.
- Zone certification: Race control distributes pass-zone metadata tagged with supported vehicle combinations; only zones with adequate clearance may be requested.
- Zone discipline: Pass engagements restrict yield-speed profiles to the configured zone boundaries; lane changes are allowed outside the zone provided they respect spacing, signalling, and track rules.
- Heartbeat-aware queuing: If a car stops transmitting heartbeat messages, pending requests referencing it stay on hold until heartbeats resume. Requests expiring before then must be resubmitted.
- Global abort propagation: Any `PASS_STATE_ABORTED` announcement for a zone forces all vehicles advertising that zone to drop to `PASS_STATE_IDLE` and re-evaluate.
- Abort lane discipline: Pass-zone configurations define the abort profile and lane assignments; all nearby vehicles hold their current lanes (attacker in the passing lane, defender in the defender lane) until the zone clears.
- Cool-down enforcement: After a completion or abort both participants stay in `PASS_STATE_IDLE` for the shared cool-down window so trailing vehicles get a deterministic opportunity to request the next pass.

### Emergency stop coordination
- Emergency stops trigger only on a transition into `STATE_EMERGENCY_STOP`; each listener latches the initiating `car_id` and message `header.stamp` as the stop event.
- Vehicles remain stopped until they receive a newer message from that initiator reporting a non-emergency state, or an explicit clear directive.
- Messages that repeat the same stop event after the clear are ignored so delayed packets do not cause phantom stops.

### Abort handling
- Every `pass_zone_id` carries a fail-safe abort profile and lane assignments so standard lane-keeping suffices.
- When `PASS_STATE_ABORTED` is announced, both cars brake toward the abort target within 100 ms while holding their lanes if they are already inside the active pass zone; otherwise they maintain their current lanes at nominal speed until race control issues further instructions.
- Trailing cars entering the zone after an abort message maintain their lanes and match the slowest vehicle ahead, blocking new passes until the zone clears.
- Participants exchange `PASS_STATE_IDLE` messages with a re-entry flag once telemetry stabilises, then accelerate back to race pace while maintaining lane discipline.
- If connectivity stays degraded, cars continue announcing `PASS_STATE_ABORTED` at least 5 Hz so observers know the zone remains restricted.

#### Autonomy guarantees
- Transitions rely only on telemetry, planner outputs, and the AVLT coordination message; no manual operator input is needed once the race starts.
- Race control overrides (`STATE_CONTROLLED_STOP` or `STATE_EMERGENCY_STOP`) force an immediate move to `PASS_STATE_ABORTED`.
- Connectivity-aware policies ensure cars falling outside the ~600 m V2V envelope pause the manoeuvre in `PASS_STATE_SUSPENDED` or abort if reconnection misses the timeout, preventing blind passes.
- Formal verification should confirm every path completes or aborts with a deterministic resolution so more than two cars cannot livelock in the same zone.
