# Laguna Seca Multicar Rules of Engagement
The goal is to exercise multi-car passing where every vehicle runs active cruise control and follows negotiated speed profiles without race control intervention.

## Summary of the Rules
- Passing is allowed only on certified straights that provide enough lateral clearance for both cars to hold their lanes.
- Race control will coordinate passes between two vehicles: an attacker and a defender.
- **If any vehicle broadcasts `STATE_EMERGENCY_STOP`, every car within radio reach must come to an immediate stop until that state is clear.**[^race-control]

## AVLT Position message
```Python
std_msgs/Header header        # Standard ROS header (stamp drives relative timing)
uint8   car_id                # Car ID [ - ]
uint8   heartbeat             # Rolling heartbeat counter; drop rate exposes link quality [ - ]

float64 lat             # Vehicle longitude, centre of rear axle [ dd.dd ]
float64 lon             # Vehicle latitude, centre of rear axle [ dd.dd ]
float32 alt             # Vehicle altitude (ellipsoid), centre of rear axle [ m ]
float32 heading         # Vehicle heading, GPS style, North = 0, East = 90 [ deg ]
float32 vel             # Vehicle speed, Vx_body [ m/s ]
uint8   state           # Vehicle state, see iac_udp_struct.h [ - ]

# Vehicle state constants
uint8 STATE_UNKNOWN = 0
uint8 STATE_EMERGENCY_STOP = 1
uint8 STATE_CONTROLLED_STOP = 2
uint8 STATE_NOMINAL = 3
```

### Emergency stop coordination
- Emergency stops trigger only on a transition into `STATE_EMERGENCY_STOP`; each listener latches the initiating `car_id` and message `header.stamp` as the stop event.
- Vehicles remain stopped until they receive a newer message from that initiator reporting a non-emergency state, or an explicit clear directive.
- Messages that repeat the same stop event after the clear are ignored so delayed packets do not cause phantom stops.

#### Autonomy guarantees
- Race control overrides (`STATE_CONTROLLED_STOP`[track red or vehicle red flag] or `STATE_EMERGENCY_STOP`[purple flag]) force an immediate move to `PASS_STATE_ABORTED`.

[^race-control]: The safe-pass flow assumes an active human race control monitoring the event. Race control must be prepared to halt the field if the transponder system fails and to red-stop trailing cars when a leading transpondered vehicle leaves the track or stops unexpectedly. Teams should evaluate additional edge cases and recognise the system's limitations; they may run their own perception stacks, but cannot assume that other entrants do so. Participation requires a working ACC that respects the transponder-provided distances.
