# One channel snapshot, straight from the board registers. No derived values are
# invented here: angle/fg/target are the raw int32 register contents, and the
# driver additionally publishes the converted degree value.
uint8 channel             # protocol channel 0..3
uint8 role                # firmware role id 0..7
bool enabled
uint16 gear_ratio
uint8 inverted
uint16 configured_rpm     # applied cruise rpm (pre-gear)

uint8 state               # 0 UNREADY, 1 IDLE, 2 RUNNING, 3 DECEL,
                          # 4 SETTLING, 5 DONE, 6 FAULT, 7 SELFTEST
uint8 fault               # 0 none, 1 feedback, 2 timeout, 3 capture,
                          # 4 unexpected fg, 5 position, 6 storage, 7 selftest
bool position_valid       # firmware "valid" bit; false means no trusted zero
bool brake
bool direction            # raw DIR output level
uint16 duty_permille      # LOW-level per mille, firmware ceiling 950; not a percent
uint16 measured_rpm       # unsigned magnitude only; sign is NOT measurable from one FG wire

int32 angle_deci_deg      # 0.1 deg on the output shaft (raw register)
int32 fg_count            # signed motor-side pulse count (raw register)
int32 target_deci_deg     # 0.1 deg target of the last accepted command

float64 angle_deg         # angle_deci_deg converted; only meaningful when position_valid
float64 target_deg
uint32 pulses_per_output_rev   # gear_ratio * 6, 0 when gear is unknown

uint8 selftest_state      # 0..7, RAM only, lost on power cycle
uint8 selftest_reason     # 0..10
uint8 selftest_phase      # 0..5
int32 selftest_outbound_pulses
int32 selftest_return_pulses
int32 selftest_residual_pulses
uint16 selftest_powered_ms
uint16 selftest_return_ms
uint16 selftest_sequence
