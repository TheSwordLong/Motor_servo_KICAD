# Whole-board snapshot published on /robot/motor_states_deg.
# Every field is either read from the board or derived from a read; nothing is
# extrapolated. "stale" is set when the last successful poll is older than the
# configured freshness limit, and the header stamp is the ROS time of the poll,
# never a rewritten old measurement.
std_msgs/Header header

string uid                  # 24 uppercase hex chars; empty until identity is confirmed
uint8 command_protocol      # board register 0x000F; 2 is required for direct commands
bool identity_ok            # UID matched the configured board AND protocol == 2
bool communication_ok       # last transaction completed with a valid CRC frame
bool stale                  # snapshot age exceeded the freshness limit
float64 age_sec             # seconds since the snapshot was taken (monotonic clock)

uint16 last_sequence        # register 0x0036
uint16 last_result          # register 0x0037, business result 0..6
bool dirty                  # register 0x0005
bool storage_error          # register 0x0006
bool pending_save           # register 0x0007

bool read_only              # driver is refusing every write, including STOP
bool unknown_command_state  # a command outcome could not be reconciled; motion scheduling halted
string unknown_detail       # human readable reason for unknown_command_state

# Motion-protection posture, published so a consumer can SEE that soft limits are not
# in play instead of having to infer it from the configuration file.
bool commissioning          # debug mode: motion allowed although limits are unconfirmed
bool soft_limits_enforced   # true only when EVERY mapped joint has limits_rad set

MotorStateDeg[] channels
