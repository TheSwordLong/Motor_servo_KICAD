# One channel's configuration as the board stores it.
# channel is the PROTOCOL channel 0..3 (YAML joint files use 1..4 and are
# converted at the configuration boundary only).
uint8 channel

# Effective (applied) values read from base+0..4.
bool enabled
uint8 role            # 0 unassigned, 1 head, 2 left_upper, 3 right_upper,
                      # 4 left_forearm, 5 right_forearm, 6 left_palm, 7 right_palm
uint16 gear_ratio     # firmware accepts only 0, 82, 290 or 516
uint8 inverted        # 0 or 1; MOVE derives direction as (arg < 0) XOR inverted
uint16 rpm            # pre-gear cruise rpm, firmware accepts 400..8000
