# One channel's self test record, exactly as the board reports it.
#
# RAM only: a power cycle clears these (BSP/robot.h:83-84), so an all-zero record
# means "nothing probed since boot", not "everything passed".
#
# "executed" is false when the record's sequence does not match this batch, i.e. the
# channel was never energised by THIS command -- which is what the firmware leaves
# behind for channels it dropped from the queue (BSP/robot.c:102-106 sets them to
# state 0 UNTESTED with a reason). Never read an unexecuted channel as a pass.
uint8 channel            # protocol channel 0..3
bool executed            # this record belongs to the batch that was just requested
uint16 sequence          # command sequence stamped into the record
uint8 state              # 0 untested, 1 queued, 2 running, 3 settling,
                         # 4 passed, 5 failed, 6 cancelled, 7 skipped
uint8 reason             # 0..10, see Robot docs; 10 is legacy and never produced
uint8 phase              # 0..5 round trip phase
int32 outbound_pulses
int32 return_pulses
int32 residual_pulses    # signed; diagnostic only, the firmware does not correct it
uint16 powered_ms        # saturates at 65535
uint16 return_ms
bool position_valid      # valid bit read back for this channel
uint8 fault              # fault code read back for this channel; a FAILED probe
                         # sets fault 7 and is NOT cleared automatically
