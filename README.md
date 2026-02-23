<img width="2743" height="1669" alt="simplefoc_driver" src="" />

## Commander Commands

Serial Commander is enabled on `Serial` at `921600` baud.

### `M` - Motor command passthrough
- Handler: `commander.motor(&motor, cmd)`
- Purpose: direct access to SimpleFOC motor Commander interface.
- Example:
	- `M...` (use standard SimpleFOC motor subcommands)

### `B` - Set current-loop bandwidth
- Handler: `setBandwidth(char* cmd)`
- Input: bandwidth in Hz, valid range `0 < B <= 1000`
- Effects:
	- Updates `current_bandwidth`
	- Recomputes `motor.PID_current_d/q.P`, `motor.PID_current_d/q.I`, `motor.LPF_current_d/q.Tf`
- Example:
	- `B150` sets current control bandwidth to `150 Hz`

### `E` - Set MT6835 ABZ resolution
- Handler: `onSetABZResolution(char* cmd)`
- Input: ABZ PPR in range `1..16384`
- Mapping:
	- `PPR=1` -> raw `0x0000`
	- `PPR=16384` -> raw `0x3FFF`
	- Internally: `raw = ppr - 1`
- Behavior:
	- Writes with `encoder2.setABZResolution(raw)`
	- Reads back with `encoder2.getABZResolution()` and prints confirmation
- Example:
	- `E4096`

### `C` - PWM input control enable/disable
- Handler: `onPWMInputControl(char* cmd)`
- Input:
	- `C1` enable PWM-input-driven current command
	- `C0` disable PWM-input-driven current command
	- `C`  query current state
- Runtime effect:
	- Enabled: loop uses `motor.move(target_current_to_amps(-target_current))`
	- Disabled: loop uses `motor.move()`
