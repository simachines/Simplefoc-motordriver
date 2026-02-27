
<img width="1909" height="1078" alt="sfocdriver" src="https://github.com/user-attachments/assets/d0afb72d-65e4-4a5a-be3b-e08eb2034131" />

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
- Input: E`1..16384` PPR(not CPR)
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
	- Enabled: loop uses `motor.move(target_current_to_amps(target_current))`
	- Disabled: loop uses `motor.move()`

[Simplefoc Commander](https://drive.google.com/file/d/1hYnQ6mngnNLS1kdi2suSRdR4zzsh0aGm/view?usp=sharing)
