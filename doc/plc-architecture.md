# PLC Architecture

This document gives a high-level overview of the TwinCAT 3 PLC code used in the CapCarbone project and how it is structured. 

## 1. System overview

The PLC project controls:

* **4 stepper-driven rails** (`Rail1`…`Rail4`), each represented as a motion axis (`AXIS_REF`) and wrapped by a `FB_StepperMotor` instance.
* **Multiple linear actuators** (chariots, compressors, sensor pivot, heat-exchange and regeneration guillotines) grouped under `ST_LinearActuator` structures and controlled by `FB_LinearActuator` function blocks.
* **Discrete cell position sensors** and drive control signals (speed, stop, fault) mapped to meaningful global variables.
* **A global alarm and safety layer** that aggregates per-device alarms into zone-level status and a global “OK to run” flag.
* **Commands coming from the HMI** (Ignition via OPC UA) are encoded as strings (e.g., `R1_Chariot`, `R3_Motor`, `R2_Home`) and dispatched dynamically to the correct function block through a lookup table and a central `PRG_CommandHandler` program.

## 2. Data types and global structures

### 2.1 Core DUTs

The project defines several key data types in `.TcDUT` files:

* **ENUM_LinearActuatorState**: States including `Extending`, `Extended`, `Retracting`, `Retracted`, `Error`, `AtProx2`, `AtProx3`, `Uninitialised`.
* **ENUM_StepperMotorState**: States for steppers including `Ready`, `Moving`, `Idle`, `Error`, `Homing`, `Hardware_Check`, `PowerError`.
* **ST_LinearActuator**: Contains Name, pointers to proximity sensors (`bProxIn`, `bProxOut`, `bProx2`, `bProx3`), forward/backward DOs, speed/stop DOs, fault DI, and a TON timeout timer.
* **ST_LinearActuatorCommand**: Contains boolean `bCommand` (move request) and `nStatus` (result/status code).
* **ST_StepperMotorCommand**: Contains `bHoming`, `nCommand` (target position), `nPosition` (current/desired), `bHomed`.
* **Alarm-related types**:
    * `AlarmIndex` enumeration (generic alarm indices).
    * `ST_Alarm` (condition, ID, severity, message).
    * `ST_ZoneAlarmStatus` and `ST_SystemAlarmStatus` for aggregating per-zone and global alarm/safety status.

These DUTs provide a uniform way to represent actuators, steppers, and alarm information across the entire project.

### 2.2 Global variable lists (GVLs)

GVLs are used both to map hardware I/O and to instantiate objects:

**I/O mapping**
* `GVL_LinearActuatorProxSensors`: Maps all proximity sensors (`Tri1_Prox_In`, `Cmp1_Prox_Out`, `Gui1_Prox_In_Slow`, etc.) to `%I*` inputs.
* `GVL_LinearActuatorRelays`: Maps forward/backward directions to `%Q*` outputs (`Tri1_Mot_Fw`, `Gui2_Mot_Bw`, etc.).
* `GVL_LinearActuatorDriveControls`: Adds speed, stop, and fault signals for drives.
* `GVL_Steppers`: Maps the four rail axes (`Rail1_Mot_Ref`…`Rail4_Mot_Ref`) and their proximity switches to inputs and associates them with an `ST_LinearTable` structure.

**Actuator instances**
* `GVL_LinearActuators`: Builds 8 linear actuator structures (`Tri1_Mot_Struct`, `Cmp1_Mot_Struct`, `Piv1_Mot_Struct`, `Gui1_Mot_Struct`, etc.) by wiring the appropriate prox/fault/command bits via pointers, then instantiates one `FB_LinearActuator` per structure.
* `GVL_Steppers`: Similarly instantiates `FB_StepperMotor` for `Rail1`…`Rail4` and allocates one `ST_StepperMotorCommand` per axis.

**Positions and process flags**
* `GVL_StepperPositions`: Defines key positions in millimetres (or steps) for each rail (e.g., `Rail1_Pos1 := 0; Rail1_Pos2 := 500; ...`).
* `GVL_Process`: Exposes high-level process flags (`HeatExchangeRunning`, `StartHeatExchange`, `AdsorptionRunning`, etc.), used by process sequences.

**Lookup table & global command context**
The main GVL defines:
* Start/stop/reset inputs (`StartCycle`, `StopCycle`, `ResetSafety`) and memory bits for edge-based logic.
* `gSystemAlarmStatus` and `bCriticalAlarm` for safety gating.
* `sRequestedActuator` : `STRING` – the current command string coming from the HMI.
* `aLookupTable` : `ARRAY[...] OF ST_LookupEntry` mapping `(nX, sY)` pairs (rail index & device type) to pointers to the correct `FB_LinearActuator`/`FB_StepperMotor` and their corresponding command structures.

This lookup table is the core indirection mechanism that lets generic string commands drive any device without hard-coding everything in the command handler.

## 3. Motion control function blocks

### 3.1 Stepper motor control (FB_StepperMotor)

`FB_StepperMotor` encapsulates all motion control for a single rail:

* **Holds pointers to:**
    * `pAxis` : `POINTER TO AXIS_REF` (actual TwinCAT axis).
    * `pProx` : `POINTER TO BOOL` (reference sensor).
* **Wraps standard MC2 function blocks:**
    * `MC_Power`, `MC_Halt`, `MC_MoveAbsolute`, `MC_Home`, `MC_Jog`, `MC_ReadActualPosition`, `MC_Reset`, `MC_ReadAxisError`.
* **Maintains:**
    * `eState` : `ENUM_StepperMotorState` (Idle / Ready / Moving / Homing / Error / PowerError / Hardware_Check).
    * `nRail` (rail index) to address the correct alarm entries.

**Key methods:**

* **KeepAlive:** Powers up the axis with `MC_Power`, sets direction enables, and tracks whether the axis is fully up. It raises a `PowerUp_Failed` alarm and fills the per-rail alarm message with a human-readable description using `F_GetMC2ErrorDescription` on error.
* **Home:** Implements the homing sequence using either:
    * `MC_DefaultHoming` with `bCalibrationCam := pProx^` and `ST_HomingOptions` for normal homing, or
    * `MC_Direct` when `bIsForced` is true (used during hardware checks / reset).
    It raises `Homing_Failed` alarms with informative messages and transitions its state based on `fbHome.Done`/`Error`.
* **MoveToPosition:** Sends an absolute move to `fPosition` with a configurable nominal velocity (`Mot_NormalMotionVel` from GVL). It monitors Busy/Error/Done and sets `Motion_Failed` or `Motion_Interrupted` alarms as appropriate, returning:
    * `1` on success
    * `0` while in progress
    * `-1` on error
* **Jog, Stop, Reset, GetPosition:** Provide jogging, controlled stops, error reset plus automatic hardware check (forced homing) and safe position reads with `UnableToReadPosition` alarms if anything goes wrong.

### 3.2 Linear actuator control (FB_LinearActuator)

`FB_LinearActuator` implements a robust state machine for controlling linear electric motors. Unlike the stepper block, this FB relies on discrete boolean logic, edge detection, and timing rather than MC2 motion commands.

**Dynamic Configuration**
The block automatically categorizes the actuator behavior by inspecting the `pActuator^.sName` string during initialization (`FB_Init`):

* **GUI (Guillotine):** Identified by names containing 'HeatExCoupola' or 'RegenCoupola'. These actuators utilize intermediate sensors (`Prox2`, `Prox3`) and specific speed control outputs to slow down movement at specific points.
* **CMP (Compressor):** Identified by names containing 'Compressor'. These require strict position validation.
* **Basic:** Any actuator not identified as GUI or CMP. These follow standard Extend/Retract logic with end-of-travel sensors only.

**State Machine & Motion Methods**
The block manages `eState` (`ENUM_LinearActuatorState`) through three primary methods:

* **Extend:**
    * Initiates forward movement by setting `bFwdDO` and clearing `bBckwDO`.
    * Starts the `tTimeout` timer to detect mechanical jams.
    * **Intermediate Logic:** For 'GUI' types, it monitors `Prox2` and `Prox3` to trigger speed control outputs (slowing down before the end stop). For 'CMP' types, it checks if the fault prox (`Prox2`) is detected during motion, and triggers an error accordingly.
    * **Completion:** Stops motion upon detecting `bProxOut` (rising or falling edge based on config), updates state to `LAS_LinearActuator_Extended`, and resets the timeout.
    * **Errors:** Triggers a `MotionTimedOut` alarm if the operation exceeds `tTimeout`.

* **Home (Retract):**
    * Initiates backward movement via `SetBackwardMovement`.
    * For 'GUI' types, it monitors intermediate sensors (`Prox2`, `Prox3`) to adjust speed on the return stroke.
    * Stops motion upon detecting `bProxIn` and updates state to `LAS_LinearActuator_Retracted`.

* **Stop:**
    * Immediately clears both forward and backward outputs (unless it is a 'Basic' type, in which case it handles outputs directly).
    * Resets the motion timer to prevent false timeout alarms while idle.

**Position & Error Handling**
* **GetPosition:** Polls hardware inputs against rising/falling edge triggers. It updates the internal `eState` to `Retracted`, `Extended`, `AtProx2`, or `AtProx3`. If inputs are inconsistent (e.g., unknown position), it flags an `UnknownPosition` alarm.
* **CheckDriveFault:** Called cyclically to monitor the physical `bFaultDI` input for all types but 'Basic'. If a drive fault occurs, it transitions `eState` to `Error` and triggers a `DriveFaulted` alarm.

## 4. Alarm and safety management

### 4.1 Alarm definitions and initialization

`GVL_Alarms` defines all per-rail and per-device alarms as `ST_Alarm` instances with unique IDs and severity levels (e.g., `R1_Stepper_Homing_Failed`, `R3_HeatExCoupola_DriveFaulted`, `SafetyRelay`).

`AlarmInit` is a simple program that initializes a generic `g_aAlarms` table with default messages for generic indices in `AlarmIndex`.

### 4.2 Per-alarm processing

`FB_AlarmManager` iterates over all generic alarms (`AlarmIndex`) and runs an `AlarmHandler` on each entry every cycle.

### 4.3 System-level status

`FB_AlarmStatusManager` aggregates all defined alarms into an `ST_SystemAlarmStatus` structure:

* On first run, it calls `InitializeAlarmList`, which registers every `ST_Alarm` pointer with a human-readable name (e.g., `R1_Sensor_UnknownPosition`).
* Each cycle, it:
    1.  Resets all zones (`bOkToRun := TRUE`, `nHighestSeverity := 0`).
    2.  Updates the safety relay alarm (`SafetyRelay.bCondition := SafetyRelayStatus`).
    3.  Loops through all registered alarms, assigns them to a zone via `ParseAlarmZone(sName)` and calls `ProcessAlarm` to update zone severities and global `bCriticalAlarm`.

`gSystemAlarmStatus` and `bCriticalAlarm` are then used by the command handler to block motions when the system is not safe.

## 5. Command and sequencing layer

### 5.1 Command encoding

Commands are encoded as strings:

**From Cycle → PLC:**
High-level function `F_IssueActuatorCommand(sDevice, bArg, bHasPosition, nPosition)` is exposed.
* Examples:
    * `F_IssueActuatorCommand('Chariot1', TRUE)` → move Chariot1 forward.
    * `F_IssueActuatorCommand('Motor2', TRUE)` → home Rail 2.
    * `F_IssueActuatorCommand('Motor1', FALSE, TRUE, Rail1_Pos3)` → move Rail 1 to a preset position.

**Inside the function:**
1.  It parses the last character of `sDevice` as an index (1–4) and the remaining prefix (Chariot, Compressor, Sensor, HeatExCoupola, RegenCoupola, Motor).
2.  It builds and writes `sRequestedActuator` in the form `R<n>_<Prefix|Home|Motor>` (e.g., `R1_Chariot`, `R3_Home`).
3.  For non-motor devices, it sets the appropriate `ST_LinearActuatorCommand.bCommand` bit.
4.  For motors, if `bArg = TRUE` it sets `bHoming := TRUE`; otherwise it writes `nCommand := nPosition` in the right `ST_StepperMotorCommand`.
5.  It returns the global `Result` code indicating execution state.

### 5.2 Validating commands and resolving targets

The command handler uses a separate validator and lookup functions:

* `F_ValidateCommandAndReturn` checks that incoming strings:
    * Start with `R`.
    * Contain a valid index `1..4` between `R` and `_`.
    * Use a supported suffix (Chariot, Compressor, Sensor, HeatExCoupola, Home, Motor).
* `F_GetLinearCommandValue` and `F_GetStepperCommandMember` take `(nX, sY)` and return pointers to the relevant `FB_LinearActuator` and `ST_LinearActuatorCommand` or `ST_StepperMotorCommand` by searching `aLookupTable`.

This lets a single handler program route any valid command to the right object.

### 5.3 PRG_CommandHandler

`PRG_CommandHandler` is the central dispatcher that ties commands request from `F_IssueActuatorCommand()` and the HMI:

* **Safety gate:** Calls `EvaluateSystemSafety()` and refuses to start new motions if the system is unsafe and the command is not a reset-type command. Also checks global `bCriticalAlarm`.
* **HMI handshake:** Uses `bAwaitingHMIClose` and `tmrHMICloseSignal` to hold the “close” command state for a short time, then resets `sRequestedActuator := 'none'` so the HMI can close the popup.
* **Cancel logic:** Monitors `CancelMove` and, when needed, sets `bCancelInProgress` and calls `ProcessCancelOperation()` until the cancellation is complete or times out.
* **Processing new commands:**
    1.  Validates `sRequestedActuator` with `F_ValidateCommand(...)`.
    2.  Determines if the target is a linear actuator or a stepper (`bisLinearActuator`, `bisLinearTableHomeCmd`, `bisLinearTableMoveCmd`, etc.).
    3.  Looks up pointers to the correct FB and command structure.
    4.  Sets `bOperationInProgress` and drives the chosen FB’s methods (Home, MoveToPosition, etc.) until they return success or failure.

This program is designed so that only one operation is in progress at a time, with explicit cancellation and proper cleanup of `sRequestedActuator`.

### 5.4 Process-level sequencing (PRG_Cycle)

`PRG_Cycle` is a higher-level program that manages the overall process state machine (heat exchange, adsorption, regeneration) using a `Status` integer to flag motion status and booleans from `GVL` to conditionaly control execution based on the state of machine security mechanism. 

## 6. Typical flow from HMI to motion

A typical end-to-end interaction looks like this:

1.  **HMI:** Operator chooses a device and action (e.g., “Move Rail 3 to position 600”).
2.  **HMI Script:** Calls `sRequestedActuator` and `nCommand` via OPC UA and monitors the returned position feedback.
3.  **PLC:** `PRG_CommandHandler` validates `R3_Motor`, resolves it to `Rail3_Mot_FB` / `Rail3_Mot_Cmd` using the lookup table, checks safety, then calls `MoveToPosition(600)` on `FB_StepperMotor`.
4.  **PLC:** `FB_StepperMotor` executes the move using MC2 blocks, updates alarms on error and returns a status code.
5.  **HMI feedback:** Ignition reads back the axis position and any alarm messages (via OPC UA) to show success, error or timeout to the user.