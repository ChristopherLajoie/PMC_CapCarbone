# CapCarbone – PLC Architecture & Motion Control

This document describes the internal structure of the TwinCAT 3 PLC code used in the CapCarbone project: how motion, actuators, commands and alarms are organized.

> **Scope**  
> - Focus on PLC code inside the `CapCarbone` TwinCAT project  
> - Complements the high-level repository README and HMI documentation

---

## 1. System Overview

CapCarbone controls a modular adsorption/regeneration system composed of:

- **4 linear rails with stepper motors** (Rail 1–4)
- **Multiple linear actuators** (chariots, compressors, sensors, heat exchangers, regeneration coupolas, motors)
- **Ignition HMI** that sends high-level commands and displays status/alarms
- **Beckhoff EL7062** terminals for stepper control and digital I/O for actuators

Core ideas:

- Each **rail** is a `FB_StepperMotor` instance (one axis per rail).
- Each **linear actuator** is a `FB_LinearActuator` instance.
- The HMI sends simple **string commands** (e.g. `"Chariot1"`, `"Motor2"`), which are resolved to the correct function block and action.
- A central **alarm model** exposes all faults in a consistent way.

---

## 2. PLC Project Structure

### 2.1 Main Components

#### GVLs (Global Variable Lists)

- `GVL_Steppers`  
  Holds stepper FB instances (e.g. `fbRail1`, `fbRail2`, …), NC axis references and rail-related globals such as positions, velocities, homing flags.

- `GVL_LinearActuators`  
  Holds linear actuator FB instances and command/status structs for:
  - Chariots
  - Compressors
  - Sensors
  - Heat exchangers / regeneration coupolas
  - Other on/off actuators

- `GVL_Alarms`  
  Central alarm instances, typically one `ST_Alarm` per rail/actuator fault type (e.g. `R1_Stepper_Homing_Failed`).

- `GVL_Process`  
  High-level process states, start/stop flags, sequencing logic and global interlocks.

---

#### DUTs (Data Types)

- `ENUM_StepperMotorState`  
  All possible states for a stepper, e.g.:
  - `LAS_Stepper_Idle`
  - `LAS_Stepper_Ready`
  - `LAS_Stepper_Homing`
  - `LAS_Stepper_Moving`
  - `LAS_Stepper_Error`
  - `LAS_Stepper_Hardware_Check`
  - `LAS_Stepper_PowerError`

- `ST_Alarm`  
  Generic alarm structure:
  - `bCondition` – TRUE = alarm active  
  - `nID` – unique numeric ID  
  - `nSeverity` – severity level (e.g. 1–5)  
  - `sMessage` – human-readable description

- `ST_SystemAlarmStatus`  
  Aggregated alarm status per zone and for the whole system (e.g. `bAdsorptionOK`, `bRegenerationOK`, `bAnyCriticalAlarm`).

- Per-actuator command/status structures  
  For example:
  - `ST_StepperMotorCmd` – desired position, homing request, status flags  
  - `ST_LinearActuatorCmd` – extend / retract commands, timeout, state

---

#### POUs (Program Organization Units)

Key POUs:

- `FB_StepperMotor`  
  One instance per rail. Encapsulates axis control, homing, moves, jogging, errors.

- `FB_LinearActuator`  
  One instance per linear actuator. Drives DOs, monitors limit switches, handles timeouts.

- `F_IssueActuatorCommand`  
  Interprets high-level string commands (e.g. `"Chariot1"`, `"Motor3"`) and writes to the correct command struct.

- `F_ValidateCommandAndReturn`  
  Validates strings of the form `"R<n>_<Name>"` and extracts rail and device name.

- Process / sequence programs  
  Implement the adsorption/regeneration process using the above FBs and command/API layer.

---

## 3. Motion Control – Steppers (Rails)

### 3.1 `FB_StepperMotor` – Overview

**Purpose**

Encapsulates all logic for one rail stepper axis:

- Power management (`KeepAlive`, `PowerDown`)
- Homing (using EL7062 + proximity sensor as PLC cam)
- Absolute positioning (`MoveToPosition`)
- Jogging (`Jog`)
- Error handling and alarm generation

**Return code convention**

Most methods follow this pattern:

- `1` – done / success  
- `0` – busy / still executing  
- `-1` – error (check alarm and error ID)

**Typical methods**

(Names may differ slightly depending on your exact implementation.)

- `Home(bIsForced : BOOL) : INT`  
  - Normal homing (`bIsForced = FALSE`)  
  - Forced homing that ignores previous calibration (`bIsForced = TRUE`)

- `MoveToPosition(fPosition : LREAL) : INT`  
  - Move axis to target position in engineering units (e.g. mm).

- `Jog(Mode, JogDir, Execute) : INT`  
  - Jog motion in positive/negative direction at slow/fast preset speeds.

- `Reset(bExecute : BOOL) : INT`  
  - Clear errors, reset state machine and related alarms.

- `KeepAlive(Error => bError) : BOOL`  
  - Keep axis powered while returning an “alive” status. If `Error` is TRUE, axis is not healthy.

- `PowerDown() : INT`  
  - Disable the axis / power stage when no longer needed.

---

### 3.2 `FB_StepperMotor` – State Machine

`ENUM_StepperMotorState` typically includes:

- `LAS_Stepper_Idle` – axis not yet powered or initialized  
- `LAS_Stepper_Ready` – axis ready for movement  
- `LAS_Stepper_Homing` – homing in progress  
- `LAS_Stepper_Moving` – absolute move in progress  
- `LAS_Stepper_Error` – error state; requires `Reset`  
- `LAS_Stepper_Hardware_Check` – hardware checks / power-up  
- `LAS_Stepper_PowerError` – power or enable issues  

The FB uses a `CASE eState OF ... END_CASE` structure and calls MC_ function blocks (`MC_Power`, `MC_Home`, `MC_MoveAbsolute`, `MC_Stop`) once per cycle. Transitions are based on `Busy`, `Done`, `Error` flags and internal conditions.

---

### 3.3 Homing Strategy

- Homing is performed against a proximity sensor configured as a **PLC cam**.
- TwinCAT `ST_HomingOptions.ReferenceMode = ENCODERREFERENCEMODE_PLCCAM`.
- The EL7062 operates in **DMC (Drive Motion Control)** mode; `MC_Home` drives motion while monitoring the cam via `bCalibrationCam`.

Typical sequence:

1. Ensure axis is enabled and free of errors.
2. Start homing with something like:

   ```pascal
   fbHome(
       Axis            := pAxis^,
       Execute         := bHomeExecute,
       HomingMode      := MC_DefaultHoming,
       Position        := ReferencePos,
       bCalibrationCam := pProx^,
       Options         := HomeOptions
   );
