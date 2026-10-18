(* A conveyor with a drive on the bus, as an S7 drives a SINAMICS with
   PROFIdrive standard telegram 1. Run it with File > New from a demo >
   Conveyor with a drive: press Start, turn the Speed potentiometer.

   Telegram 1 is two words each way:
     %QW12  STW1     control word    16#047E switches on, 16#047F runs
     %QW14  NSOLL_A  speed setpoint  16#4000 (16384) = 100 %
     %IW12  ZSW1     status word
     %IW14  NIST_A   actual speed    the same scale

   The status bits are read as single bits. A word is big-endian on an S7:
   %IW12's high byte is %IB12 and its low byte %IB13, so ZSW1 bit 0 is
   %IX13.0 and bit 8 is %IX12.0. *)
PROGRAM Main
VAR
    start AT %IX0.0 : BOOL; (* panel button *)
    stop AT %IX0.1 : BOOL; (* panel button, wired normally closed *)
    acknowledge AT %IX0.2 : BOOL; (* panel button: acknowledge a drive fault *)
    speedPot AT %IW16 : INT; (* the Speed potentiometer: 0 to 27648 *)
    driveRunning AT %IX13.2 : BOOL; (* ZSW1.2 operation enabled *)
    driveFault AT %IX13.3 : BOOL; (* ZSW1.3 fault *)
    driveInhibited AT %IX13.6 : BOOL; (* ZSW1.6 switching on inhibited *)
    driveAtSpeed AT %IX12.0 : BOOL; (* ZSW1.8 actual speed at the setpoint *)
    actualSpeed AT %IW14 : INT; (* NIST_A *)
    stw1 AT %QW12 : INT; (* STW1 *)
    nsoll AT %QW14 : INT; (* NSOLL_A *)
    runningLamp AT %QX0.0 : BOOL;
    faultLamp AT %QX0.1 : BOOL;
    atSpeedLamp AT %QX0.2 : BOOL;
    run : BOOL;
    speedPercent : REAL; (* NIST_A in %, to watch online *)
END_VAR

    // Start holds itself; Stop (normally closed) and a drive fault drop it.
    run := (start OR run) AND stop AND NOT driveFault;

    // STW1: 16#047E = OFF2 and OFF3 released, ramp and setpoint enabled,
    // control by PLC, but OFF1 active: "switch on, ready". 16#047F adds
    // bit 0: run. A drive in "switching on inhibited" (after power-up, a
    // fault, OFF2 or OFF3) has to see 16#047E before it accepts 16#047F,
    // so while it says so, send 16#047E even if Start is held.
    IF run AND NOT driveInhibited THEN
        stw1 := 16#047F;
    ELSE
        stw1 := 16#047E;
    END_IF;
    // Bit 7 acknowledges a fault on its rising edge. (There's no OR on INT
    // in soft_plc yet; bit 7 is 0 in both words, so adding 16#80 sets it.)
    IF acknowledge THEN
        stw1 := stw1 + 16#80;
    END_IF;

    // NSOLL_A: the potentiometer's 0..27648 scaled to 0..16384 (100 %).
    // Multiplying in INT would overflow (27648 * 16384), so in DINT.
    nsoll := DINT_TO_INT(INT_TO_DINT(speedPot) * 16384 / 27648);

    runningLamp := driveRunning;
    faultLamp := driveFault;
    atSpeedLamp := driveRunning AND driveAtSpeed;
    speedPercent := INT_TO_REAL(actualSpeed) * 100.0 / 16384.0;
END_PROGRAM
