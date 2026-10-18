(* The conveyor program from ladder_editor/examples/conveyor (a KOP PROGRAM
   and an ST function block), as its Export ST writes it. Generated: edit
   the ladder project, not this file. *)
PROGRAM Main
VAR
    start AT %IX0.0 : BOOL; (* start button *)
    stop AT %IX0.1 : BOOL; (* stop button, wired normally closed *)
    eStop AT %IX0.2 : BOOL; (* emergency stop, wired normally closed *)
    boxAtStart AT %IX0.3 : BOOL; (* light barrier at the start of the belt *)
    boxAtEnd AT %IX0.4 : BOOL; (* light barrier at the end of the belt *)
    resetButton AT %IX0.5 : BOOL; (* clears a jam and the box count *)
    warningLamp AT %QX0.1 : BOOL; (* blinks while jammed *)
    batchFull AT %QX0.2 : BOOL; (* 100 boxes counted *)
    tooFast AT %QX0.3 : BOOL; (* more than 30 boxes a minute *)
    running : BOOL;
    jammed : BOOL;
    travel : TON; (* how long the box has been travelling *)
    travelTime : TIME;
    arrived : R_TRIG;
    boxCount : CTU;
    boxes : INT;
    blinkOn : TON;
    blinkOff : TON;
    blinkPhase : BOOL;
    rate : BoxRate; (* boxes per minute: a function block written in ST *)
    boxesPerMinute : INT;
    _kop_QX0_0 AT %QX0.0 : BOOL; (* generated: the operand %QX0.0 *)
END_VAR

    // Network 1: Start/stop: start holds itself, stop and the emergency stop (both wired normally closed) always win
    running := (start OR running) AND stop AND eStop;

    // Network 2: The belt motor follows "running", unless the jam detector tripped
    _kop_QX0_0 := running AND NOT jammed;

    // Network 3: A box should reach the end sensor within 8 s of passing the start sensor
    travel(IN := boxAtStart AND NOT boxAtEnd, PT := T#8s);
    travelTime := travel.ET;
    IF travel.Q THEN jammed := TRUE; END_IF;

    // Network 4: The operator clears the jam with the reset button, once the end sensor is free again
    IF resetButton AND NOT boxAtEnd THEN jammed := FALSE; END_IF;

    // Network 5: Count boxes: one per rising edge of the end sensor
    arrived(CLK := boxAtEnd);
    boxCount(CU := arrived.Q, R := resetButton, PV := 100);
    boxes := boxCount.CV;
    batchFull := boxCount.Q;

    // Network 6: The warning lamp blinks while jammed: two timers, each starting the other
    blinkOn(IN := jammed AND NOT blinkOff.Q, PT := T#500ms);
    warningLamp := blinkOn.Q;

    // Network 7
    blinkOff(IN := jammed AND blinkOn.Q, PT := T#500ms);
    blinkPhase := blinkOff.Q;

    // Network 8: Boxes per minute, from the ST block BoxRate; tooFast lights above 30
    rate(arrived := boxAtEnd, limit := 30);
    boxesPerMinute := rate.perMinute;
    tooFast := rate.tooFast;

END_PROGRAM
(* Boxes per minute: counts the rising edges of `arrived` and publishes
   the count once a minute. Written in ST and used as a box in the KOP
   program: `arrived` (its first BOOL input) takes the rung's power, and
   `tooFast` (its first BOOL output) gives it back. *)
FUNCTION_BLOCK BoxRate
VAR_INPUT
    arrived : BOOL;
    limit : INT := 30;
END_VAR
VAR_OUTPUT
    tooFast : BOOL;
    perMinute : INT;
END_VAR
VAR
    edge : R_TRIG;
    window : TON;
    count : INT;
END_VAR
    edge(CLK := arrived);
    IF edge.Q THEN
        count := count + 1;
    END_IF;
    window(IN := NOT window.Q, PT := T#1m);
    IF window.Q THEN
        perMinute := count;
        count := 0;
    END_IF;
    tooFast := perMinute > limit;
END_FUNCTION_BLOCK
