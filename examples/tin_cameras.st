(* The tin line's camera triggering, as the PLC in the real project had it.
   One photocell (B1) sees each tin. K1, K2 and K3 look at the tin right
   there and are triggered at once; K4 sits 15 cm further on, so its trigger
   is delayed 150 ms with a timer. One timer: a tin that arrives while it's
   still running gets no trigger for K4 at all.

   Run it with the tin demo: in factory_sim, Load the tin demo, then choose
   soft_plc (lockstep) and this file. It does exactly what the built-in
   camera triggering does with K4 on "one timer". *)
PROGRAM Main
VAR
    tin AT %IX0.0 : BOOL; (* B1: a tin in the light barrier *)
    k1 AT %QX0.0 : BOOL; (* camera triggers: a rising edge takes a picture *)
    k2 AT %QX0.1 : BOOL;
    k3 AT %QX0.2 : BOOL;
    k4 AT %QX0.3 : BOOL;
    newTin : R_TRIG;
    k4Waiting : BOOL; (* K4's timer is running for a tin *)
    k4Delay : TON;
    k4Lost : INT; (* tins that came while it was running *)
END_VAR

    newTin(CLK := tin);

    // K1-K3: a one-scan pulse in the scan that sees the tin.
    k1 := newTin.Q;
    k2 := newTin.Q;
    k3 := newTin.Q;

    // K4: start the timer, unless it's already running for the tin before.
    IF newTin.Q THEN
        IF k4Waiting THEN
            k4Lost := k4Lost + 1;
        ELSE
            k4Waiting := TRUE;
        END_IF;
    END_IF;
    k4Delay(IN := k4Waiting, PT := T#150ms);
    k4 := k4Delay.Q;
    IF k4Delay.Q THEN
        // Fired: stop the timer now, so the next tin starts it afresh.
        k4Waiting := FALSE;
        k4Delay(IN := FALSE, PT := T#150ms);
    END_IF;
END_PROGRAM
