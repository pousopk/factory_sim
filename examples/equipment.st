(* The line equipment demo (File > New from a demo > Line equipment):
   C1 accumulates boxes in four zones and releases them one at a time; the
   turntable C2 turns each one through 90 degrees onto C3; the lift C4
   takes it up to C5.

   The zone logic itself (zero-pressure accumulation) is on C1's zone cards;
   the PLC only says when the last zone may let a box go. The turntable and
   the lift are each a state machine: a CASE on a step number, as a GRAPH
   sequence would be in TIA. *)
PROGRAM Main
VAR
    lastZone AT %IX0.3 : BOOL; (* C1's last zone eye *)
    release AT %QX0.0 : BOOL; (* C1's last zone may discharge *)
    turn AT %QX0.1 : BOOL; (* turntable: turn (off: back home) *)
    turned AT %IX0.4 : BOOL; (* turntable at 90 degrees *)
    tableHome AT %IX0.5 : BOOL; (* turntable at home *)
    tableBelt AT %QX0.2 : BOOL; (* turntable belt *)
    onTable AT %IX0.6 : BOOL; (* BT: a box on the turntable, in place *)
    c3Run AT %QX0.3 : BOOL; (* C3's motor *)
    c3End AT %IX0.7 : BOOL; (* B3: a box at the end of C3 *)
    raise AT %QX0.4 : BOOL; (* lift: up (off: down) *)
    liftUp AT %IX1.0 : BOOL;
    liftDown AT %IX1.1 : BOOL;
    liftBelt AT %QX0.5 : BOOL;
    onLift AT %IX1.2 : BOOL; (* BL: a box on the lift *)
    table : INT; (* the turntable's step *)
    lift : INT; (* the lift's step *)
    released : BOOL; (* a box has left C1 for the table *)
    leftZone : F_TRIG;
    tableClear : TON;
    liftClear : TON;
END_VAR

    // A box leaves C1's last zone when its eye goes off.
    leftZone(CLK := lastZone);

    CASE table OF
        0: // Home, empty: the belt runs, and C1 may send one box.
            turn := FALSE;
            tableBelt := TRUE;
            IF leftZone.Q THEN
                released := TRUE;
            END_IF;
            release := NOT released;
            IF onTable THEN
                table := 1;
            END_IF;
        1: // A box in place: stop, and turn.
            tableBelt := FALSE;
            release := FALSE;
            turn := TRUE;
            IF turned THEN
                table := 2;
            END_IF;
        2: // Turned: run the box off onto C3, until the table has been clear 1 s.
            tableBelt := TRUE;
            tableClear(IN := NOT onTable, PT := T#1s);
            IF tableClear.Q THEN
                tableClear(IN := FALSE, PT := T#1s);
                table := 3;
            END_IF;
        3: // Back home.
            tableBelt := FALSE;
            turn := FALSE;
            IF tableHome THEN
                released := FALSE;
                table := 0;
            END_IF;
    END_CASE;

    CASE lift OF
        0: // Down, empty: the belt runs.
            raise := FALSE;
            liftBelt := TRUE;
            IF onLift THEN
                lift := 1;
            END_IF;
        1: // A box on it: stop, and up.
            liftBelt := FALSE;
            raise := TRUE;
            IF liftUp THEN
                lift := 2;
            END_IF;
        2: // Up: run the box off onto C5, until the lift has been clear 1 s.
            liftBelt := TRUE;
            liftClear(IN := NOT onLift, PT := T#1s);
            IF liftClear.Q THEN
                liftClear(IN := FALSE, PT := T#1s);
                lift := 3;
            END_IF;
        3: // Down again.
            liftBelt := FALSE;
            raise := FALSE;
            IF liftDown THEN
                lift := 0;
            END_IF;
    END_CASE;

    // C3 holds a box at its end while the lift is busy.
    c3Run := NOT (c3End AND lift <> 0);
END_PROGRAM
