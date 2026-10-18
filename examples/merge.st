(* The merge demo (File > New from a demo > Merge): a side conveyor C2
   joins the main line C1 from the left. The main line has priority: the
   side conveyor only sends a box when there's a gap.

   A gap means B1, before the merge, has seen nothing for long enough for
   the last box on the main line to have passed the merge. Until then, a box
   at the end of C2 (B2) waits there with its belt stopped. *)
PROGRAM Main
VAR
    mainBox AT %IX0.0 : BOOL; (* B1: a box on the main line, before the merge *)
    sideWaiting AT %IX0.1 : BOOL; (* B2: a box at the end of the side conveyor *)
    sideRun AT %QX0.0 : BOOL; (* C2's motor *)
    passing : TOF; (* still "busy" for a while after B1 clears *)
END_VAR

    // When B1 (at 2.2 m) goes off, the box's back has passed it. Its back has
    // to get past the merge (3.0 m, plus half the side belt, 0.2 m) before a
    // side box may come: 1.0 m, 2 s at 0.5 m/s. 2.2 s with a margin.
    passing(IN := mainBox, PT := T#2200ms);

    // The side belt runs, except when a box waits at its end and the main
    // line isn't clear: then it holds the box there.
    sideRun := NOT (sideWaiting AND passing.Q);
END_PROGRAM
