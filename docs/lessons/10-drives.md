# Lesson 10: Analog I/O and drives

Until now, a PLC program could only switch a belt on and off. Milestone 10 brings two changes:
- **Words** reach the PLC: analog values, and a drive's control and status words.
- A conveyor can have a **variable-speed drive on the bus**, driven the way an S7 drives a SINAMICS: PROFIdrive standard telegram 1, with the drive's state machine, ramps, stops and faults.

```
   PLC                                     drive                       belt
 ┌──────────────┐  %QW12 STW1    16#047F  ┌─────────────────────┐
 │ drive.st     │ ───────────────────────►│ state machine       │
 │  run, speed  │  %QW14 NSOLL_A 11851    │ ramp (acceleration) │ ──► 0.72 m/s
 │  lamps       │ ◄───────────────────────│ OFF1 / OFF2 / OFF3  │
 └──────────────┘  %IW12 ZSW1    16#FB37  │ fault, acknowledge  │
        ▲          %IW14 NIST_A  11851    └─────────────────────┘
        │ %IW16 Speed potentiometer, 0..27648
```

Try it: **File > New from a demo > Conveyor with a drive**. Press **Start**, turn the **Speed** potentiometer, and select the conveyor to watch both words bit by bit in the inspector. Then press **Trip it**, and acknowledge the fault.

---

## Part 1: Words in the I/O image

### 1.1 More room
The I/O image had 16 bytes each way. A drive takes four words (8 bytes) and an encoder a double word, so it's now **64 bytes** each way. Bits still go in bytes 0-7; words and double words start at byte 8, and `freeAddress` hands out the next free one, aligned to its size. A real S7-1200 has 1024 bytes; 64 is plenty for a simulated line.

There's one catch. `soft_plc --s7` on its own still has 16 bytes, and a live S7 link that wrote 64 into it would be refused. So the live link exchanges only the scene's **bytes** setting, 16 by default. It's a limit to lift in soft_plc later.

### 1.2 Analog values: 0 to 27648
A Siemens analog input module turns 0-10 V (or 4-20 mA) into **0 to 27648**, not 0 to 32767. The range above 27648, up to 32511, is the *overrange*: the module can still report a signal a bit too high instead of clipping it, and 32767 means "wire break" or "overflow". The panel's **potentiometers** give exactly that, 0 to 27648, on an input word.

### 1.3 How words look
The I/O window now shows words as TIA's watch table does: `16#047F` in hex, with the INT and WORD values in the tooltip. A control word is a set of bits, so hex is how you read it: 047F is 0000 0100 0111 1111.

---

## Part 2: PROFIdrive telegram 1

### 2.1 Two words each way
Standard telegram 1, the simplest speed-control telegram:

| Word | Direction | Meaning |
|---|---|---|
| STW1 | PLC → drive | control word: on, the stops, enable, acknowledge, "control by PLC" |
| NSOLL_A | PLC → drive | speed setpoint, **16#4000 (16384) = 100 %** of the drive's reference speed |
| ZSW1 | drive → PLC | status word: ready, running, fault, at speed, ... |
| NIST_A | drive → PLC | actual speed, same scale |

Why is 100 % 16#4000 and not 32767? For the same reason as analog modules' 27648: headroom. A signed 16-bit word with 100 % at 16384 can carry up to almost 200 %, so field weakening or a setpoint above the reference speed still fits.

### 2.2 The state machine
A drive won't simply run because a bit is on. It goes through states, and the PLC moves it along with STW1:

```
switching on inhibited ──STW1.0 = 0 (16#047E)──► ready to switch on
ready to switch on     ──STW1.0 = 1 (16#047F)──► switched on ──STW1.3 = 1──► operation enabled
```

16#047E is OFF2 and OFF3 released (bits 1, 2), enable operation (3), ramp enable, ramp start and setpoint enable (4, 5, 6), and control by PLC (10), with bit 0, ON, still off. 16#047F adds bit 0. Those two words are the first thing anyone driving a SINAMICS learns.

The important rule: **after power-up, a fault, OFF2 or OFF3, the drive is "switching on inhibited", and only leaves it on seeing ON = 0.** A run command left on from before therefore can't restart the motor by itself. That's a safety feature (restart inhibit), and the simulator has it.

### 2.3 It caught a bug in my program
The first `drive.st` sent 16#047F whenever Start was latched. In the test, Start was pressed after the drive had already seen 16#047E, and everything worked. In the screenshot run, Start was held from the very first scan. The drive stayed in "switching on inhibited" forever, showing ZSW1 = 16#EA70 with bit 6 set. The fix is a standard part of every drive block:

```
IF run AND NOT driveInhibited THEN   (* ZSW1.6 *)
    stw1 := 16#047F;
ELSE
    stw1 := 16#047E;
END_IF;
```

A test now holds Start from power-up. This is exactly the kind of bug that shows up on commissioning day, when the line is switched on with a switch left on. Finding it in a simulator is the point of having one.

### 2.4 Three ways to stop

| | STW1 | What happens | Used for |
|---|---|---|---|
| **OFF1** | bit 0 = 0 | down the normal ramp, then the pulses go off | a normal stop |
| **OFF2** | bit 1 = 0 | pulses off at once: the motor **coasts** | safety: the motor must not be driven at all |
| **OFF3** | bit 2 = 0 | down the **quick-stop** ramp, then off | an emergency stop that brakes (stop category 1) |

After OFF2 or OFF3, the drive is "inhibited" again, so it needs the 047E/047F sequence to restart. After OFF1 it's only "ready to switch on". Clearing bit 3 (enable operation) also cuts the pulses, so the motor coasts. Clearing bit 6 (setpoint enable) ramps to zero but stays enabled.

A coasting belt doesn't stop instantly. The `coasting` setting is how fast the loaded belt slows down by friction alone, 1.5 m/s² by default.

### 2.5 Faults
**Trip it** raises a fault, as an overload would. The pulses go off, the belt coasts, ZSW1 bit 3 comes on, and the drive ignores "run". The PLC acknowledges with a **rising edge** of STW1 bit 7. It's an edge because a held "acknowledge" mustn't silently clear each new fault as it comes. After acknowledging, the drive is "inhibited", and the restart rule above applies.

One fault comes from the PLC side: if STW1 bit 10, **control by PLC**, goes off while the drive runs, the drive faults with F07220. That's what happens when the PLC goes to STOP and its outputs drop to 0: the drive won't carry on with nobody in charge.

---

## Part 3: The program side

### 3.1 A word's bits, and the S7 byte order
soft_plc has no `WORD` type or bit slices (`zsw1.%X3`) yet, so `drive.st` reads ZSW1's bits as individual input bits. That brings out something every S7 programmer trips over once. **Words are big-endian:** `%IW12`'s *high* byte is `%IB12`, and its low byte is `%IB13`. So:

| ZSW1 bit | Meaning | Address |
|---|---|---|
| 0 | ready to switch on | `%IX13.0` |
| 2 | operation enabled | `%IX13.2` |
| 3 | fault | `%IX13.3` |
| 6 | switching on inhibited | `%IX13.6` |
| 8 | at speed | `%IX12.0` |

Bit 0 of the word is in the **second** byte. The inspector prints the addresses for the drive's own status word, so you don't have to work them out.

### 3.2 Scaling without overflow
The potentiometer's 0..27648 becomes NSOLL_A's 0..16384:

```
nsoll := DINT_TO_INT(INT_TO_DINT(speedPot) * 16384 / 27648);
```

27648 × 16384 = 452 984 832, far beyond an INT's 32 767. So the multiplication happens in DINT, and the division goes last so no precision is lost. Dividing first (`speedPot / 27648 * 16384`) would give 0 for everything below full scale.

### 3.3 Setting bit 7 without OR
soft_plc has no `OR` on integers yet either. The program sets the acknowledge bit with `stw1 + 16#80`, which works only because bit 7 is 0 in both 047E and 047F. It's a workaround, noted as something for soft_plc to gain: `WORD`, bit operations on words, and slices.

---

## Part 4: The model

### 4.1 A drive on its own
`Drive` (`src/sim/drive.hpp`) knows nothing of belts or metres. It works in **fractions of its reference speed** (1.0 = 100 % = 16#4000):
- `command(controlWord, setpoint)` is called at each scan, with what the PLC wrote.
- `update(seconds, acceleration, quickStop, coast)` is called every 1 ms step.
- `statusWord()` and `actualSpeed()` are what the PLC reads back.

The World turns fractions into metres per second with the conveyor's reference speed, and turns the conveyor's acceleration into the drive's ramp. That's why `drive_test.cpp` can check the whole state machine without a World or physics, in nine cases.

### 4.2 Following the state machine through
One control word can move the drive through several states: 16#047F from "ready to switch on" passes through "switched on" to "operation enabled" in one go. `step()` applies the rules in a loop until the state stops changing, with a guard against looping forever. A real drive takes a few milliseconds for the same path; within one 10 ms scan, it's the same.

### 4.3 Without a program
A belt without a control program always runs (lesson 5). A belt with a drive and no program does the same: the World plays the PLC's part, sending 047E, then 047F with the target speed as setpoint. The rule is the same either way: no program, the line runs.

---

## Part 5: C++ and design

### 5.1 Bit masks as constants, not an enum class
STW1's bits are `constexpr std::uint16_t` constants in a namespace (`stw1::kOn`, `stw1::kRun`), not an `enum class`. A control word is built and tested with `|`, `&` and `~`, and an `enum class` deliberately has none of those without casts. So `stw1::kRun & ~stw1::kNoQuickStop` reads like the documentation, and the namespace keeps the names apart from ZSW1's `zsw1::kNoQuickStop`.

### 5.2 `[[fallthrough]]` that means something
The status word builds up with the state: "operation enabled" implies "ready to operate", which implies "ready to switch on". The `switch` in `statusWord()` lets each case fall into the next, and `[[fallthrough]]` says so on purpose. Without it, the compiler warns, rightly, since a forgotten `break` is a classic bug.

### 5.3 Signed words through unsigned bytes
NIST_A is signed (backwards is negative), but the I/O image stores unsigned bytes. `static_cast<std::uint16_t>(actualSpeed())` keeps the same 16 bits (two's complement), and the PLC reads them back as a negative INT. Nothing is lost if both sides agree on the type, which is exactly what a PLC's `AT %IW14 : INT` declares.

---

## Part 6: Testing

- **`sim/drive_test.cpp`** (9 cases):
  - switching on (047E, then 047F; a held "on" doesn't start an inhibited drive);
  - the ramp, NIST_A, "at speed" and running backwards;
  - the three stops, and setpoint disable;
  - a fault coasting, ignoring "run", and acknowledged by an edge;
  - "control by PLC" missing;
  - a belt in the World following a test program through its words, with the drive tripping;
  - running without a program;
  - a potentiometer held to 27648;
  - drives and potentiometers saved with the scene.
- **`sim/control_test.cpp`:** the larger image (14 double words fit from byte 8).
- **`plc/plc_test.cpp`** (2 new cases): soft_plc running `drive.st`:
  - start, speed from the potentiometer, at speed, a trip with coasting down, Start ignored while faulted, acknowledge, restart, and Stop down the ramp;
  - Start held from power-up still starts, through 047E.
- **By eye:** the demo at 72 % (the potentiometer at 20000), STW1 16#047F, ZSW1 16#FB37, the Running and At speed lamps lit, and the words bit by bit in the inspector.

---

## Review questions

**1.** NSOLL_A is 8192. What speed does the drive run at, with a reference speed of 1.5 m/s?
<details><summary>Answer</summary>

8192 / 16384 = 50 % of 1.5 m/s: 0.75 m/s.
</details>

**2.** The PLC sends 16#047F from its first scan after power-up. Why doesn't the drive start, and what should the program do?
<details><summary>Answer</summary>

After power-up the drive is "switching on inhibited", and it only leaves that state on seeing ON (bit 0) = 0. So the program sends 16#047E while ZSW1 bit 6 says "inhibited", then 16#047F. That's the restart inhibit: a run command left on can't start a motor by itself.
</details>

**3.** ZSW1 is at `%IW20`. At which bit address is its "fault" bit (bit 3), and why not `%IX20.3`?
<details><summary>Answer</summary>

`%IX21.3`. S7 words are big-endian: `%IW20`'s high byte (bits 8-15) is `%IB20`, and its low byte (bits 0-7) is `%IB21`.
</details>

**4.** Why does a drive acknowledge a fault on the *rising edge* of bit 7, not while it's on?
<details><summary>Answer</summary>

So an acknowledge left on can't clear each new fault the moment it happens. Every fault has to be seen and acknowledged on purpose.
</details>
