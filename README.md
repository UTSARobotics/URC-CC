# SCARA Pen Plotter Walkthrough

This project controls a small SCARA-style drawing arm with:

- `R` joint 1: revolute servo at the base
- `R` joint 2: revolute servo at the elbow
- `L` axis: pen lift servo used as a simple up/down actuator

The arm is controlled over serial. It can:

- move to Cartesian targets
- move directly to requested servo angles
- draw straight lines by breaking motion into many tiny points
- draw circles
- optionally run a startup square test

This README is written for a complete beginner. It explains both what the code does and the inverse-kinematics math behind it.

## Quick Start

If you want to use the robot before reading the whole document, start here.

### 1. Upload the sketch

Upload:

- [RAS_URC_CC_Arm.ino](/home/loopk/Arduino/InvControl/RAS_URC_CC_Arm/RAS_URC_CC_Arm.ino)

Then open the Arduino Serial Monitor at:

- `115200` baud
- newline line ending

### 2. Check that the arm homes correctly

Type:

```text
HOME
STATUS
```

That moves the arm to its centered start pose and prints the current position.

### 3. Try a few angle moves

Type:

```text
A 90 90
A 50 50
```

This is the simplest way to confirm:

- the servos are connected
- the arm moves
- the reported Cartesian position changes

### 4. Try simple Cartesian moves

Type:

```text
G 0 0
G 0 10
G 10 10
G 10 0
G 0 0
```

That should make a square path around the centered origin.

### 5. Lift and lower the pen

Type:

```text
PEN UP
PEN DOWN
```

If the pen directions are backwards, use:

```text
CAL SET PEN_UP <angle>
CAL SET PEN_DOWN <angle>
```

### 6. Draw a circle

Type:

```text
C 0 0 10 24
```

That draws a 10 mm radius circle around the current user origin using 24 segments.

### 7. Queue many commands on one line

This parser supports multiple commands in one input line:

```text
G 0 0 G 0 10 G 10 10 G 10 0 G 0 0
```

That is useful for drawing polygons and test paths.

## 1. What Hardware This Code Assumes

The sketch is built around these physical dimensions:

- `LINK1 = 35.0 mm`
- `LINK2 = 42.0 mm`

These are the two planar arm links used for inverse kinematics.

The pins are:

- joint 1 servo: pin `9`
- joint 2 servo: pin `10`
- pen lift servo: pin `11`

The servo library used is Arduino `Servo.h`.

## 2. What A SCARA Arm Is

A 2-link SCARA arm is a simple planar robot arm:

- the first servo rotates link 1
- the second servo rotates link 2 relative to link 1
- together they place the pen tip somewhere in 2D space

If you know the two joint angles, you can calculate the pen tip position. That is called **forward kinematics**.

If you know the pen tip position you want, and you need to find the joint angles that will place the pen there, that is called **inverse kinematics**.

This project uses both:

- forward kinematics for reporting position
- inverse kinematics for moving to `G x y` targets

## 3. Project Files

The main Arduino sketch is:

- [RAS_URC_CC_Arm.ino](/home/loopk/Arduino/InvControl/RAS_URC_CC_Arm/RAS_URC_CC_Arm.ino)

There is also a local math test program:

- [ik_test.c](/home/loopk/Arduino/InvControl/ik_test.c)

That C file is important because it verifies the kinematics and line-planning logic outside Arduino before code is copied into the sketch.

## 4. High-Level Program Flow

At a high level, the sketch does this:

1. Start serial and attach the three servos.
2. Move the arm to a predefined centered start pose.
3. Wait for serial input.
4. Parse commands like `G`, `A`, `C`, `HOME`, `PEN UP`, and `PEN DOWN`.
5. For motion commands, convert the request into many tiny line segments.
6. Solve inverse kinematics for each tiny point.
7. Write the new servo angles.

That means the arm does not jump directly to a final point. Instead it walks there through many small Cartesian points.

## 5. Important Data Structures

The sketch uses two main structs.

### `Calibration`

This stores servo calibration values:

- joint offsets
- joint inversion flags
- pen up angle
- pen down angle

These values let the logical math stay separate from the real hardware servo orientation.

### `MotionState`

This stores the current state of the arm:

- centered origin in absolute Cartesian space
- current relative `x, y`
- previous relative `x, y`
- current joint angle estimates
- current pen angle
- whether the pen is down

This state lets the code:

- know where the arm thinks it is
- plan lines from the current point
- report status back to serial

## 6. Coordinate Systems

This is one of the most important parts of the project.

There are multiple coordinate systems in play.

### A. Servo Angle Space

This is just the raw servo command space:

- joint 1: `0` to `180`
- joint 2: `0` to `180`

Example:

```text
A 90 90
```

That tells the arm to move to servo angle 90 degrees on both joints.

### B. Internal Cartesian Space

This is the raw XY position of the pen tip relative to the arm base, computed by forward kinematics.

The math uses the actual link lengths:

- link 1 = `35 mm`
- link 2 = `42 mm`

### C. User Cartesian Space

This is the coordinate system used by `G x y`.

The origin is not the arm base. The origin is the chosen centered start pose.

So:

- `G 0 0` means “go to the center/start point”
- `G 10 0` means “go 10 mm in +X from the center”
- `G 0 10` means “go 10 mm in +Y from the center”

The sketch also rotates this user frame to match the physical mounting of the arm.

That means the reported axes match the real machine:

- `+Y` is physically up
- `-Y` is physically down
- `+X` is physically forward/right
- `-X` is physically back/left

This is why there are conversion functions between absolute coordinates and relative user coordinates.

## 7. Forward Kinematics

Forward kinematics means:

> “Given joint angles, where is the pen tip?”

That is handled by `forwardKinematics(...)`.

The core math is:

```cpp
x = L1 * cos(q1) + L2 * cos(q1 + q2)
y = L1 * sin(q1) + L2 * sin(q1 + q2)
```

Where:

- `q1` is joint 1 angle in radians
- `q2` is joint 2 angle in radians
- `L1` and `L2` are the link lengths

This is the standard 2-link planar robot equation.

The code first converts servo angles into the math frame by subtracting `90` degrees, because the mechanical “zero” of the robot is not the same as the trigonometric zero used by the equations.

## 8. Inverse Kinematics

Inverse kinematics means:

> “Given a target `x, y`, what joint angles put the pen there?”

This sketch uses two layers for IK:

### A. Analytic 2-link seed

The function `solveIKAnalytic(...)` computes a closed-form solution for a 2-link arm.

This uses the geometry equation:

```text
cos(q2) = (x² + y² - L1² - L2²) / (2 * L1 * L2)
```

Then it computes two possible elbow configurations:

- elbow-up
- elbow-down

It chooses the one closest to the arm’s current joint angles.

This is useful because it gives the numerical solver a very good starting point.

### B. Gauss-Newton refinement

After the analytic seed, the code runs `solveIKGaussNewton(...)`.

Gauss-Newton is a numerical optimization method. It repeatedly:

1. computes where the arm currently is
2. compares that against the desired point
3. computes a Jacobian matrix
4. calculates a small correction to the joint angles
5. repeats until the error becomes very small

Why use both methods?

- the analytic solution is fast and accurate for a perfect 2R arm
- Gauss-Newton helps refine the result and makes the solver more robust for continuous motion

## 8.1 The Actual 2-Link IK Math

This section explains the inverse-kinematics math more directly.

We know:

- link 1 length `L1 = 35 mm`
- link 2 length `L2 = 42 mm`
- target point `(x, y)`

We want:

- joint 1 angle `q1`
- joint 2 angle `q2`

### Step 1: Compute distance from the base to the target

The distance from the arm base to the target is:

```text
r = sqrt(x² + y²)
```

This already tells us something important:

- if `r > L1 + L2`, the point is too far away
- if `r < |L1 - L2|`, the point is too close to the base

For this arm:

- max radius = `35 + 42 = 77 mm`
- min radius = `|35 - 42| = 7 mm`

So the target must lie in the ring from `7 mm` to `77 mm`, and it must also satisfy the servo angle limits.

### Step 2: Solve for the elbow angle using the law of cosines

Imagine the triangle formed by:

- link 1
- link 2
- the line from base to target

The law of cosines gives:

```text
cos(q2) = (x² + y² - L1² - L2²) / (2 L1 L2)
```

So:

```text
q2 = acos(cos(q2))
```

But there are two possible elbows:

- elbow-up
- elbow-down

So the code tries both:

```text
q2a = +acos(...)
q2b = -acos(...)
```

That is why the analytic solver produces two possible solutions.

### Step 3: Solve for the shoulder angle

Once `q2` is known, `q1` can be found with:

```text
q1 = atan2(y, x) - atan2(L2 sin(q2), L1 + L2 cos(q2))
```

This formula is doing two things:

- `atan2(y, x)` gives the angle from the base to the target
- the second `atan2(...)` subtracts the link-geometry triangle angle

That gives the shoulder angle needed to place the elbow correctly.

### Step 4: Convert math angles into servo angles

The math frame assumes zero degrees along the positive x-axis.

The physical arm does not use that same zero.

So the sketch converts between them by adding or subtracting `90` degrees:

```text
servo_angle = math_angle_in_degrees + 90
```

That is why you see many places in the code using:

```cpp
(angle * RAD_TO_DEG) + 90.0
```

or:

```cpp
(jointDeg - 90.0f) * DEG_TO_RAD
```

Those are frame-conversion steps between:

- math angles used by trigonometry
- physical servo angles used by the robot

### Step 5: Pick the better branch

If both elbow-up and elbow-down are valid under the servo limits, the code chooses the one closest to the current arm pose.

That matters because without this rule the arm could suddenly “flip” to the other branch, even though both land on the same Cartesian point.

So the solver computes a simple cost:

```text
cost = |candidate_joint1 - current_joint1| + |candidate_joint2 - current_joint2|
```

Then it picks the lower-cost solution.

That makes motion smoother and more predictable.

## 8.2 Why Gauss-Newton Is Still Used After Analytic IK

A beginner might ask:

> If the analytic solution already solves the 2-link arm exactly, why run Gauss-Newton too?

Good question.

The answer is that the analytic solution is used as a **seed**, not as the final motion engine.

Gauss-Newton is still useful because:

- it keeps motion continuous from point to point
- it behaves well when walking along many tiny line subpoints
- it helps stabilize branch choice during dense path execution
- it gives a consistent way to refine the result numerically

So the solver flow is:

1. use exact 2R geometry to get a valid seed
2. refine numerically using the Jacobian

That combination is more robust than using either one alone in this project.

## 8.3 The Jacobian In More Detail

For the 2-link arm:

```text
x = L1 cos(q1) + L2 cos(q1 + q2)
y = L1 sin(q1) + L2 sin(q1 + q2)
```

Take derivatives with respect to the two joint angles:

```text
dx/dq1 = -L1 sin(q1) - L2 sin(q1 + q2)
dx/dq2 = -L2 sin(q1 + q2)
dy/dq1 =  L1 cos(q1) + L2 cos(q1 + q2)
dy/dq2 =  L2 cos(q1 + q2)
```

So the Jacobian is:

```text
J = [ -L1 sin(q1) - L2 sin(q1 + q2)    -L2 sin(q1 + q2) ]
    [  L1 cos(q1) + L2 cos(q1 + q2)     L2 cos(q1 + q2) ]
```

That exact matrix appears in the code through the variables:

- `j11`
- `j12`
- `j21`
- `j22`

### What Gauss-Newton Does With It

At each iteration:

1. compute current position `(x, y)`
2. compute error:

```text
ex = targetX - x
ey = targetY - y
```

3. use the Jacobian to estimate how the angles should change
4. update `q1` and `q2`
5. repeat until the error is tiny

The code also adds a small damping term:

```text
GN_DAMPING = 0.001
```

That helps avoid numerical trouble near singular or poorly conditioned configurations.

## 8.4 What A Singularity Means Here

A singularity is a configuration where small angle changes stop giving useful motion in one direction.

For a 2-link arm this often happens when the arm is nearly:

- fully stretched out
- fully folded back

Near singularities:

- the Jacobian becomes hard to invert cleanly
- motion can become unstable
- the arm may need very large angle changes for very small Cartesian moves

That is why this project:

- avoids bad center poses
- uses analytic seeding
- uses damping
- tests paths point-by-point

## 9. Why A Jacobian Is Needed

The Jacobian tells you how small changes in joint angles affect small changes in end-effector position.

For this arm:

```text
J = [dx/dq1  dx/dq2]
    [dy/dq1  dy/dq2]
```

The code computes:

- `j11`
- `j12`
- `j21`
- `j22`

These are the partial derivatives of `x` and `y` with respect to the two joints.

Gauss-Newton uses that Jacobian to decide how to nudge the angles toward the target.

## 10. Reachability

Not every XY point is reachable.

For a 2-link planar arm, the base-level radial reach limits are:

- minimum radius: `abs(L1 - L2)` = `7 mm`
- maximum radius: `L1 + L2` = `77 mm`

But this project also has servo limits:

- both joints are limited to `0..180`

So a point can be inside the radial ring and still not be reachable under the actual servo angle limits.

That is why the code does not rely only on radius checks.

For line motion, it actually solves IK at every subpoint along the path during precheck.

## 11. Why Straight Lines Are Hard

If you move a 2-joint arm by linearly changing the angles, the pen tip will usually move in a curve.

But for drawing, you want the pen tip to move in a straight line.

So the sketch does this:

1. define a start point and end point in Cartesian space
2. split the path into `100` subpoints
3. solve IK at each subpoint
4. command the servos for each subpoint
5. wait `5 ms`

That is what makes `G` commands trace straight lines.

## 12. The `G` Command

`G x y` means:

> move to the user-relative Cartesian point `(x, y)`

Examples:

```text
G 0 0
G 0 10
G 10 10
G 10 0
G 0 0
```

That sequence makes a square.

The line is not drawn unless the pen is down.

Internally, the sketch:

1. checks whether the full line path is reachable
2. stores the previous point
3. interpolates 100 subpoints
4. solves IK for each subpoint
5. updates the servos

After completion it prints the solved final joint angles.

## 13. Queued Commands On One Line

The parser supports multiple commands in one input line.

Example:

```text
G 0 0 G 0 10 G 10 10 G 10 0 G 0 0
```

The parser reads tokens one by one and executes commands in order.

This is useful for:

- drawing polygons
- chaining moves
- testing paths quickly

## 14. The `A` Command

`A joint1 joint2` moves directly in angle space.

Example:

```text
A 90 90
```

This bypasses Cartesian planning and simply places the servos at the requested angles.

The sketch then reports:

- absolute Cartesian position
- user-relative Cartesian position

This is very useful for:

- calibration
- understanding how angle space maps to XY space
- debugging axis direction

## 15. The `C` Command

The sketch supports a native circle command:

```text
C centerX centerY radius [segments]
```

Examples:

```text
C 0 0 10
C 0 0 10 36
```

Meaning:

- center at `(0, 0)` in user coordinates
- radius `10 mm`
- optional segment count

The circle is approximated by many short straight lines.

It works by:

1. moving to the start of the circle
2. putting the pen down
3. stepping through points around the circle
4. using `executeLinearG(...)` between those points
5. lifting the pen again

## 16. Pen Control

The pen lift is controlled as simple servo-angle states:

- pen up
- pen down

Default values are stored in `Calibration`.

Commands:

```text
PEN UP
PEN DOWN
Z 90
```

`PEN UP` and `PEN DOWN` use the calibrated defaults.

`Z angle` directly sets the pen lift servo.

If your hardware is inverted, you can change it live:

```text
CAL SET PEN_UP 110
CAL SET PEN_DOWN 70
```

## 17. Calibration Commands

You can adjust servo calibration over serial.

Examples:

```text
CAL GET
CAL SET J1_OFFSET 90
CAL SET J2_OFFSET 90
CAL SET J1_INVERT 1
CAL SET J2_INVERT 0
CAL SET PEN_UP 110
CAL SET PEN_DOWN 70
```

This helps match the math model to the real physical assembly.

## 18. Startup Square

There is an optional startup self-test square.

Constants:

- `STARTUP_SQUARE_SIZE_MM = 20.0`
- `RUN_STARTUP_SQUARE = false`

If you set `RUN_STARTUP_SQUARE = true`, the sketch will:

1. move to the bottom-left of a 20 mm square around the center
2. lower the pen
3. draw the square
4. lift the pen
5. return to center

Important:

- this is `20 mm`, not `20 cm`
- a 20 cm square is far beyond the reach of this arm

## 19. Why There Is Also A C Test Program

The Arduino environment is harder to test quickly.

So the project also includes [ik_test.c](/home/loopk/Arduino/InvControl/ik_test.c), which:

- uses the same arm geometry
- uses the same IK approach
- verifies line motion behavior
- checks axis conventions
- checks reachability cases

This is extremely useful because it lets you catch kinematics bugs before uploading to hardware.

## 20. Important Tests Already Covered

The C test program checks things like:

- forward kinematics for known angle pairs
- reachable and unreachable IK targets
- vertical line motion staying vertical
- horizontal line motion staying horizontal
- nonzero-start moves like `(-10, -10)` to `(10, -10)`
- axis sign conventions after frame rotation

That means the motion system is being tested for path shape, not only endpoint accuracy.

## 21. How To Read The Main Sketch

If you want to study the Arduino file, read it in this order:

1. constants and structs
2. coordinate conversion functions
3. forward kinematics
4. analytic IK
5. Gauss-Newton IK
6. line-path precheck
7. `executeLinearG(...)`
8. serial parser in `handleCommand(...)`

That order matches the logic flow of the robot.

## 22. Example Serial Session

Move home:

```text
HOME
```

Move in a square:

```text
PEN DOWN
G 0 0 G 0 10 G 10 10 G 10 0 G 0 0
PEN UP
```

Move directly by angle:

```text
A 90 90
```

Draw a circle:

```text
C 0 0 10 24
```

Check status:

```text
STATUS
```

## 23. Common Beginner Questions

### Why does the arm move in curves when I test angles directly?

Because `A` moves in joint space, not Cartesian space.

### Why does `G` look smoother?

Because it computes many tiny Cartesian points and solves IK for each one.

### Why can a point still be unreachable even if it looks “close”?

Because the robot is limited by:

- link lengths
- joint limits
- current workspace orientation

### Why not use only analytic IK?

Because continuous motion near limits or singular regions behaves better when the analytic solution is used as a seed for a numerical refinement step.

## 24. What To Modify If You Change The Hardware

If you rebuild the arm, the first things to change are:

- `LINK1`
- `LINK2`
- servo offsets and inversion
- pen up/down angles
- centered start pose

If those are wrong, the rest of the math will not match the hardware.

## 25. Final Mental Model

The easiest way to understand the whole system is this:

- `A` command: “put servos here”
- `forwardKinematics`: “where did that put the pen?”
- `G` command: “I want the pen here”
- `inverseKinematics`: “what servo angles will place it there?”
- line planner: “do that over 100 tiny steps so the pen moves in a straight line”

That is the whole robot.
