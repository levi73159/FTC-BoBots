# Road Runner 1.0 setup

## BIOBUZZ Dashboard field

Deploy the updated APK, open Dashboard, and select `BIOBUZZ Field Map` in the
`quickstart` group to preview the 2026-2027 field without moving hardware.
LocalizationTest and active mecanum/tank trajectory and turn actions use this
background automatically. The image is sourced from FIRST's Event Field Setup
Guide; see `src/main/assets/images/README.md` for provenance.

The display keeps the existing field coordinates (inches, origin at center),
with red at the bottom and blue at the top. It does not change paths, poses,
localizer settings, or implement obstacle avoidance. The source diagram is a
visual reference, not precise coordinates for autonomous targets. Dependency
provided tuning/logger OpModes may still show Dashboard's built-in DECODE map;
new custom telemetry packets can call `BiobuzzField.apply(packet)` to use BIOBUZZ.

Installed following https://rr.brott.dev/docs/v1-0/installation/ using the local
quickstart at commit `6e63a7792e9bb6958798bf46fc03d84765b50c51`.

Dependencies: core/actions 1.0.1, FTC integration 0.1.25, Dashboard 0.5.1.
The complete quickstart drive, messages, optional localizers, and tuning sources
are included. `tuning/TuningOpModes.java` selects `MecanumDrive.class`.

## Robot configuration

The active Road Runner drive uses `left_front`, `left_back`, `right_front`, and
`right_back` as DcMotorEx devices, with the left motors reversed to match TeleOp.
It uses the four drive motor encoders and an IMU named `imu`. Connect all four
drive motor encoder cables. Optional dead-wheel, OTOS, Pinpoint, and tank classes
are included from the quickstart but are not selected.

Before running tuning OpModes, set `logoFacingDirection` and
`usbFacingDirection` in `MecanumDrive.PARAMS` to the actual hub mounting.
The quickstart defaults (logo UP, USB FORWARD) are placeholders, not measured
robot settings. Verify motor/encoder directions with the tuning tools.

## Required robot-side tuning

Follow https://rr.brott.dev/docs/v1-0/tuning/ in order. The tuning OpModes appear
under the `quickstart` group. Connect to the robot network and open FTC Dashboard
at http://192.168.43.1:8080/dash (Control Hub default).

`inPerTick`, `lateralInPerTick`, `trackWidthTicks`, feedforward constants,
motion limits, and controller gains still contain quickstart defaults. Measure
and tune them for this robot; do not run trajectory tests with these defaults.
Save tuned values back into the source; Dashboard edits are temporary.

The existing timed `Auto` and driver TeleOp are unchanged. The old Basic Drive
OpMode's Kotlin class was renamed to `BasicMecanumDrive` to avoid colliding with
Road Runner's Java `MecanumDrive`; its Driver Station name remains Basic Drive.

After tuning, Kotlin autonomous code can construct a drive and run an action:

```kotlin
val drive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
waitForStart()
if (isStopRequested) return
Actions.runBlocking(
    drive.actionBuilder(Pose2d(0.0, 0.0, 0.0))
        .strafeTo(Vector2d(24.0, 0.0))
        .build()
)
```

Use imports `com.acmerobotics.roadrunner.Pose2d`,
`com.acmerobotics.roadrunner.Vector2d`, and
`com.acmerobotics.roadrunner.ftc.Actions`. Distances are inches and angles radians.
