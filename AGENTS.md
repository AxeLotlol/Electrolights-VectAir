# AGENTS.md - FTC Robot Controller Project

## Project Overview
FIRST Tech Challenge (FTC) robot control Android app. Two-module Gradle project:
- `FtcRobotController` - SDK library module (do not modify)
- `TeamCode` - Team's custom opModes, subsystems, vision, pathing code

## Build System
- **Gradle 8.9** (wrapper), **Android Gradle Plugin 8.7.0**
- **Java 8** (source/target compatibility) - required for OnBotJava
- **minSdk 24**, **targetSdk 28**, **compileSdk 34**
- NDK 21.3.6528147

## Key Commands
```bash
./gradlew assembleDebug          # Build debug APK
./gradlew installDebug           # Build and install to connected device
./gradlew clean                  # Clean build artifacts
./gradlew :TeamCode:assembleDebug  # Build only TeamCode module
```

## Project Structure
```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── opModes/           # TeleOp & Autonomous opModes
│   ├── TeleOp/        # Driver-controlled modes (TeleOpBlue, TeleOpRed, TeleOpBlue2, TeleOpRed2)
│   └── Auto/          # Autonomous routines (Red*/Blue* BallSpamLinear, path-based)
├── subsystems/        # Robot hardware abstractions
│   ├── DriveTrain2.java      # Main drivetrain (mecanum, Pinpoint, PedroPathing) - Limelight/Kalman logic here (commented)
│   ├── Flywheel.java         # Shooter flywheel control
│   ├── Storage.java          # Ball storage/intake
│   ├── ShooterCalc.java      # Shot trajectory calculation
│   ├── AutoShooterCalc.java  # Autonomous shooting logic
│   ├── TempHood.java         # Hood angle control
│   ├── LaunchDetector.java   # Launch zone detection
│   ├── Airsort.java          # Air sort mechanism
│   └── ShooterConstants.java # Shooter tuning constants
├── vision/            # Vision processing (classes exist but not wired in TeleOp)
│   ├── Limelight.java        # Limelight 3A wrapper (AprilTag pipeline)
│   ├── KalmanFilter.java     # Sensor fusion (odometry + vision)
│   └── ColorBlobDetector.java
└── pedroPathing/      # Path following constants & tuning
    ├── Constants.java        # Follower, drivetrain, localizer config
    └── Tuning.java           # TeleOp tuning OpModes (localization, PIDF, velocity)
```

## Framework & Key Patterns

### NextFTC Framework (`dev.nextftc.*`)
- OpModes extend `NextFTCOpMode` (not `LinearOpMode`)
- Components registered in constructor via `addComponents()`:
  - `PedroComponent(Constants::createFollower)` - path follower
  - `SubsystemComponent(Subsystem.INSTANCE)` - hardware subsystems
  - `BulkReadComponent.INSTANCE` - bulk reads for performance
  - `BindingsComponent.INSTANCE` - gamepad bindings
- Lifecycle methods: `onInit()`, `onUpdate()`, `onStartButtonPressed()`, `onStop()`
- Gamepad bindings via `dev.nextftc.ftc.Gamepads` fluent API

### Hardware Abstraction
- Custom classes: `MotorEx`, `ServoEx`, `IMUEx` (from `dev.nextftc.hardware.impl`)
- Use `.brakeMode()` on motors for TeleOp
- Access via `hardwareMap.get()` or constructor injection
- All hardware init in `onInit()` (I2C lazy init)

### PedroPathing (Path Following)
- Configuration in `pedroPathing/Constants.java`:
  - `FollowerConstants` - mass, PIDF, predictive braking, zero power accel
  - `MecanumConstants` - motor names, directions, velocity scaling
  - `PinpointConstants` - GoBILDA Pinpoint odometry pod offsets
  - `PathConstraints` - velocity, acceleration, jerk limits
- `Constants.createFollower(hardwareMap)` builds the Follower
- Tuning via `pedroPathing/Tuning.java` TeleOp (selectable OpModes)

### Vision System (Limelight mount not ready - logic commented in DriveTrain2)
- **Limelight.java**: Wrapper for `Limelight3A`, pipeline switching, pose/distance extraction
- **KalmanFilter.java**: 2D position fusion (odometry prediction + vision correction)
  - `predict(Pose odomPose)` - call every loop
  - `correct(Pose visionPose, double distance)` - call when `limelight.canSeeTarget()`
  - `getFusedPose(heading)` - fused position estimate
- **Currently**: Logic implemented in `DriveTrain2.java` (commented out) - initialize in `initialize()`, update in `periodic()`, toggle via gamepad1 dpadLeft
- TeleOpRed2/TeleOpBlue2 no longer contain vision code

### OpMode Conventions
- Annotations: `@TeleOp(name="...", group="...")` or `@Autonomous` / `@Configurable`
- Alliance-specific: `TeleOpRed*` / `TeleOpBlue*` (static `isRed()`/`isBlue()`)
- Autonomous: Linear OpModes using PedroPathing `PathChain` + commands
- Paths built via `follower.pathBuilder().addPath(...).build()`

## Dependencies (build.dependencies.gradle)
- FTC SDK 11.0.0 (Inspection, Blocks, RobotCore, RobotServer, OnBotJava, Hardware, FtcCommon, Vision)
- PedroPathing 2.1.1 (path planning)
- NextFTC: ftc, hardware, control, pedro extensions
- FullPanels 1.0.9 (dashboard UI via `com.bylazar.configurables`)

## Build Config Notes
- Common config in `build.common.gradle` - **avoid editing**
- TeamCode customizations go in `TeamCode/build.gradle`
- Debug keystore: `libs/ftc.debug.keystore` (password: `android`)
- Version code/name synced from `FtcRobotController/src/main/AndroidManifest.xml`

## Important Constraints
- **Do not modify** `build.common.gradle` or `FtcRobotController/build.gradle` - managed by FTC SDK updates
- Java 8 only (OnBotJava compatibility)
- All hardware init must happen in OpMode `init()` phase (I2C lazy init)
- Use `hardwareMap.get()` for device access
- `@Configurable` fields are live-tunable via FullPanels dashboard

## Key Files to Know
- `pedroPathing/Constants.java` - ALL drivetrain/localization/pathing constants
- `pedroPathing/Tuning.java` - run "Tuning" TeleOp for localization/PIDF tuning
- `subsystems/DriveTrain2.java` - main drivetrain, turret, Pinpoint odometry, **Limelight/Kalman (commented)**
- `subsystems/ShooterCalc.java` - shot vector calculation (distance → flywheel/hood)
- `vision/Limelight.java` - Limelight 3A integration
- `vision/KalmanFilter.java` - sensor fusion for pose estimation