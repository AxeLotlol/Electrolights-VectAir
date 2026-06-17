# AGENTS.md - FTC Robot Controller Project

## Project Overview
FIRST Tech Challenge (FTC) robot control Android app. Two-module Gradle project:
- `FtcRobotController` - SDK library module
- `TeamCode` - Team's custom opModes, subsystems, vision, pathing code

## Build System
- **Gradle 8.9** (wrapper), **Android Gradle Plugin 8.7.0**
- **Java 8** (source/target compatibility)
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
│   ├── TeleOp/        # Driver-controlled modes
│   └── Auto/          # Autonomous routines
├── subsystems/        # Robot hardware abstractions (DriveTrain, Flywheel, etc.)
├── vision/            # Vision processing (Limelight, ColorBlobDetector, KalmanFilter)
└── pedroPathing/      # Path following constants & tuning
```

## Dependencies (build.dependencies.gradle)
- FTC SDK 11.0.0 (Inspection, Blocks, RobotCore, RobotServer, OnBotJava, Hardware, FtcCommon, Vision)
- PedroPathing 2.1.1 (path planning)
- NextFTC libraries (ftc, hardware, control, pedro extensions)
- FullPanels 1.0.9 (dashboard UI)

## Build Config Notes
- Common config in `build.common.gradle` - avoid editing
- TeamCode customizations go in `TeamCode/build.gradle`
- Debug keystore: `libs/ftc.debug.keystore` (password: `android`)
- Version code/name synced from `FtcRobotController/src/main/AndroidManifest.xml`

## Development Workflow
1. Edit code in `TeamCode/src/main/java/...`
2. Build with `./gradlew assembleDebug`
3. Deploy to Control Hub via `./gradlew installDebug` or Android Studio
4. OpModes auto-register via `@TeleOp` / `@Autonomous` annotations

## Important Constraints
- **Do not modify** `build.common.gradle` or `FtcRobotController/build.gradle` - they're managed by FTC SDK updates
- Java 8 only (OnBotJava compatibility)
- All hardware init must happen in OpMode `init()` phase (I2C lazy init)
- Use `hardwareMap.get()` for device access