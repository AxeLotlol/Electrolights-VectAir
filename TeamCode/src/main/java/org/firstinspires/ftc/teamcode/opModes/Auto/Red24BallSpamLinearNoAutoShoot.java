package org.firstinspires.ftc.teamcode.opModes.Auto;



import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterCalc.calculateShotVectorandUpdateHeading;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Storage;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;


@Autonomous
@Configurable
public class Red24BallSpamLinearNoAutoShoot extends NextFTCOpMode {

    public Red24BallSpamLinearNoAutoShoot() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(hwMap -> Constants.createFollower(hwMap))
        );
    }

    private Follower follower;

    private MotorEx transfer;
    private Timer opmodeTimer;
    private Paths paths;
    public static double gateTime1 = 1;
    public static double gateTime = 1.8;

    public Pose start = new Pose(110.58652658884565, 133.1659559014267, Math.toRadians(270));

    // --- Turret tracking ---
    private ServoEx turret1;
    private ServoEx servoStopper;
    private ServoEx turret2;
    private ServoEx hoodServo;
    private MotorEx intakeMotor;
    double goalY = 144;
    double goalX = 144;
    private static final double MIN_ANGLE = -224.75;
    private static final double MAX_ANGLE =  224.75;
    private static final double TURRET_RANGE =  449.51;
    private double currentTurretPos = 180.0;

    private boolean matchStarted = false;

    private Command intakeMotorOn = new LambdaCommand()
            .setStart(() ->{
                intakeMotor.setPower(1);
                transfer.setPower(1);}
            );
    private Command intakeMotorOff = new LambdaCommand()
            .setStart(() ->{
                        intakeMotor.setPower(0);
                        transfer.setPower(0);
                    }
            );
    public Command servoOpen = new LambdaCommand()
            .setStart(() ->{
                        servoStopper.setPosition(0.16);
                    }
            );
    public Command servoClose = new LambdaCommand()
            .setStart(() ->{
                        servoStopper.setPosition(0.035);
                    }
            );

    public SequentialGroup shoot = new SequentialGroup(
            servoOpen,
            new Delay(0.05),
            intakeMotorOn,
            new Delay(0.3),
            servoClose
            );



    public double getClosestValidTurretAngle(double relativeGoalDegrees) {
        // Option 1: The raw 0-360 input from your vector calculation
        double option1 = relativeGoalDegrees;

        // Option 2: The 360-degree alternative wrap position
        double option2 = (option1 > 180.0) ? (option1 - 360.0) : (option1 + 360.0);

        boolean opt1Valid = (option1 >= MIN_ANGLE && option1 <= MAX_ANGLE);
        boolean opt2Valid = (option2 >= MIN_ANGLE && option2 <= MAX_ANGLE);

        // If both options are mechanically safe, pick the one closest to current position
        if (opt1Valid && opt2Valid) {
            return (Math.abs(option1 - currentTurretPos) < Math.abs(option2 - currentTurretPos)) ? option1 : option2;
        }

        if (opt1Valid) return option1;
        if (opt2Valid) return option2;

        // Safety fallback clamp
        return Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, option1));
    }

    // ----------------------

    public void onInit() {
        follower = PedroComponent.follower();
        follower.setStartingPose(start);
        paths = new Paths(follower);
        opmodeTimer = new Timer();
        intakeMotor = new MotorEx("intakeMotor");
        transfer = new MotorEx("transferMotor");
        turret1  = new ServoEx("turretServo1");
        turret2  = new ServoEx("turretServo2");
        hoodServo = new ServoEx("hoodServo");
        servoStopper = new ServoEx("servoStopper");
        telemetry.addLine("Initialized");
        telemetry.update();
    }

    public Command Auto() {
        return new SequentialGroup(
                new FollowPath(paths.preLoadShoot, true, 1.0),
                shoot,
                intakeMotorOn,
                new FollowPath(paths.spike2, true, 1.0),

                new FollowPath(paths.launchSpike2, true, 1.0),
                shoot,
                intakeMotorOn,
                new FollowPath(paths.gate1, true, 1.0),
                new Delay(gateTime1),
                shoot,
                new FollowPath(paths.spike1, true, 1.0),
                new FollowPath(paths.gate2Intake, true, 1.0),
                new Delay(gateTime),



                new FollowPath(paths.gateShoot, true, 1.0),
                shoot,
                new FollowPath(paths.gateIntake, true, 1.0),
                new Delay(gateTime),


                new FollowPath(paths.gateShoot, true, 1.0),
                shoot,
                new FollowPath(paths.gateIntake, true, 1.0),
                new Delay(gateTime),


                new FollowPath(paths.gateShoot, true, 1.0),
                shoot,
                new FollowPath(paths.gateIntake, true, 1.0),
                new Delay(gateTime),


                new FollowPath(paths.gateShoot, true, 1.0),
                shoot,
                new FollowPath(paths.gateIntake, true, 1.0),
                new Delay(gateTime),


                new FollowPath(paths.gateShoot, true, 1.0),
                shoot,
                intakeMotorOff,
                new FollowPath(paths.park, true, 1.0)
        );
    }

    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        matchStarted = true;
        Auto().schedule();
    }


    @Override
    public void onUpdate() {
        follower.update();
        Storage.currentPose = follower.getPose();

        if (!matchStarted) return;

        Pose currPose = follower.getPose();
        double robotHeading = follower.getPose().getHeading();
        Vector robotToGoalVector = new Vector(follower.getPose().distanceFrom(new Pose(goalX, goalY)), Math.atan2(goalY - currPose.getY(), goalX - currPose.getX()));
        Double[] results = calculateShotVectorandUpdateHeading(robotHeading, robotToGoalVector, follower.getVelocity());
        Double headingError = results[2];
        double flywheelSpeed = results[0];
        shooter((float) flywheelSpeed);
        double hoodAngle = results[1];
        hoodServo.setPosition(hoodAngle);
        double targetTurretAngle = getClosestValidTurretAngle(headingError);
        double servoPositionSignal = 0.05 + ((targetTurretAngle - MIN_ANGLE) / 449.51) * 0.90;
        servoPositionSignal = Math.max(0.05, Math.min(0.95, servoPositionSignal));
        turret1.setPosition(servoPositionSignal);
        turret2.setPosition(servoPositionSignal);
        currentTurretPos=((turret1.getPosition() - 0.05) / 0.90) * 449.51 - 44.75;


    }


    @Override
    public void onStop() {
        Storage.currentPose = follower.getPose();
        follower.breakFollowing();
    }

    public class Paths {
        public PathChain preLoadShoot;
        public PathChain spike2;
        public PathChain launchSpike2;
        public PathChain gate1;
        public PathChain spike1;
        public PathChain gate2Intake;
        public PathChain gateIntake;
        public PathChain gateShoot;
        public PathChain park;

        public Paths(Follower follower) {
            preLoadShoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(110.587, 133.166),
                                    new Pose(97.723, 87.429)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setConstantHeadingInterpolation(Math.toRadians(270))
                    .build();

            spike2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(97.723, 87.429),
                                    new Pose(85.512, 58.943),
                                    new Pose(104.243, 59.193)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(104.243, 59.193),
                                    new Pose(126.044, 59.193)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setTangentHeadingInterpolation()
                    .build();

            launchSpike2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(126.044, 59.193),
                                    new Pose(84.228, 69.272)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            gate1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(84.228, 69.272),
                                    new Pose(98.931, 47.688),
                                    new Pose(135.8, 58.85)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37))
                    .build();



            spike1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(135.8, 58.85),
                                    new Pose(95.715, 67.249),
                                    new Pose(96.535, 82.990)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPoseCallback(new Pose(96.535,82.990),shoot,0.98)

                    .addPath(
                            new BezierLine(
                                    new Pose(96.535, 82.990),
                                    new Pose(119.503, 82.439)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(119.503, 82.439),
                                    new Pose(96.535, 82.439)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setTangentHeadingInterpolation()
                    .setReversed()

                    .build();

            gate2Intake = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(96.535, 82.439),
                                    new Pose(98.931, 47.688),
                                    new Pose(135.8, 58.85)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37))
                    .build();

            gateIntake = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(83.921, 69.318),
                                    new Pose(98.931, 47.688),
                                    new Pose(135.8, 58.85)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37))
                    .build();

            gateShoot = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(135.8, 58.85),
                                    new Pose(95.715, 67.249),
                                    new Pose(83.921, 69.318)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))
                    .build();

            park = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(83.921, 69.318),
                                    new Pose(128.904, 83.589)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
        }
    }
}