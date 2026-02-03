package org.firstinspires.ftc.teamcode.opModes.Auto;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
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
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DistanceRed;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

@Autonomous
@Configurable
public class Blue15BallSpam extends NextFTCOpMode {

    public Blue15BallSpam(){
        addComponents(
                new SubsystemComponent(Flywheel.INSTANCE, DistanceRed.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(hwMap -> Constants.createFollower(hwMap))
        );
    }

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Paths paths;

    public MotorEx intakeMotor;
    private MotorEx transfer1;
    private ServoEx transfer2;

    public static MotorEx flywheel = new MotorEx("launchingmotor");
    public static MotorEx flywheel2 = new MotorEx("launchingmotor2");

    // 🔵 Blue mirrored start
    public Pose start = new Pose(
            144 - 119.4,
            126.4,
            Math.PI - Math.toRadians(49)
    );

    private static Servo hoodServo1n;
    private static Servo hoodServo2n;
    private static ServoEx hoodServo1 = new ServoEx(() -> hoodServo1n);
    private static ServoEx hoodServo2 = new ServoEx(() -> hoodServo2n);

    private boolean preloadspinreal = false;

    @Override
    public void onInit() {
        follower = PedroComponent.follower();
        new IMUEx("imu", Direction.LEFT, Direction.BACKWARD).zeroed();

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(7);
        limelight.start();

        intakeMotor = new MotorEx("intake");
        transfer1 = new MotorEx("transfer");
        transfer2 = new ServoEx("transferServo1");

        hoodServo1n = hardwareMap.get(Servo.class, "hoodServo1");
        hoodServo2n = hardwareMap.get(Servo.class, "hoodServo2");

        paths = new Paths(follower);

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        follower.setStartingPose(start);
        follower.update();
    }

    /* ---------------- HOOD ---------------- */

    public static void hoodToPos(double pos) {
        if (!Double.isNaN(pos)) {
            new ParallelGroup(
                    new SetPosition(hoodServo1, pos),
                    new SetPosition(hoodServo2, 1 - pos)
            ).schedule();
        }
    }

    /* ---------------- COMMANDS ---------------- */

    private Command intakeMotorOn = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(-1));

    Command transferOn = new LambdaCommand()
            .setStart(() -> transfer1.setPower(-1));

    Command transferOff = new LambdaCommand()
            .setStart(() -> transfer1.setPower(0));

    Command transferOnForIntake = new LambdaCommand()
            .setStart(() -> transfer1.setPower(-1));

    public Command reverseIntakeForMe = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(0.5));

    public Command opentransfer = new LambdaCommand()
            .setStart(() -> transfer2.setPosition(0.3));

    public Command closeTransfer = new LambdaCommand()
            .setStart(() -> transfer2.setPosition(0.7));

    public SequentialGroup shoot = new SequentialGroup(
            opentransfer,
            new Delay(0.15),
            transferOn,
            new Delay(0.97),
            transferOff,
            closeTransfer
    );

    /* ---------------- AUTO ---------------- */

    public Command Auto(){
        return new SequentialGroup(
                new FollowPath(paths.PreloadLaunch,true,1.0),
                intakeMotorOn,
                shoot,
                transferOnForIntake,

                new FollowPath(paths.intakeSet2,true,1.0),
                new FollowPath(paths.launchSet2,true,1.0),

                shoot,
                transferOnForIntake,

                new FollowPath(paths.resetHelper,true,1.0),
                new FollowPath(paths.resetIntakeSpam,true,1.0),

                new FollowPath(paths.launchSpam,true,0.9),
                shoot,

                transferOnForIntake,
                new FollowPath(paths.intakeSet1,true,1.0),
                new FollowPath(paths.launchSet1,true,1.0),

                shoot,

                transferOnForIntake,
                new FollowPath(paths.intakeSet3,true,1.0),
                new FollowPath(paths.launchSet3,true,1.0),

                shoot,
                new FollowPath(paths.teleOpPar,true,1.0)
        );
    }

    @Override
    public void onStartButtonPressed() {
        preloadspinreal = true;
        shooter(1085);
        Auto().schedule();
    }

    @Override
    public void onUpdate(){
        follower.update();

        Pose currPose = follower.getPose();
        Vector robotToGoal = new Vector(
                currPose.distanceFrom(new Pose(144 - 138, 138)),
                Math.atan2(138 - currPose.getY(), (144 - 138) - currPose.getX())
        );

        Double[] results = calculateShotVectorandUpdateHeading(
                currPose.getHeading(),
                robotToGoal,
                follower.getVelocity()
        );

        shooter(results[0].floatValue());
        hoodToPos(results[1]);
    }

    @Override
    public void onStop() {
        follower.breakFollowing();
    }

    /* ---------------- PATHS (FULL MIRROR) ---------------- */

    public class Paths {

        public PathChain PreloadLaunch, intakeSet2, launchSet2,
                resetHelper, resetIntakeSpam, launchSpam,
                intakeSet1, launchSet1, intakeSet3,
                launchSet3, teleOpPar;

        public Paths(Follower follower) {

            PreloadLaunch = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(24.6,126.4),
                            new Pose(50.226,95.871)
                    )
            ).setLinearHeadingInterpolation(
                    Math.toRadians(131),
                    Math.toRadians(139)
            ).build();

            intakeSet2 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(50.226,95.871),
                            new Pose(58.817,86.907),
                            new Pose(58.817,65.967),
                            new Pose(19.0,65.0)
                    )
            ).setTangentHeadingInterpolation().build();

            launchSet2 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(19.0,65.0),
                            new Pose(56.3,55.5),
                            new Pose(50.226,95.871)
                    )
            ).setLinearHeadingInterpolation(
                    Math.toRadians(183),
                    Math.toRadians(139)
            ).addTemporalCallback(1.3,transferOff).build();

            resetHelper = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(50.226,95.871),
                            new Pose(58.221,74.456),
                            new Pose(45.062,60.551),
                            new Pose(40.695,69.294),
                            new Pose(18.0,69.0)
                    )
            ).setTangentHeadingInterpolation().build();

            resetIntakeSpam = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(19.0,68.0),
                            new Pose(17.626,62.813),
                            new Pose(17.0,59.0)
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(131)).build();

            launchSpam = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(18.0,56.5),
                                    new Pose(18.0,58.0),
                                    new Pose(36.6,56.0),
                                    new Pose(57.488,90.001)
                            )
                    ).setLinearHeadingInterpolation(
                            Math.toRadians(133),
                            Math.toRadians(141)
                    ).addTemporalCallback(2.3,reverseIntakeForMe)
                    .addTemporalCallback(1.67,transferOff)
                    .build();

            intakeSet1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(57.488,90.001),
                                    new Pose(55.097,87.056),
                                    new Pose(42.5,86.0),
                                    new Pose(25.335,86.898)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(139),Math.toRadians(180))
                    .build();

            launchSet1 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(25.335,86.898),
                            new Pose(42.5,84.0),
                            new Pose(50.226,95.871)
                    )
            ).setLinearHeadingInterpolation(
                    Math.toRadians(180),
                    Math.toRadians(143)
            ).build();

            intakeSet3 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(50.226,95.871),
                            new Pose(48.720,84.031),
                            new Pose(59.318,44.648),
                            new Pose(19.5,43.0)
                    )
            ).setTangentHeadingInterpolation().build();

            launchSet3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(19.5,43.0),
                                    new Pose(43.997,43.010),
                                    new Pose(50.226,95.871)
                            )
                    ).setLinearHeadingInterpolation(
                            Math.toRadians(179),
                            Math.toRadians(143)
                    ).addTemporalCallback(1.7,transferOff)
                    .build();

            teleOpPar = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(50.226,95.871),
                                    new Pose(56.210,109.420)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();
        }
    }
}
