package org.firstinspires.ftc.teamcode.opModes.Auto;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DistanceRed;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
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

    // 🔵 Blue start pose (mirrored)
    public Pose start = new Pose(24.6, 126.4, Math.toRadians(131));

    private Boolean preloadspinreal = false;

    /* ---------------- INIT ---------------- */

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

        paths = new Paths(follower);

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        follower.setStartingPose(start);
        follower.update();
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

    Command preloadSpunReal = new LambdaCommand()
            .setStart(() -> preloadspinreal = false);

    /* ---------------- AUTO ---------------- */

    public Command Auto(){
        return new SequentialGroup(
                new FollowPath(paths.PreloadLaunch, true, 1.0),

                intakeMotorOn,
                opentransfer,
                new Delay(1),

                shoot,
                transferOnForIntake,
                preloadSpunReal,

                new FollowPath(paths.intakeSet2, true, 0.9),
                new FollowPath(paths.launchSet2, true, 0.9),

                intakeMotorOn,
                shoot,
                transferOnForIntake,

                new FollowPath(paths.resetHelper, true, 1.0),
                new Delay(0.35),
                new FollowPath(paths.resetIntakeSpam, true, 1.0),
                new Delay(1.3),
                transferOff,

                new FollowPath(paths.launchSpam, true, 0.9),
                intakeMotorOn,
                shoot,

                transferOnForIntake,
                new FollowPath(paths.intakeSet1, true, 1.0),
                new FollowPath(paths.launchSet1, true, 1.0),

                intakeMotorOn,
                shoot,

                transferOnForIntake,
                new FollowPath(paths.intakeSet3, true, 1.0),
                new FollowPath(paths.launchSet3, true, 1.0),

                shoot,
                new FollowPath(paths.teleOpPar, true, 1.0)
        );
    }

    /* ---------------- START / UPDATE ---------------- */

    @Override
    public void onStartButtonPressed() {
        preloadspinreal = true;
        shooter(1085);
        Auto().schedule();
    }

    @Override
    public void onUpdate(){
        follower.update();

        if (preloadspinreal) {
            shooter(1080);
        } else {
            if (DistanceRed.INSTANCE.getDistanceFromTag() != 0) {
                shooter(findTPS(DistanceRed.INSTANCE.getDistanceFromTag()));
            } else {
                shooter(1070);
            }
        }
    }

    @Override
    public void onStop() {
        follower.breakFollowing();
    }

    /* ---------------- PATHS (BLUE MIRROR) ---------------- */

    public class Paths {

        public PathChain PreloadLaunch, intakeSet2, launchSet2,
                resetHelper, resetIntakeSpam, launchSpam,
                intakeSet1, launchSet1, intakeSet3,
                launchSet3, teleOpPar;

        public Paths(Follower follower) {

            PreloadLaunch = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(24.6, 126.4),
                            new Pose(50.226, 95.871)
                    )
            ).setLinearHeadingInterpolation(
                    Math.toRadians(131),
                    Math.toRadians(137)
            ).build();

            intakeSet2 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(50.226, 95.871),
                            new Pose(59.0, 87.0),
                            new Pose(58.817, 66.060),
                            new Pose(19.0, 65.0)
                    )
            ).setTangentHeadingInterpolation().build();

            launchSet2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(19.0, 67.1),
                                    new Pose(56.3, 55.5),
                                    new Pose(50.226, 87.871)
                            )
                    ).setLinearHeadingInterpolation(
                            Math.toRadians(183),
                            Math.toRadians(139)
                    ).addTemporalCallback(0.7, transferOff)
                    .build();

            resetHelper = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(50.226, 87.871),
                            new Pose(58.238, 74.462),
                            new Pose(46.8, 56.2),
                            new Pose(37.7, 62.5),
                            new Pose(19.0, 68.0)
                    )
            ).setTangentHeadingInterpolation().build();

            resetIntakeSpam = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(19.0, 68.5),
                            new Pose(20, 66.0),
                            new Pose(17, 56)
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(127)).build();

            launchSpam = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(17, 56.0),
                                    new Pose(18.0, 58.0),
                                    new Pose(36.6, 56.0),
                                    new Pose(59.226, 90.871)
                            )
                    ).setLinearHeadingInterpolation(
                            Math.toRadians(151),
                            Math.toRadians(143)
                    ).addTemporalCallback(1.0, reverseIntakeForMe)
                    .build();

            intakeSet1 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(61.4, 87.2),
                            new Pose(58.5, 97.3),
                            new Pose(42.5, 86.0),
                            new Pose(25.335, 84.898)
                    )
            ).setTangentHeadingInterpolation().build();

            launchSet1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(25.335, 84.898),
                                    new Pose(42.5, 84.0),
                                    new Pose(50.226, 95.871)
                            )
                    ).setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(141)
                    ).addTemporalCallback(1, transferOff)
                    .build();

            intakeSet3 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(61.4, 87.2),
                            new Pose(56.0, 89.5),
                            new Pose(60.0, 41.5),
                            new Pose(19.5, 41)
                    )
            ).setTangentHeadingInterpolation().build();

            launchSet3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(19.5, 41),
                                    new Pose(44.0, 43.0),
                                    new Pose(50.226, 95.871)
                            )
                    ).setLinearHeadingInterpolation(
                            Math.toRadians(179),
                            Math.toRadians(143)
                    ).addTemporalCallback(0.8, transferOff)
                    .addTemporalCallback(1.5, reverseIntakeForMe)
                    .build();

            teleOpPar = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(61.4, 87.2),
                            new Pose(53.602, 110.0)
                    )
            ).setTangentHeadingInterpolation().build();
        }
    }
}
