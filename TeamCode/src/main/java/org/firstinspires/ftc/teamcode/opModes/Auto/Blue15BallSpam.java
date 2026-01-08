package org.firstinspires.ftc.teamcode.opModes.Auto;

import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
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
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Autonomous(name = "Blue Auto 15 Ball Spam Linear", group = "Autonomous")
@Configurable
public class Blue15BallSpam extends NextFTCOpMode {

    public Blue15BallSpam() {
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

    private MotorEx intakeMotor;
    private MotorEx transfer1;
    private ServoEx transfer2;

    public static MotorEx flywheel = new MotorEx("launchingmotor");

    public boolean spinup = true;

    // ================= START POSE (BLUE) =================
    public Pose start = new Pose(
            24.6,
            126.4,
            Math.toRadians(131)
    );

    @Override
    public void onInit() {
        telemetry.addLine("Initializing...");
        telemetry.update();

        follower = PedroComponent.follower();

        new IMUEx("imu", Direction.LEFT, Direction.BACKWARD).zeroed();

        paths = new Paths(follower);

        intakeMotor = new MotorEx("intake");
        transfer1 = new MotorEx("transfer");
        transfer2 = new ServoEx("transferServo1");

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        follower.setStartingPose(start);

        telemetry.addLine("Init complete (BLUE)");
        telemetry.update();
    }

    // ================= COMMANDS =================

    private Command intakeMotorOn = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(-1));

    private Command intakeMotorOff = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(0));

    private Command distance = new LambdaCommand()
            .setStart(() -> findTPS(DistanceRed.INSTANCE.getDistanceFromTag()));

    private Command transferOn = new LambdaCommand()
            .setStart(() -> transfer1.setPower(-0.8));

    private Command transferOff = new LambdaCommand()
            .setStart(() -> transfer1.setPower(0));

    public Command openTransfer = new LambdaCommand()
            .setStart(() -> transfer2.setPosition(0.3));

    public Command closeTransfer = new LambdaCommand()
            .setStart(() -> transfer2.setPosition(0.7));

    public SequentialGroup shoot = new SequentialGroup(
            openTransfer,
            new Delay(0.35),
            transferOn,
            new Delay(1),
            transferOff,
            closeTransfer
    );

    public Command spinupFalse = new LambdaCommand()
            .setStart(() -> spinup = false);

    // ================= AUTO =================

    public Command Auto() {
        return new SequentialGroup(
                new FollowPath(paths.PreloadLaunch, true, 1.0),
                spinupFalse,
                intakeMotorOn,
                openTransfer,
                new Delay(1),
                shoot,
                transferOn,
                new FollowPath(paths.intakeSet2, true, 0.9),
                transferOff,
                new FollowPath(paths.launchSet2, true, 0.9),
                shoot,
                transferOn,
                new FollowPath(paths.resetHelper, true, 1.0),
                new Delay(0.15),
                new FollowPath(paths.resetIntakeSpam, true, 1.0),
                new Delay(1.3),
                transferOff,
                new FollowPath(paths.launchSpam, true, 0.9),
                shoot,
                transferOn,
                new FollowPath(paths.intakeSet1, true, 1.0),
                transferOff,
                new FollowPath(paths.launchSet1, true, 1.0),
                shoot,
                transferOn,
                new FollowPath(paths.intakeSet3, true, 1.0),
                transferOff,
                new FollowPath(paths.launchSet3, true, 1.0),
                shoot,
                new FollowPath(paths.teleOpPar, true, 1.0)
        );
    }

    @Override
    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        Auto().schedule();
    }

    @Override
    public void onUpdate() {
        if (spinup) {
            flywheel.setPower(1);
        } else {
            shooter(1150);
        }
        follower.update();
    }

    @Override
    public void onStop() {
        follower.breakFollowing();
        telemetry.addLine("Auto stopped");
        telemetry.update();
    }

    // ================= PATHS (BLUE) =================

    public static class Paths {

        public Pose preloadStart = new Pose(24.6, 126.4, Math.toRadians(131));
        public Pose preloadEnd = new Pose(61.4, 87.2);

        public Pose i2_1 = new Pose(61.4, 87.2);
        public Pose i2_2 = new Pose(59.0, 87.0);
        public Pose i2_3 = new Pose(60.0, 61.0);
        public Pose i2_4 = new Pose(20.0, 61.0);

        public Pose l2_1 = new Pose(20.0, 59.0);
        public Pose l2_2 = new Pose(56.3, 55.5);
        public Pose l2_3 = new Pose(61.4, 87.2);

        public Pose rh1 = new Pose(61.4, 87.2);
        public Pose rh2 = new Pose(63.6, 86.4);
        public Pose rh3 = new Pose(46.8, 56.2);
        public Pose rh4 = new Pose(37.7, 62.0);
        public Pose rh5 = new Pose(19.0, 68.5);

        public Pose ris1 = new Pose(19.0, 68.5);
        public Pose ris2 = new Pose(17.0, 60.0);

        public Pose ls1 = new Pose(17.0, 60.0);
        public Pose ls2 = new Pose(36.6, 50.0);
        public Pose ls3 = new Pose(61.4, 87.2);

        public Pose i1_1 = new Pose(61.4, 87.2);
        public Pose i1_2 = new Pose(52.5, 97.3);
        public Pose i1_3 = new Pose(42.5, 87.0);
        public Pose i1_4 = new Pose(18.335, 86.898);

        public Pose l1_1 = new Pose(18.335, 86.898);
        public Pose l1_2 = new Pose(42.5, 84.0);
        public Pose l1_3 = new Pose(61.4, 87.2);

        public Pose i3_1 = new Pose(61.4, 87.2);
        public Pose i3_2 = new Pose(56.0, 89.5);
        public Pose i3_3 = new Pose(60.0, 39.5);
        public Pose i3_4 = new Pose(20.0, 40.0);

        public Pose l3_1 = new Pose(20.0, 40.0);
        public Pose l3_2 = new Pose(44.0, 43.0);
        public Pose l3_3 = new Pose(61.4, 87.2);

        public Pose tp1 = new Pose(61.4, 87.2);
        public Pose tp2 = new Pose(36.0, 72.0);

        public PathChain PreloadLaunch, intakeSet2, launchSet2,
                resetHelper, resetIntakeSpam, launchSpam,
                intakeSet1, launchSet1,
                intakeSet3, launchSet3,
                teleOpPar;

        public Paths(Follower follower) {

            PreloadLaunch = follower.pathBuilder()
                    .addPath(new BezierLine(preloadStart, preloadEnd))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            intakeSet2 = follower.pathBuilder()
                    .addPath(new BezierCurve(i2_1, i2_2, i2_3, i2_4))
                    .setTangentHeadingInterpolation()
                    .build();

            launchSet2 = follower.pathBuilder()
                    .addPath(new BezierCurve(l2_1, l2_2, l2_3))
                    .setLinearHeadingInterpolation(Math.toRadians(183), Math.toRadians(139))
                    .build();

            resetHelper = follower.pathBuilder()
                    .addPath(new BezierCurve(rh1, rh2, rh3, rh4, rh5))
                    .setTangentHeadingInterpolation()
                    .build();

            resetIntakeSpam = follower.pathBuilder()
                    .addPath(new BezierLine(ris1, ris2))
                    .setConstantHeadingInterpolation(Math.toRadians(140))
                    .build();

            launchSpam = follower.pathBuilder()
                    .addPath(new BezierCurve(ls1, ls2, ls3))
                    .setLinearHeadingInterpolation(Math.toRadians(151), Math.toRadians(139))
                    .build();

            intakeSet1 = follower.pathBuilder()
                    .addPath(new BezierCurve(i1_1, i1_2, i1_3, i1_4))
                    .setTangentHeadingInterpolation()
                    .build();

            launchSet1 = follower.pathBuilder()
                    .addPath(new BezierCurve(l1_1, l1_2, l1_3))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(139))
                    .build();

            intakeSet3 = follower.pathBuilder()
                    .addPath(new BezierCurve(i3_1, i3_2, i3_3, i3_4))
                    .setTangentHeadingInterpolation()
                    .build();

            launchSet3 = follower.pathBuilder()
                    .addPath(new BezierCurve(l3_1, l3_2, l3_3))
                    .setLinearHeadingInterpolation(Math.toRadians(179), Math.toRadians(139))
                    .build();

            teleOpPar = follower.pathBuilder()
                    .addPath(new BezierLine(tp1, tp2))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();
        }
    }
}
