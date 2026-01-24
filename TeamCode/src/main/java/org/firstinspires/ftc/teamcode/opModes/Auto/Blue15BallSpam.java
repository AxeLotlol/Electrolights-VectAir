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
import com.qualcomm.robotcore.hardware.CRServo;

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
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.powerable.SetPower;

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

    public MotorEx intakeMotor;
    private MotorEx transfer1;
    private ServoEx transfer2;

    public static MotorEx flywheel = new MotorEx("launchingmotor");
    public static MotorEx flywheel2 = new MotorEx("launchingmotor2");

   // private CRServo hoodServo1n;
    //private CRServo hoodServo2n;

    //private CRServoEx hoodServo1 = new CRServoEx(() -> hoodServo1n);
    //private CRServoEx hoodServo2 = new CRServoEx(() -> hoodServo2n);

    public Pose start = new Pose(24.6, 126.4, Math.toRadians(131));



    /*ParallelGroup HoodRunUp = new ParallelGroup(
            new SetPower(hoodServo1, -1),
            new SetPower(hoodServo2, 1)
    );

    ParallelGroup HoodPowerZero = new ParallelGroup(
            new SetPower(hoodServo1, 0),
            new SetPower(hoodServo2, 0)
    );

    SequentialGroup HoodUp = new SequentialGroup(
            HoodRunUp,
            new Delay(0.18),
            HoodPowerZero
    );
*/


    private Command intakeMotorOn = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(-1));

    Command intakeMotorOff = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(0));

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
            new Delay(0.35),
            transferOn,
            new Delay(0.87),
            transferOff,
            closeTransfer
    );

    private Boolean preloadspinreal = false;

    Command preloadSpunReal = new LambdaCommand()
            .setStart(() -> preloadspinreal = false);



    public Command Auto() {
        return new SequentialGroup(
                new FollowPath(paths.PreloadLaunch, true, 1.0),

                intakeMotorOn,
                opentransfer,
                new Delay(1),

                shoot,
                transferOnForIntake,
                preloadSpunReal,

                new FollowPath(paths.intakeSet2, true, 0.9),
                transferOff,

                new FollowPath(paths.launchSet2, true, 0.9),
                intakeMotorOn,
                shoot,

                transferOnForIntake,
                new FollowPath(paths.resetHelper, true, 1.0),
                new Delay(0.35),

                new FollowPath(paths.resetIntakeSpam, true, 1.0),
                new Delay(1.3),
                transferOff,

                reverseIntakeForMe,
                new FollowPath(paths.launchSpam, true, 0.9),

                intakeMotorOn,
                shoot,
                transferOnForIntake,

                new FollowPath(paths.intakeSet1, true, 1.0),
                transferOff,

                new FollowPath(paths.launchSet1, true, 1.0),
                intakeMotorOn,
                shoot,
                transferOnForIntake,

                new FollowPath(paths.intakeSet3, true, 1.0),
                transferOff,

                new FollowPath(paths.launchSet3, true, 1.0),
                shoot,

                new FollowPath(paths.teleOpPar, true, 1.0)
        );
    }



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

       // hoodServo1n = ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo1");
        //hoodServo2n = ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo2");

        paths = new Paths(follower);
        follower.setStartingPose(start);

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        follower.update();
    }

    @Override
    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();

        preloadspinreal = true;
        shooter(1085);

        Auto().schedule();
    }

    @Override
    public void onUpdate() {
        follower.update();

        if (preloadspinreal) {
            shooter(1095);
        } else {
            if (DistanceRed.INSTANCE.getDistanceFromTag() != 0) {
                shooter(findTPS(DistanceRed.INSTANCE.getDistanceFromTag()));
            } else {
                shooter(1085);
            }
        }
    }

    @Override
    public void onStop() {
        follower.breakFollowing();
    }



    public static class Paths {

        public PathChain PreloadLaunch, intakeSet2, launchSet2, resetHelper,
                resetIntakeSpam, launchSpam, intakeSet1, launchSet1,
                intakeSet3, launchSet3, teleOpPar;

        public Paths(Follower follower) {

            PreloadLaunch = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(24.6, 126.4),
                            new Pose(60.226, 95.871)
                    )
            ).setLinearHeadingInterpolation(
                    Math.toRadians(131),
                    Math.toRadians(41 + 180)
            ).build();

            intakeSet2 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(60.226, 95.871),
                            new Pose(59.0, 87.0),
                            new Pose(60.0, 62.0),
                            new Pose(20.0, 62.0)
                    )
            ).setTangentHeadingInterpolation().build();

            launchSet2 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(20.0, 59.0),
                            new Pose(56.3, 55.5),
                            new Pose(60.226, 90.871)
                    )
            ).setLinearHeadingInterpolation(
                    Math.toRadians(177),
                    Math.toRadians(141)
            ).build();

            resetHelper = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(60.226, 90.871),
                            new Pose(63.6, 86.4),
                            new Pose(46.8, 56.2),
                            new Pose(37.7, 62.0),
                            new Pose(19.0, 68.5)
                    )
            ).setTangentHeadingInterpolation().build();

            resetIntakeSpam = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(19.0, 68.5),
                            new Pose(24.0, 66.0),
                            new Pose(13.1, 56.0)
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(135)).build();

            launchSpam = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(13.1, 56.0),
                            new Pose(36.6, 54.0),
                            new Pose(60.226, 90.871)
                    )
            ).setLinearHeadingInterpolation(
                    Math.toRadians(151),
                    Math.toRadians(143)
            ).build();

            intakeSet1 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(61.4, 87.2),
                            new Pose(58.5, 97.3),
                            new Pose(42.5, 87.0),
                            new Pose(25.335, 86.898)
                    )
            ).setTangentHeadingInterpolation().build();

            launchSet1 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(25.335, 86.898),
                            new Pose(42.5, 84.0),
                            new Pose(60.226, 90.871)
                    )
            ).setLinearHeadingInterpolation(
                    Math.toRadians(180),
                    Math.toRadians(141)
            ).build();

            intakeSet3 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(61.4, 87.2),
                            new Pose(56.0, 89.5),
                            new Pose(60.0, 42.5),
                            new Pose(20.0, 43.0)
                    )
            ).setTangentHeadingInterpolation().build();

            launchSet3 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(20.0, 40.0),
                            new Pose(44.0, 43.0),
                            new Pose(60.226, 90.871)
                    )
            ).setLinearHeadingInterpolation(
                    Math.toRadians(179),
                    Math.toRadians(143)
            ).build();

            teleOpPar = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(61.4, 87.2),
                            new Pose(36.0, 72.0)
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(90)).build();
        }
    }
}
