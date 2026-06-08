package org.firstinspires.ftc.teamcode.opModes.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Storage;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Autonomous
@Configurable
public class Red24BallSpamLinearNithin extends NextFTCOpMode {

    public Red24BallSpamLinearNithin() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(hwMap -> Constants.createFollower(hwMap))
        );
    }

    private Follower follower;
    private Timer opmodeTimer;
    private Paths paths;
    public static double gateTime = 1.32;


    public Pose start = new Pose(110.58652658884565, 133.1659559014267, Math.toRadians(270));

    public void onInit() {
        follower = PedroComponent.follower();
        follower.setStartingPose(start);
        paths = new Paths(follower);
        opmodeTimer = new Timer();
        telemetry.addLine("Initialized");
        telemetry.update();
    }

    public Command Auto() {
        return new SequentialGroup(
                new FollowPath(paths.preLoadShoot, true, 1.0),
                new FollowPath(paths.spike2, true, 1.0),
                new FollowPath(paths.launchSpike2, true, 1.0),
                new FollowPath(paths.gate1, true, 1.0),
                new Delay(gateTime),
                new FollowPath(paths.openForPartner, true, 1.0),

                new FollowPath(paths.spike1, true, 1.0),
                new FollowPath(paths.gate2Intake, true, 1.0),
                new Delay(gateTime),
                new FollowPath(paths.openForPartner, true, 1.0),

                new FollowPath(paths.gateShoot, true, 1.0),
                new FollowPath(paths.gateIntake, true, 1.0),
                new Delay(gateTime),
                new FollowPath(paths.openForPartner, true, 1.0),

                new FollowPath(paths.gateShoot, true, 1.0),
                new FollowPath(paths.gateIntake, true, 1.0),
                new Delay(gateTime),
                new FollowPath(paths.openForPartner, true, 1.0),

                new FollowPath(paths.gateShoot, true, 1.0),
                new FollowPath(paths.gateIntake, true, 1.0),
                new Delay(gateTime),
                new FollowPath(paths.openForPartner, true, 1.0),

                new FollowPath(paths.gateShoot, true, 1.0),
                new FollowPath(paths.park, true, 1.0)
        );
    }

    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        Auto().schedule();
    }

    @Override
    public void onUpdate() {
        follower.update();
        Storage.currentPose = follower.getPose();
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
        public PathChain openForPartner;
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
                                    new Pose(101.121, 49.603),
                                    new Pose(134.754, 60.596)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(20))
                    .build();


            openForPartner = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(134.754, 60.596),
                                    new Pose(131.710, 59.133),
                                    new Pose(131.154, 59.202),
                                    new Pose(130.639, 59.300),
                                    new Pose(130.164, 59.427),
                                    new Pose(129.729, 59.584),
                                    new Pose(129.335, 59.770),
                                    new Pose(128.981, 59.986),
                                    new Pose(128.667, 60.230),
                                    new Pose(128.394, 60.505),
                                    new Pose(128.161, 60.808),
                                    new Pose(127.968, 61.141),
                                    new Pose(127.816, 61.503),
                                    new Pose(127.704, 61.895),
                                    new Pose(131.138, 65.082)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();


            spike1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(131.138, 65.082),
                                    new Pose(95.715, 67.249),
                                    new Pose(96.535, 82.990)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(96.535, 82.990),
                                    new Pose(123.903, 82.439)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(123.903, 82.439),
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
                                    new Pose(101.121, 49.603),
                                    new Pose(134.754, 60.596)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(20))
                    .build();


            gateIntake = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(83.921, 69.318),
                                    new Pose(101.121, 49.603),
                                    new Pose(134.754, 60.596)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(20))
                    .build();


            gateShoot = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(131.138, 65.082),
                                    new Pose(95.715, 67.249),
                                    new Pose(83.921, 69.318)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(0))
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