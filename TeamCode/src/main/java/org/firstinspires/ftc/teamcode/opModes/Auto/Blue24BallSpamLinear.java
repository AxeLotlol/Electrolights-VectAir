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
public class Blue24BallSpamLinear extends NextFTCOpMode {

    public Blue24BallSpamLinear() {
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

    public Pose start = new Pose(33.41347341115435, 133.1659559014267, Math.toRadians(270));

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
                                    new Pose(33.413, 133.166),
                                    new Pose(46.277, 87.429)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setConstantHeadingInterpolation(Math.toRadians(270))
                    .build();

            spike2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(46.277, 87.429),
                                    new Pose(58.488, 58.943),
                                    new Pose(39.757, 59.193)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(39.757, 59.193),
                                    new Pose(17.956, 59.193)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setTangentHeadingInterpolation()
                    .build();

            launchSpike2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(17.956, 59.193),
                                    new Pose(59.772, 69.272)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            gate1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(59.772, 69.272),
                                    new Pose(42.879, 49.603),
                                    new Pose(9.246, 60.596)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(160))
                    .build();

            openForPartner = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(9.246, 60.596),
                                    new Pose(12.290, 59.133),
                                    new Pose(12.846, 59.202),
                                    new Pose(13.361, 59.300),
                                    new Pose(13.836, 59.427),
                                    new Pose(14.271, 59.584),
                                    new Pose(14.665, 59.770),
                                    new Pose(15.019, 59.986),
                                    new Pose(15.333, 60.230),
                                    new Pose(15.606, 60.505),
                                    new Pose(15.839, 60.808),
                                    new Pose(16.032, 61.141),
                                    new Pose(16.184, 61.503),
                                    new Pose(16.296, 61.895),
                                    new Pose(12.862, 65.082)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            spike1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(12.862, 65.082),
                                    new Pose(48.285, 67.249),
                                    new Pose(47.465, 82.990)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(47.465, 82.990),
                                    new Pose(20.097, 82.439)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(20.097, 82.439),
                                    new Pose(47.465, 82.439)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            gate2Intake = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(47.465, 82.439),
                                    new Pose(42.879, 49.603),
                                    new Pose(9.246, 60.596)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(160))
                    .build();

            gateIntake = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(60.079, 69.318),
                                    new Pose(42.879, 49.603),
                                    new Pose(9.246, 60.596)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(160))
                    .build();

            gateShoot = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(12.862, 65.082),
                                    new Pose(48.285, 67.249),
                                    new Pose(60.079, 69.318)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setLinearHeadingInterpolation(Math.toRadians(160),Math.toRadians(180))
                    .build();

            park = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(60.079, 69.318),
                                    new Pose(15.096, 83.589)
                            )
                    )
                    .setTValueConstraint(0.8)
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }
}