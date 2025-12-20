

package org.firstinspires.ftc.teamcode.opModes.Auto;


import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.teamcode.subsystems.DistanceRed;


import org.firstinspires.ftc.teamcode.subsystems.Flywheel;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;


@Autonomous(name = "RedTest", group = "Autonomous")
@Configurable
public class redTest extends NextFTCOpMode {
    public redTest(){
        addComponents(
                new SubsystemComponent(Flywheel.INSTANCE,DistanceRed.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(hwMap -> Constants.createFollower(hwMap))


        );
    }
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;




    private Paths paths;
    public Pose start = new Pose(120,125, Math.toRadians(51));





    public void onInit() {
        telemetry.addLine("Initializing Follower...");





        telemetry.update();
        follower = PedroComponent.follower();


        IMUEx imu = new IMUEx("imu", Direction.LEFT, Direction.BACKWARD).zeroed();


        paths = new Paths(follower);


        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        follower.setStartingPose(start);


        pathState = 0;
        telemetry.addLine("Follower + IMU + Odo Pods initialized successfully!");
        telemetry.addLine("Initialization complete!");
        telemetry.update();
    }



    public Command Auto(){
        return new SequentialGroup(
                new FollowPath(paths.preloadLaunch,true,0.8),
                new FollowPath(paths.intakeRow1,false,0.8),
                new FollowPath(paths.launchRow1,true,0.8),
                new FollowPath(paths.intakeRow2,false,0.7),
                new FollowPath(paths.launchRow2,true,0.9),
                new FollowPath(paths.intakeRow3,false,0.8),
                new FollowPath(paths.launchRow3,false,0.8),
                new FollowPath(paths.leavePoints,true,0.9)




        );
    }

    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        Auto().schedule();
        telemetry.update();









    }

    @Override
    public void onUpdate(){

        shooter(1125);


    }




    @Override
    public void onStop() {
        follower.breakFollowing();
        telemetry.addLine("Autonomous Stopped.");
        telemetry.update();
    }

    public static class Paths {

        public PathChain preloadLaunch;
        public PathChain intakeRow1;
        public PathChain launchRow1;
        public PathChain intakeRow2;
        public PathChain launchRow2;
        public PathChain intakeRow3;
        public PathChain launchRow3;
        public PathChain leavePoints;

        public Paths(Follower follower) {
            preloadLaunch = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(120.000, 125.000),
                                    new Pose(96.000, 96.000),
                                    new Pose(72.000, 72.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            intakeRow1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(72.000, 72.000),
                                    new Pose(86.000, 87.000),
                                    new Pose(125.000, 77.800),
                                    new Pose(131.000, 78.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            launchRow1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(131.000, 78.000),
                                    new Pose(86.000, 86.500),
                                    new Pose(72.000, 72.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            intakeRow2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(72.000, 72.000),
                                    new Pose(86.000, 60.000),
                                    new Pose(130.000, 58.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            launchRow2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(130.000, 58.000),
                                    new Pose(86.000, 86.500),
                                    new Pose(72.000, 72.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            intakeRow3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(72.000, 72.000),
                                    new Pose(90.000, 35.000),
                                    new Pose(130.000, 35.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            launchRow3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(130.000, 35.000),
                                    new Pose(86.000, 86.500),
                                    new Pose(72.000, 72.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            leavePoints = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72.000, 72.000), new Pose(96.000, 50.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .setReversed()
                    .build();
        }
    }
}