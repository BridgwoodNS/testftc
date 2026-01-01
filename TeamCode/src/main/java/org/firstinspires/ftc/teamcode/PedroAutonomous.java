package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain BlueToShoot;
        public PathChain ShootToRow1;
        public PathChain Row1ToShoot;

        public Paths(Follower follower) {
            BlueToShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(72.000, 8.000),
                                    new Pose(37.844, 44.010),
                                    new Pose(112.322, 56.343),
                                    new Pose(72, 94)
                            ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(270))
                    .build();

            ShootToRow1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72.000, 94), new Pose(72, 8))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(90))
                    .build();

            Row1ToShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72, 8), new Pose(72.000, 94))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(270))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine


            switch (pathState) {
                case 0:
                    // Start first path: BlueToShoot

                    follower.followPath(paths.BlueToShoot);
                    pathState++;
                    break;

                case 1:
                    // Wait for BlueToShoot to finish
                    if (!follower.isBusy()) {
                        pathState++;
                    }
                    break;

                case 2:
                    // Start second path: ShootToRow1
                    follower.followPath(paths.ShootToRow1, 0.85, false);
                    pathState++;
                    break;

                case 3:
                    // Wait for ShootToRow1 to finish
                    if (!follower.isBusy()) {
                        pathState++;
                    }
                    break;

                case 4:
                    // Start third path: Row1ToShoot
                    follower.followPath(paths.Row1ToShoot, 0.75, false);
                    pathState++;
                    break;

                case 5:
                    // Wait for Row1ToShoot to finish
                    if (!follower.isBusy()) {
                        pathState++;
                    }
                    break;

                case 6:
                    // All paths done, stop the robot
                    break;
            }

            return pathState;

    }
}
