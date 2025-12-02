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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BlueLeave", group = "Autonomous")
@Configurable // Panels
public class BlueFlushLeaveShootIntakeShoot extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private DcMotor shooter;
    private DcMotor intake;

    // Pedro timer (simple)
    private long stateStartTime;


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        shooter = hardwareMap.get(DcMotor.class, "Shooter");
        intake  = hardwareMap.get(DcMotor.class, "Intake");



        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(32, 135, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        stateStartTime = System.currentTimeMillis();   // start the timer

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
    private double getStateTime() {
        return (System.currentTimeMillis() - stateStartTime) / 1000.0;
    }

    private void nextState() {
        pathState++;
        stateStartTime = System.currentTimeMillis(); // reset timer
    }

    public static class Paths {

        public PathChain StartToShoot;
        public PathChain ShootToIntake;
        public PathChain IntakeBackToShoot;

        public Paths(Follower follower) {
            StartToShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(32.000, 135.000), new Pose(59.500, 84.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                    .setVelocityConstraint(25)
                    .build();

            ShootToIntake = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(59.500, 84.500), new Pose(20.071, 84.756))
                    )
                    .setTangentHeadingInterpolation()
                    .setVelocityConstraint(25)
                    .build();

            IntakeBackToShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.071, 84.756), new Pose(59.500, 84.500))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(135))

                    .setVelocityConstraint(25)
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine



        switch (pathState) {
            case 0:
                // Start path 1
                follower.followPath(paths.StartToShoot);
                nextState();
                break;

            case 1:
                // While driving to shoot:
                // Example motor behavior
                shooter.setPower(.15);  // preload shooter
                intake.setPower(0);

                if (!follower.isBusy()) nextState();
                break;

            case 2:
                // Start path 2
                shooter.setPower(.6);   // stop shooter if needed
                intake.setPower(.5);

                if(getStateTime() > 5){
                    shooter.setPower(0);
                    intake.setPower(.5);
                    follower.followPath(paths.ShootToIntake);
                    nextState();

                }

                break;

            case 3:
                // Running ShootToRow1

                    intake.setPower(0.5); // example timed action

                if (!follower.isBusy()) nextState();
                break;

            case 4:
                // Start path 3
                intake.setPower(0);
                follower.followPath(paths.IntakeBackToShoot);
                nextState();
                break;

            case 5:
                // Running Row1ToShoot
                if (!follower.isBusy()) nextState();
                break;

            case 6:
                // All Done

                if(getStateTime() < 4){
                shooter.setPower(0.2);
                intake.setPower(0.2);}
                else {
                    shooter.setPower(0);
                    intake.setPower(0);
                }
                break;
        }

        return pathState;
    }
}
