package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BlueFarOneRow", group = "Autonomous")
@Configurable // Panels
public class BlueFarSideShoot extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private DcMotorEx shooter;
    private DcMotor intake;

    // Pedro timer (simple)
    private long stateStartTime;

    private int shotCount = 0;
    boolean artifactWasShot = false;
    boolean canMove = false;

    private int farShotRPM = 1575;



    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        intake  = hardwareMap.get(DcMotor.class, "Intake");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(36.5, 0, 0, 14.32)
        );


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(47.25, 9.25, Math.toRadians(90)));

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
    private void shootAndThenIntake(double velocity, double tolerance){
        shooter.setVelocity(velocity);

        if(Math.abs(shooter.getVelocity() - velocity) <= tolerance){
        intake.setPower(.725);
         artifactWasShot = true;
        }
        else{
            intake.setPower(0);
            if(artifactWasShot == true){
                shotCount++;
                artifactWasShot = false;
                telemetry.addData("Shot Count", shotCount);
                panelsTelemetry.debug("shotCounttt", shotCount);
            }
            if(shotCount == 3){
                canMove = true;
            }
        }

    }
    private double getStateTime() {
        return (System.currentTimeMillis() - stateStartTime) / 1000.0;
    }


    public void setShooterPower(double power) {
        shooter.setPower(power);
    }

    public void setIntakePower(double power) {
        intake.setPower(power);
    }
    private void nextState() {
        pathState++;
        stateStartTime = System.currentTimeMillis(); // reset timer
    }

    public static class Paths {

        public PathChain FarSideToShoot;
        public PathChain LineUpIntake;
        public PathChain IntakeRow1;
        public PathChain BackToShoot;
        public PathChain OffLine;

        public Paths(Follower follower) {
            FarSideToShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.250, 9.250), new Pose(58.400, 19.800))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(117.5)) //was 115
                    .build();

            LineUpIntake = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(58.400, 19.800), new Pose(57.000, 48.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(117.5), Math.toRadians(180))
                    .build();

            IntakeRow1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(57.000, 48.000), new Pose(5, 56.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            BackToShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(5.000, 56.000), new Pose(58.400, 19.800))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(117.5))
                    .build();

            OffLine = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(58.400, 19.800), new Pose(60.000, 50.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
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
                follower.setMaxPower(.8);
                follower.followPath(paths.FarSideToShoot);
                nextState();
                break;

            case 1:
                // While driving to shoot:
                // Example motor behavior
                shooter.setVelocity(farShotRPM);  // preload shooter
                intake.setPower(0);

                if (!follower.isBusy()) nextState();
                break;

            case 2:
                // Start path 2
                shootAndThenIntake(farShotRPM, 21);
                telemetry.addData("rpm", shooter.getVelocity());

                if(getStateTime() > 5.5){

                    intake.setPower(.7);

                }

                if(getStateTime() > 7 ){
                    shooter.setVelocity(-800);
                    canMove = false;
                    intake.setPower(.7);

                    follower.followPath(paths.LineUpIntake);
                    nextState();

                }

                break;

            case 3:
                //Running line
                intake.setPower(0.61);
                shooter.setVelocity(-1400);

                if (!follower.isBusy()) nextState();
                break;

            case 4:
                shooter.setVelocity(-1400);

                intake.setPower(.6);
                follower.setMaxPower(0.35);
                follower.followPath(paths.IntakeRow1);
                nextState();
                break;


            case 5:
                // Running ShootToRow1

                    intake.setPower(0.6); // example timed action
                shooter.setVelocity(-1400);

                if (!follower.isBusy()) nextState();
                break;

            case 6:
                // Start path 3
                intake.setPower(0);
                follower.setMaxPower(.7);
                follower.followPath(paths.BackToShoot);
                nextState();
                break;

            case 7:
                // Running Row1ToShoot
                if (!follower.isBusy()) nextState();
                //pre run shooter
                shooter.setVelocity(farShotRPM);
                break;

            case 8:
                // All Done

                shootAndThenIntake(farShotRPM, 21);
                telemetry.addData("rpm", shooter.getVelocity());


                if(getStateTime() > 8){
                    shooter.setVelocity(0);
                    intake.setPower(0);
                    follower.setMaxPower(0.85);
                    follower.followPath(paths.OffLine);
                    nextState();

                }





                break;
        }

        return pathState;
    }
}
