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

@Autonomous(name = "BlueTwoRow", group = "Autonomous")
@Configurable // Panels
public class BlueTwoRows extends OpMode {

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

    private  int shootRPM = 1200;



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
        follower.setStartingPose(new Pose(33, 134.5, Math.toRadians(90)));

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
        intake.setPower(.76);
         artifactWasShot = true;
        }
        else{
            intake.setPower(0);
            if(artifactWasShot == true){
                shotCount++;
                artifactWasShot = false;
                telemetry.addData("Shot Count", shotCount);
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

        public PathChain StartToShoot;

        public PathChain LineUpIntake;
        public PathChain ShootToIntake;
        public PathChain IntakeBackToShoot;

        public PathChain ShootToRow2;

        public PathChain BackToShootFinal;


        public PathChain ChillOffLine;

        public Paths(Follower follower) {
            StartToShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(33.000, 134.5), new Pose(59.500, 84.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                    .build();

            LineUpIntake = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(59.500, 84.500),new Pose(52.5, 70.5))

                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();


            ShootToIntake = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(52.500, 70.5), new Pose(11.5, 70.5))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            IntakeBackToShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(11.5, 70.5), new Pose(59.500, 84.500))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(135))
                    .build();

            ShootToRow2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(59.500, 84.5), new Pose(55.5, 70.5 - 36))

                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .addPath(
                            new BezierLine(new Pose(55.5, 70.5 - 36), new Pose(11.5, 70.5-34))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            BackToShootFinal = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(11.5, 70.5-34),
                                    new Pose(59.500, 84.500)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(135))
                    .build();



            ChillOffLine = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(59.500, 84.500),
                                    new Pose(60, 120))
                            )
                    .setConstantHeadingInterpolation(Math.toRadians(135))
                    .build();


        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine



        switch (pathState) {
            case 0:
                // Drive to 1st shot
                follower.followPath(paths.StartToShoot);
                nextState();
                break;

            case 1:
                // While driving to shoot:
                // Prime the Motors
                shooter.setVelocity(shootRPM);  // preload shooter
                intake.setPower(0);

                if (!follower.isBusy()) nextState();
                break;

            case 2:
                // Before Next Path
                // Shoot for at least 6 seconds
                //Then start Path to intake row 1

                shootAndThenIntake(shootRPM, 21);

                if(getStateTime() > 4.5){

                    intake.setPower(.7);

                }

                if(getStateTime() > 5.5 ){
                    shooter.setVelocity(-800);
                    canMove = false;
                    intake.setPower(.7);
                    follower.followPath(paths.LineUpIntake);
                    nextState();

                }

                break;

            case 3:
                //While going to Line
                //Rev up intake and backlash on flywheel
                intake.setPower(0.67);
                shooter.setVelocity(-1200);

                if (!follower.isBusy()) nextState();
                break;

            case 4:
                //Immediatly go to next path to better intake
                shooter.setVelocity(-1200);

                intake.setPower(.69);
                follower.setMaxPower(0.33);
                follower.followPath(paths.ShootToIntake);
                nextState();
                break;


            case 5:
                // While doing intake line
                // rev intake Motors

                    intake.setPower(0.7); // example timed action

                if (!follower.isBusy()) nextState();
                break;

            case 6:
                // Turn off all motors
                //start back to shoot paht
                intake.setPower(0);
                follower.setMaxPower(.85);
                follower.followPath(paths.IntakeBackToShoot);
                nextState();
                break;

            case 7:
                // on the way to the shoot spot
                //prerun shoot motors
                if (!follower.isBusy()) nextState();
                //pre run shooter
                shooter.setVelocity(shootRPM);
                break;

            case 8:
                //6 seconds to shoot then start path to row2

                shootAndThenIntake(shootRPM, 21);

                if(getStateTime() > 4){

                    intake.setPower(.7);

                }

                if(getStateTime() > 5) {
                    shooter.setVelocity(0);
                    intake.setPower(0.7);
                    follower.setMaxPower(0.65);
                    follower.followPath(paths.ShootToRow2);
                    nextState();
                }



                break;

            case 9:
                //rwhile running row 2
                //motors on
                intake.setPower(0.69);
                shooter.setVelocity(-1200);

                if (!follower.isBusy()) nextState();
                break;

                case 10:
                    //Start back to shoot
                    //run motors

                    intake.setPower(0);
                follower.setMaxPower(0.85);
                follower.followPath(paths.BackToShootFinal);
                nextState();
                break;

                case 11:
                    //While going back to shoot
                    if (!follower.isBusy()) nextState();
                    //pre run shooter
                    shooter.setVelocity(shootRPM);
                    break;

                    case 12:
                        // Shoot in 6 seconds
                        //then start leave line
                        shootAndThenIntake(shootRPM, 21);

                        if(getStateTime() > 3){

                            intake.setPower(.7);

                        }

                        if(getStateTime() > 4.5) {
                            shooter.setVelocity(0);
                            intake.setPower(0);
                            follower.setMaxPower(0.85);
                            follower.followPath(paths.ChillOffLine);
                            nextState();
                        }



                        break;






        }

        return pathState;
    }
}
