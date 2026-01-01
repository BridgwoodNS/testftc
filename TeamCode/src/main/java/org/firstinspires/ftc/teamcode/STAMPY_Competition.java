package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.WebcamSubsystem;

@TeleOp(name = "STAMPY_Competition", group = "Competition")
@Configurable
public class STAMPY_Competition extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance

    public static double closeShotVelocity =1200;
    public static double farShotVelocity = 1650;

    public static double variableShot = 0;


    Drive drive = new Drive();
    Intake intake = new Intake();
    Shooter shooter = new Shooter();
    WebcamSubsystem webcam = new WebcamSubsystem();

    private boolean gamepadA_wasPressed = false;





    @Override
    public void init() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        drive.init(hardwareMap);
        intake.init(hardwareMap);
        shooter.init(hardwareMap);
        webcam.init(hardwareMap, "Webcam 1");

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

    }

    @Override
    public void loop() {

        //drive
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        double speed = gamepad1.left_bumper ? 0.3:1.0;

        if (gamepad1.back){
            drive.resetHeading();
        }

        webcam.update();

        if ( webcam.hasTag()) {
            telemetry.addData("bearing", webcam.getTagBearing());
            telemetry.addData("range", webcam.getTagRange());
            telemetry.addData("yaw", webcam.getTagYaw());
            telemetry.addData("id", webcam.getTagID());
        }


        if (gamepad1.left_trigger > .35 && webcam.hasTag()) {
            double[] speeds = webcam.getTurnSpeeds();
            drive.drive(speeds[0], speeds[1], -1 *speeds[2], 1.0);
            telemetry.addLine("Auto-turning to tag");
            telemetry.addLine("Bearing error: " + webcam.getTagBearing());
        } else {
            // manual drive
            drive.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, speed);
        }







        // shooter controls for gamepad a
        if (gamepad1.a) {
            shooter.shoot(-1000);
        } else if (gamepadA_wasPressed) { // Released
            shooter.shoot(0);
        }
        gamepadA_wasPressed = gamepad1.a;

        //intake
        if (gamepad1.a) {
            intake.intake();
        } else if (gamepad1.b) {
            intake.outake();
        } else if (gamepad1.dpad_up) {
            intake.setPower(0.75);
        } else {
            intake.stop();
        }

        //shooter
        double shooterSpeed = shooter.getVelocity();
        panelsTelemetry.debug("Shooter Speed", shooterSpeed);
        panelsTelemetry.addData("Shooter Speeed", shooterSpeed);
        double tolerance = 0.035; // 2% tolerance


// Close Manual Shot
        if (gamepad1.x) {
            shooter.shoot(closeShotVelocity);

            // Auto-feed when within 2%
            if (Math.abs(shooterSpeed - closeShotVelocity) / closeShotVelocity <= tolerance) {
                intake.setPower(0.85); // feed ball
            } else {
                intake.stop(); // stop intake until speed reached
            }
        }

// Far Manual Shot
        if (gamepad1.y) {
            shooter.shoot(farShotVelocity);

            // Auto-feed when within 2%
            if (Math.abs(shooterSpeed - farShotVelocity) / farShotVelocity <= tolerance) {
                intake.setPower(0.85); // feed ball
            } else {
                intake.stop(); // stop intake until speed reached
            }
        }

        //Variable Shot
        if (gamepad1.right_trigger > .55 && webcam.hasTag()) {
            panelsTelemetry.debug("Running vairable shot");
            double range = webcam.getTagRange();
            double rpm = 1256 -3.333*range +.08333*range*range;
            shooter.shoot(rpm);



            if(Math.abs(shooterSpeed - shooter.getVariableShootSpeed()) <= tolerance)

                intake.setPower(0.85);
            else
                intake.stop();
        }




        panelsTelemetry.update(telemetry);


    }


}
