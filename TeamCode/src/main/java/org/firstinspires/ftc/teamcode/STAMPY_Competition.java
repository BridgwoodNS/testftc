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

    public static double closeShotVelocity =1500;
    public static double farShotVelocity = 3000;


    Drive drive = new Drive();
    Intake intake = new Intake();
    Shooter shooter = new Shooter();
    WebcamSubsystem webcam = new WebcamSubsystem();





    @Override
    public void init() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        drive.init(hardwareMap);
        intake.init(hardwareMap);
        shooter.init(hardwareMap);
        webcam.init(hardwareMap, "Webcam");

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

    }

    @Override
    public void loop() {

        //drive
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        double speed = gamepad1.left_bumper ? 0.4:1.0;

        if (gamepad1.back){
            drive.resetHeading();
        }

        webcam.update();

        if (gamepad1.left_trigger > .35 && webcam.hasTag()) {
            double[] speeds = webcam.getAutoAlignSpeeds();
            drive.drive(speeds[0], speeds[1], speeds[2], 1.0);
            telemetry.addLine("Auto-aligning to tag");
        } else {
            // manual drive
            drive.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, speed);
        }







        //intake
        if (gamepad1.a) {
            intake.intake();
            shooter.shoot(-2000);
        } else if (gamepad1.b) {
            intake.outake();
        } else {
            intake.stop();
        }

        //shooter
        double shooterSpeed = shooter.getVelocity();
        double tolerance = 0.025; // 2% tolerance


// Close Manual Shot
        if (gamepad1.y) {
            shooter.shoot(closeShotVelocity);

            // Auto-feed when within 2%
            if (Math.abs(shooterSpeed - closeShotVelocity) / closeShotVelocity <= tolerance) {
                intake.setPower(0.5); // feed ball
            } else {
                intake.stop(); // stop intake until speed reached
            }
        }

// Far Manual Shot
        if (gamepad1.x) {
            shooter.shoot(farShotVelocity);

            // Auto-feed when within 2%
            if (Math.abs(shooterSpeed - farShotVelocity) / farShotVelocity <= tolerance) {
                intake.setPower(0.5); // feed ball
            } else {
                intake.stop(); // stop intake until speed reached
            }
        }

        //Variable Shot
        if (gamepad1.right_trigger > .35) {
            shooter.variableShoot();

            if(Math.abs(shooterSpeed - shooter.getVariableShootSpeed()) <= tolerance)

                intake.setPower(0.5);
            else
                intake.stop();
        }




        panelsTelemetry.update(telemetry);


    }


}
