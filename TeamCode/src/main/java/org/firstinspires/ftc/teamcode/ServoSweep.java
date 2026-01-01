package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="goBILDA Servo Sweep", group="Tutorial")
public class ServoSweep extends LinearOpMode {
    private Servo servo;
    private double position = 0.5; // Start at middle
    private double sweepSpeed = 0.005; // Adjust for sweep speed

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "servo");

        waitForStart();

        while (opModeIsActive()) {
            // Use Gamepad buttons to control sweep direction
            if (gamepad1.dpad_up) {
                position += sweepSpeed;
            } else if (gamepad1.dpad_down) {
                position -= sweepSpeed;
            }

            // Keep position within valid 0.0 to 1.0 range
            position = Range.clip(position, 0.0, 1.0);
            servo.setPosition(position);

            // Telemetry for real-time monitoring
            telemetry.addData("Status", "Running");
            telemetry.addData("Servo Position", "%.3f", position);
            telemetry.addData("Instructions", "DPAD Up/Down to Sweep");
            telemetry.update();
        }
    }
}
