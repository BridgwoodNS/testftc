package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private DcMotor intake;

    double intakeSpeed = 0.85;
    double outakeSpeed = -0.65;


    public void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "Intake");


    }

    public void intake() {
        intake.setPower(intakeSpeed);
    }

    public void outake() {
        intake.setPower(outakeSpeed);
    }

    public void stop() {
        intake.setPower(0);
    }
    public void setPower(double power) {
        intake.setPower(power);
    }





}
