package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Configurable
public class Shooter {

    private DcMotorEx shooter;

    public static double kP = 36.5;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 14.32;

    public static double variableShootSpeed = 1200;


    public void init(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");

        //shooter config
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF)
        );
    }

    public void shootBasedOffAprilTag(double range) {

        //replace with formula
        double velocity = range * range;
        shooter.setPower(velocity);
    }

    public void variableShoot() {


        shooter.setVelocity(variableShootSpeed);
    }

    public void shoot(double speed) {
        shooter.setVelocity(speed);
    }

    public double getVelocity(){
        return shooter.getVelocity();

    }

    public double getVariableShootSpeed(){
        return variableShootSpeed;

    }
}