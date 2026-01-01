package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;

import java.util.List;
import java.util.concurrent.TimeUnit;
public class WebcamSubsystem {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection lastDetection;

    private final boolean useWebcam = true;
    private boolean manualExposureEnabled = false;
    private int exposureMS = 5;
    private int gain = 250;

    // Auto-align PID gains
    private double forwardGain = 0.02; // forward/backward
    private double strafeGain = 0.015; // left/right
    private double turnGain = 0.0431;    // rotation

    private double maxForward = 0.5;
    private double maxStrafe = 0.5;
    private double maxTurn = 0.63;

    private double desiredDistance = 12.0; // inches

    public void init(HardwareMap hardwareMap, String camName) {

        // Initialize AprilTag processor
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        // Initialize vision portal
        if (useWebcam) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, camName))
                    .addProcessor(aprilTag)
                    .build();
        }

        if (manualExposureEnabled) {
            setManualExposure(exposureMS, gain);
        }
    }

    public void enableManualExposure(int exposureMS, int gain) {
        this.manualExposureEnabled = true;
        this.exposureMS = exposureMS;
        this.gain = gain;
    }

    public void update() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections != null && !detections.isEmpty()) {
            lastDetection = detections.get(0);
        } else {
            lastDetection = null;
        }
    }

    public AprilTagDetection getTag() {
        return lastDetection;
    }

    public boolean hasTag() {
        return lastDetection != null;
    }

    public int getTagID() { return hasTag() ? lastDetection.id : -1; }
    public double getTagRange() { return hasTag() ? lastDetection.ftcPose.range : -1; }
    public double getTagBearing() { return hasTag() ? lastDetection.ftcPose.bearing : 0; }
    public double getTagYaw() { return hasTag() ? lastDetection.ftcPose.yaw : 0; }

    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return;

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            try { Thread.sleep(20); } catch (InterruptedException ignored) {}
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            try { Thread.sleep(50); } catch (InterruptedException ignored) {}
        }
        exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        try { Thread.sleep(20); } catch (InterruptedException ignored) {}
    }

    /**
     * Compute x/y/turn speeds for auto-aligning to the last detected tag.
     * @return array [xSpeed, ySpeed, turnSpeed] for Drive.drive()
     */
    public double[] getAutoAlignSpeeds() {
        if (!hasTag()) return new double[]{0, 0, 0};

        double rangeError = getTagRange() - desiredDistance; // forward/back
        double bearingError = getTagBearing();               // rotation
        double yawError = getTagYaw();                       // strafing

        double xSpeed = Range.clip(rangeError * forwardGain, -maxForward, maxForward);
        double ySpeed = Range.clip(-yawError * strafeGain, -maxStrafe, maxStrafe);
        double turnSpeed = Range.clip(bearingError * turnGain, -maxTurn, maxTurn);

        return new double[]{xSpeed, ySpeed, turnSpeed};
    }

    public double[] getTurnSpeeds(){

        double bearingError = getTagBearing();               // rotation

        //just get turn speed to align with tag
        double turnSpeed = Range.clip(bearingError * turnGain, -maxTurn, maxTurn);
        return new double[]{0,0, turnSpeed};
    }



    // Optional: setters to tune gains and limits
    public void setForwardGain(double gain) { forwardGain = gain; }
    public void setStrafeGain(double gain) { strafeGain = gain; }
    public void setTurnGain(double gain) { turnGain = gain; }

    public void setMaxForward(double max) { maxForward = max; }
    public void setMaxStrafe(double max) { maxStrafe = max; }
    public void setMaxTurn(double max) { maxTurn = max; }

    public void setDesiredDistance(double inches) { desiredDistance = inches; }
}