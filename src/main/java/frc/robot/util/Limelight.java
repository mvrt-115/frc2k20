package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Add your docs here.
 */
public class Limelight {
    
    private NetworkTable limelight;
    private static final double kLimelightMountAngle = 15;     
    private static final double kLimelightHeight = 29.125;       // Inches    
    private static final double kTargetHeight = 98;  // Inches **** CHANGE VALUE


    public static enum LED_MODE {
        ON, OFF, BLINKING;
    }

    public static enum PIPELINE_STATE {
        VISION_WIDE, DRIVER, VISION_ZOOM;
    }

    public Limelight() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double getHorizontalAngle() {
        return limelight.getEntry("tx").getDouble(0.0);
    }

    public double getVerticalAngle() {
        return limelight.getEntry("ty").getDouble(0.0);
    }

    public double getArea() {
        return limelight.getEntry("ta").getDouble(0.0);
    }

    public boolean hasTarget() {
        return limelight.getEntry("tv").getDouble(0) == 1;
    }

    public void setLED(LED_MODE newMode) {
        switch (newMode) {
        case ON:
            limelight.getEntry("ledMode").setNumber(3);
            break;
        case OFF:
            limelight.getEntry("ledMode").setNumber(1);
            break;
        case BLINKING:
            limelight.getEntry("ledMode").setNumber(2);
            break;
        }
    }

    public void setPipeline(PIPELINE_STATE newPipeline) {
        switch (newPipeline) {
        case VISION_WIDE:
            limelight.getEntry("pipeline").setNumber(0);
            break;
        case VISION_ZOOM:
            limelight.getEntry("pipeline").setNumber(0);
            break;
        case DRIVER:
            limelight.getEntry("pipeline").setNumber(2);
            break;
        }
    }

    public double getDistanceFromTarget(double angle){

        double height = kTargetHeight-kLimelightHeight;
        double correctedAngle = Math.toRadians(kLimelightMountAngle + angle);
        double distance = height/Math.atan(correctedAngle);
        return distance/12;
    }


    public double getRPMFromDistance(double distance){
        return 74.451 * (distance) + 3918.7;
    }

}