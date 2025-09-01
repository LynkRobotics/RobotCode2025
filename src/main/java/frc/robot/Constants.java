package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.RobotController;

public final class Constants {
    public static final boolean atHQ = true; //TODO: consider a field calibration option to allow NT broadcasting
    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public static final double stickDeadband = 0.1;
    public static final double driveStickSensitivity = 1.00; 
    public static final double turnStickSensitivity = 1.00;
    public static final double aimingOverride = 0.001;
    public static final double algaeScoredTimeout = 1.0; // How long (in seconds) we should prevent auto-aiming after scoring algae
    public static final double maxVisionDiffCoral = Units.inchesToMeters(1.5);
    public static final double algaeSlowRot = 0.6; // Slower rotation when holding algae
    public static final double mechanismSlowdown = 1.0; // Useful to help analyze issues with mechanisms

    public static final int indexSensorID = 7;
    public static final int candiID = 0;
    public static final String candiBus = "rio";

    // The robot knows who it is, because it knows who it isn't
    public static final String latchSerial = "0327B9A2";
    public static final boolean isRocky = !RobotController.getSerialNumber().toString().matches(latchSerial);

    // Elastic Notifications
    public static final int warningTime = 4000;
    public static final int errorTime = 7000;   
}