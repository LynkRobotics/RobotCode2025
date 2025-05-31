package frc.robot.subsystems.vision;

import java.util.EnumMap;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.pose.PoseConstants;

public class VisionConstants {
    public static final double fieldBorderMargin = 0.25; // Reject poses this far outside the field
    public static final double maxZError = 0.5; // Reject poses this far above or below the floor
    public static final double autoAcceptAmbiguity = 0.1; // Automatically accept results with ambiguity less than this
    public static final double maxAmbiguity = 0.35; // Reject results with ambiguity greater than this

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.30; // Meters
    public static double angularStdDevBaseline = 2.0; // Radians

    public enum Camera {
        LEFT  ("AprilTag Left", new Transform3d(
            new Translation3d(PoseConstants.robotFrameLength / 2.0 - Units.inchesToMeters(2.772),
            PoseConstants.robotFrameWidth / 2.0 - Units.inchesToMeters(4.843),
                Units.inchesToMeters(8.46)),
            new Rotation3d(Units.degreesToRadians(1.3), Units.degreesToRadians(-15.5), Units.degreesToRadians(-30)))),
        CENTER("AprilTag Center", new Transform3d(
            new Translation3d(PoseConstants.robotFrameLength / 2.0 - Units.inchesToMeters(6.958),
                0.0,
                Units.inchesToMeters(6.55)),
            new Rotation3d(Units.degreesToRadians(0.9), Units.degreesToRadians(-20.3), Units.degreesToRadians(0)))),
        RIGHT ("AprilTag Right", new Transform3d(
            new Translation3d(PoseConstants.robotFrameLength / 2.0 - Units.inchesToMeters(2.772),
                -PoseConstants.robotFrameWidth / 2.0 + Units.inchesToMeters(4.843),
                Units.inchesToMeters(8.46)),
            new Rotation3d(Units.degreesToRadians(0.8), Units.degreesToRadians(-14.0), Units.degreesToRadians(30)))),
        REAR ("AprilTag Rear", new Transform3d(
            new Translation3d(PoseConstants.robotFrameLength / 2.0 - Units.inchesToMeters(12.94),
                -PoseConstants.robotFrameWidth / 2.0 + Units.inchesToMeters(2.75),
                Units.inchesToMeters(39.6)),
            new Rotation3d(0.0, Units.degreesToRadians(-8.5), Units.degreesToRadians(180.0))));
    
        public final String name;
        public final Transform3d robotToCamera;
            
        Camera(String name, Transform3d robotToCamera) {
            this.name = name;
            this.robotToCamera = robotToCamera;
        }
    }

    public static final Camera[] camerasAvailable = Camera.values();
    // public static final Camera[] camerasAvailable = { Camera.CENTER };

    public enum CameraMode {
        DEFAULT(0.9, 0.75, 0.9, 2.0),
        FRONT(1.0, 0.5, 1.0, Double.POSITIVE_INFINITY),
        REAR(9.0, 9.0, 9.0, 0.01); // TODO Check other values between 0.1 and 0.001 -- changed linearStdDev since

        private final EnumMap<Camera, Double> stddev = new EnumMap<>(Camera.class);

        CameraMode(double left, double center, double right, double rear) {
            stddev.put(Camera.LEFT, left);
            stddev.put(Camera.CENTER, center);
            stddev.put(Camera.RIGHT, right);
            stddev.put(Camera.REAR, rear);
        }

        public double getStdDev(Camera camera) {
            return stddev.get(camera);
        }
    }
}
