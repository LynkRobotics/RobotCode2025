package frc.robot.subsystems.vision;

import java.util.EnumMap;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;

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
        FRONT("AprilTagCam", new Transform3d(
            new Translation3d(Units.Inches.of(-3.22), Units.Inches.of(9.443), Units.Inches.of(9.127)),
            new Rotation3d(Units.Degree.of(0), Units.Degree.of(-12.0), Units.Degree.of(156.0)))),
        REAR("Detection Cam", new Transform3d(
            new Translation3d(Units.Inches.of(-0.109), Units.Inches.of(-4.550), Units.Inches.of(39.230)),
			new Rotation3d(Units.Degree.of(0.0), Units.Degree.of(-33.0), Units.Degree.of(0.0))));
    
        public final String name;
        public final Transform3d robotToCamera;
            
        Camera(String name, Transform3d robotToCamera) {
            this.name = name;
            this.robotToCamera = robotToCamera;
        }
    }

    public static final Camera[] camerasAvailable = Camera.values();

    public enum CameraMode {
        DEFAULT(1.0, Double.POSITIVE_INFINITY),
        FRONT(1.0, Double.POSITIVE_INFINITY),
        REAR(1.0, Double.POSITIVE_INFINITY);

        private final EnumMap<Camera, Double> stddev = new EnumMap<>(Camera.class);

        CameraMode(double front, double rear) {
            stddev.put(Camera.FRONT, front);
            stddev.put(Camera.REAR, rear);
        }

        public double getStdDev(Camera camera) {
            return stddev.get(camera);
        }
    }
}