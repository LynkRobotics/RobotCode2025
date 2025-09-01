package frc.robot.subsystems.detection;

import java.util.EnumMap;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import frc.robot.subsystems.vision.VisionConstants.Camera;

public class DetectionConstants {
    
    public enum Camera {
        CORAL("Coral", new Transform3d(
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
        CORAL(1.0, Double.POSITIVE_INFINITY);

        private final EnumMap<Camera, Double> stddev = new EnumMap<>(Camera.class);

        CameraMode(double front, double rear){
            stddev.put(Camera.CORAL, front);
        }

        public double getStdDev(Camera camera) {
            return stddev.get(camera);
        }

    }

}
