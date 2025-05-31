package frc.robot.commands.pidswerve;

import frc.robot.subsystems.swerve.SwerveConstants;

public class PIDSwerveConstants {
    public static final double translationKP = 0.070;
    public static final double roughTranslationKP = 0.10;
    public static final double positionTolerance = 1.0; // inches
    public static final double roughPositionTolerance = 2.5; // inches
    public static final double positionKS = 0.02;
    public static final double positionIZone = 4.0;
    public static final double alignedTimerMax = 0.2;
    public static final double fastAlignedTimerMax = 0.1;
    
    public static final double rotationKP = 0.015; // Small overshoot at 0.015, more noticeable with 0.020, but still functional
    public static final double rotationTolerance = 0.5; // degrees
    public static final double roughRotatationTolerance = 1.5; // degrees
    public static final double maxAngularVelocity = SwerveConstants.maxAngularVelocity / 2.0;
        
    public enum PIDSpeed {
        SLOW(SwerveConstants.maxSpeed / 8.0),
        FAST(SwerveConstants.maxSpeed / 3.0),
        TURBO(SwerveConstants.maxSpeed / 2.0); // TODO Reference
        
        PIDSpeed(double speed) {
            this.speed = speed;
        }

        public double speed;
    }
}
