package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.swerve.SwerveConstants;

public final class Constants {
    public static final boolean atHQ = false; //TODO: consider a field calibration option to allow NT broadcasting
    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public static final double stickDeadband = 0.1;
    public static final double driveStickSensitivity = 1.00; 
    public static final double turnStickSensitivity = 1.00;
    public static final double aimingOverride = 0.001;
    public static final double algaeScoredTimeout = 1.0; // How long (in seconds) we should prevent auto-aiming after scoring algae
    public static final double maxVisionDiffCoral = Units.inchesToMeters(1.5);
    public static final double algaeSlowRot = 0.6; // Slower rotation when holding algae

    public static final int indexSensorID = 7;
    public static final int candiID = 0;
    public static final String candiBus = "rio";

    // The robot knows who it is, because it knows who it isn't
    public static final String latchSerial = "0327B9A2";
    public static final boolean isRocky = !RobotController.getSerialNumber().toString().matches(latchSerial);

    // Elastic Notifications
    public static final int warningTime = 4000;
    public static final int errorTime = 7000;


    
    
    
    public static final class Auto { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        public static final double backUpPushDistance = Units.inchesToMeters(4.0);
        public static final double backUpCSDistance = Units.inchesToMeters(12.0);

        public static final double maxSetupXError = Units.inchesToMeters(4.0);
        public static final double maxSetupYError = Units.inchesToMeters(8.0);
        public static final double maxSetupDegError = 15.0;

        public static final double scoreCoralTimeout = 3.5;
        public static final double scoreCoralTimeLeft = 4.0;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class PathPlanner {
        // TODO Find out why this doesn't work
        public static final RobotConfig robotConfig = new RobotConfig(
            Mass.ofRelativeUnits(isRocky ? 145.0 : 132.0, Pounds),
            MomentOfInertia.ofRelativeUnits(isRocky ? 8.224 : 7.267, KilogramSquareMeters),
            new ModuleConfig(
                SwerveConstants.wheelCircumference / (Math.PI * 2.0),
                SwerveConstants.maxSpeed * 0.95, // Leave a little headroom for inefficiencies
                1.916, // 3847 Spectrum Vex GripLock v2 CoF
                DCMotor.getKrakenX60Foc(1),
                SwerveConstants.chosenModule.driveGearRatio,
                SwerveConstants.driveCurrentLimit,
                1),
            new Translation2d(SwerveConstants.wheelBase / 2.0, SwerveConstants.trackWidth / 2.0),
            new Translation2d(SwerveConstants.wheelBase / 2.0, -SwerveConstants.trackWidth / 2.0),
            new Translation2d(-SwerveConstants.wheelBase / 2.0, SwerveConstants.trackWidth / 2.0),
            new Translation2d(-SwerveConstants.wheelBase / 2.0, -SwerveConstants.trackWidth / 2.0));
    }
}
