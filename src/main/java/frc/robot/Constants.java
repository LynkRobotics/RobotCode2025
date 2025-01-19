package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final boolean atHQ = true;
    public static final double stickDeadband = 0.1;
    public static final double driveStickSensitivity = 1.00; 
    public static final double turnStickSensitivity = 1.00;
    public static final double aimingOverride = 0.001;

    public static final class Swerve {
        public static final String swerveCanBus = "lynk";

        // Multipliers when speed limit is in effect;
        public static final double speedLimitRot = 0.50;

        public static final COTSTalonFXSwerveConstants chosenModule =  
            COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2_5);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.75); 
        /* Center to Center distance of left and right modules in meters. */
        public static final double wheelBase = Units.inchesToMeters(15.75); 
        /* Center to Center distance of front and rear module wheels in meters. */
        public static final double wheelCircumference = chosenModule.wheelCircumference * 0.97845; // testing

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 40; 
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.10; //TODO: This must be tuned to specific robot
        /* After completeing characterization and inserting 
         * the KS, KV, and KA values into the code, tune the 
         * drive motor kP until it doesn't overshoot and 
         * doesnt oscilate around a target velocity. */
        public static final double driveKI = 0.0; //Leave driveKI at 0.0
        public static final double driveKD = 0.0; //Leave driveKD at 0.0
        public static final double driveKF = 0.0; //Leave driveKF at 0.0 

        /* Drive Motor Characterization Values From SYSID */ 
        public static final double driveKS = 0.32; 
        public static final double driveKV = 1.51; 
        public static final double driveKA = 0.27;  

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 5.21208; //L2.5 therotical max speed, test later.
        /* These are theorectial values to start with, tune after
         * Kraken FOC (L1.0): ft/s = 12.4 | m/s = 3.77952
         * Kraken FOC (L1.5): ft/s = 14.2 | m/s = 4.32816
         * Kraken FOC (L2.0): ft/s = 15.0 | m/s = 4.572
         * Kraken FOC (L2.5): ft/s = 17.1 | m/s = 5.21208
         * Kraken FOC (L3.0): ft/s = 16.5 | m/s = 5.0292
         * Kraken FOC (L3.5): ft/s = 18.9 | m/s = 5.76072
         */
        /** Radians per Second */
        // public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot
        public static final double driveRadius = Math.hypot(wheelBase, trackWidth) / 2.0;
        public static final double maxAngularVelocity = maxSpeed / driveRadius;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { 
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 0;
            public static final String canBusID = swerveCanBus;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(33.6);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, canBusID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int driveMotorID = 18;
            public static final int angleMotorID = 19;
            public static final int canCoderID = 1;
            public static final String canBusID = swerveCanBus;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-70.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, canBusID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 2;
            public static final String canBusID = swerveCanBus;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(163.4);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, canBusID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 3;
            public static final String canBusID = swerveCanBus;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-25.3);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, canBusID, angleOffset);
        }
    }

    public class Pose {
        public static final int pigeonID = 1;

        public static final Rotation2d blueDumpAngle = new Rotation2d(Units.degreesToRadians(-38.0));
        public static final Rotation2d redDumpAngle = new Rotation2d(Units.degreesToRadians(-142.0));
        public static final double maxDumpError = 1.5; // degrees
        public static final Rotation2d blueSlideAngle = new Rotation2d(Units.degreesToRadians(0.0));
        public static final Rotation2d redSlideAngle = new Rotation2d(Units.degreesToRadians(180.0));
        public static final double maxSlideError = 2.0; // degrees
        public static final double maxShuttleError = 1.5; // degrees
        public static final PIDController rotationPID = new PIDController(0.0070, 0.000, 0.0); // kI was 0.050 for NCCMP
        public static final PIDController maintainPID = new PIDController(0.0040, 0.000, 0.0);
        public static final double rotationKS = 0.015;
        public static final double rotationIZone = 2.5; // degrees

        public static final Translation2d blueSpeakerLocation = new Translation2d(0.0, 5.548);
        public static final Translation2d redSpeakerLocation = new Translation2d(16.579, 5.548);
        public static final Translation2d blueShuttleLocation = new Translation2d(1.25, 6.7);
        public static final Translation2d redShuttleLocation = new Translation2d(15.25, 6.7);
        public static final Translation2d blueFarShuttleLocation = new Translation2d(7.5, 7.0);
        public static final Translation2d redFarShuttleLocation = new Translation2d(9.1, 7.0);
        public static final Translation2d blueAmpLocation = new Translation2d(1.84, 8.2);
        public static final Translation2d redAmpLocation = new Translation2d(14.7, 8.2);
        public static final double fieldLength = 16.54;
        public static final double zoneMiddleStart = 5.3;
        public static final double zoneSpeakerEnd = 5.8;
        public static final double zoneSourceStart = 11.5;
        public static final double zoneMiddleEnd = 12.5;
    }

    public class Intake {
        /* IDs */
        public static final int intakeMotorID = 3;
        /* CANBus */
        public static final String intakeMotorCanBus = "rio";
        /* Motor Speed Values */
        public static final double intakingSpeed = 0.50;
        public static final double ejectingSpeed = -0.50;
        public static final double stoppingSpeed = 0.00;
        /* Motor Config Values */
        public static final double peakForwardVoltage = 12.0;
        public static final double peakReverseVoltage = -12.0;
        public static final InvertedValue motorOutputInverted = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue motorNeutralValue = NeutralModeValue.Brake;
    }

    public class Shooter {
        /* IDs */
        public static final int topShooterID = 4; 
        public static final int bottomShooterID = 17;
        /* CANBus */
        public static final String shooterMotorCanBus = "rio";
        /* Motor Speed Values */
        public static final double idleSpeed = 2500; 
        public static final double intakeSpeed = -800;
        public static final double stopSpeed = 0.00;
        public static final double topSpeed = 6000;
        public static final double maxRPMError = 60.0;
        public static final double maxRPMErrorLong = 30.0;
        public static final double slideShotVelocityErrorMax = 100.0;
        public static final double dumpShotVelocityErrorMax = 60.0;
        public static final double shuttleShotVelocityErrorMax = 75.0;
        public static final double farDistance = Units.inchesToMeters(114.0); // when more precision is required
        /* Motor Config Values */
        public static final double peakForwardVoltage = 12.0;
        public static final double peakReverseVoltage = -12.0;
        public static final InvertedValue motorOutputInverted = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue motorNeutralValue = NeutralModeValue.Brake;
        public static final double kP = 0.25; // Voltage per 1 RPS of error
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.21;  // Voltage to overcome static friction
        public static final double RPMsPerVolt = 490;
        /* Time to complete shot once Note no longer detected */
        public static final double postShotTimeout = 0.1; // in seconds
    }

    public class Index {
        /* IDs */
        public static final int indexMotorID = 13;
        public static final int leftIndexSensorID = 0;
        public static final int rightIndexSensorID = 1;
        /* CANBus */
        public static final String indexMotorCanBus = "rio";
        /* Motor Speed Values */
        public static final double indexSpeed = 0.80;
        public static final double feedSpeed = 1.00;
        public static final double softFeedSpeed = 0.25;
        public static final double ejectSpeed = -1.00;
        public static final double stopSpeed = 0.00;
        /* Timer Values */
        public static final double waitToShootTime = 0.75;
        /* Motor Config Values */
        public static final double peakForwardVoltage = 12.0;
        public static final double peakReverseVoltage = -12.0;
        public static final InvertedValue motorOutputInverted = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue motorNeutralValue = NeutralModeValue.Brake;
    }

    public class Climber {
        /* IDs */
        public static final int rightID = 47;
        public static final int leftID = 48;
        /* CANBus */
        public static final String CanBus = "rio";
        /* Motor Config Values */
        public static final double peakForwardVoltage = 12.0;
        public static final double peakReverseVoltage = -12.0;
        public static final InvertedValue motorOutputInverted = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue motorNeutralValue = NeutralModeValue.Brake;

        public static final double extendedPosition = -230.0;
        // public static final double midPosition = -100.0; //-450.0;  
        public static final double retractedPosition = -15.0;
        public static final double positionError = 1.0;
        public static final double slowVoltage = 1.5;

        public static final double RPSperVolt = 7.9; // RPS increase with every volt
        public static final double kP = 0.50; // output per unit of error in position (output/rotation)
        public static final double kI = 0.0; // output per unit of integrated error in position (output/(rotation*s))
        public static final double kD = 0.0; // output per unit of error in velocity (output/rps)
        public static final double kS = 0.22; // output to overcome static friction (output)
        public static final double kV = 1.0 / RPSperVolt; // output per unit of target velocity (output/rps)
        public static final double kA = 0.0; // output per unit of target acceleration (output/(rps/s))
        public static final double kG = 0.0; // do not factory in gravity
        public static final double cruiseVelocity = 120.0; // RPS
        public static final double acceleration = cruiseVelocity * 0.5; // Accelerate in 0.5 seconds
        
        public static final double timeCutOff = 25.0;
    }

    public static final class Vision {
        public static final String cameraName = "AprilTagCam";
        public static final Transform3d robotToCam = new Transform3d(
            new Translation3d(-0.148, 0.005, 0.325),
            new Rotation3d(Units.degreesToRadians(1.2), Units.degreesToRadians(-30.7), Math.PI)); // As measured by PhotonVision
        public static final double centerToReferenceOffset = -Units.inchesToMeters(27.0/2.0 + 3.0); // Reference point is outside of bumper
        public static final double maxAngleError = 1.0; // degrees
        public static final double calibrationFactorBlue = atHQ ? 0.98 : 1.0;
        public static final double calibrationOffsetBlue = atHQ ? Units.inchesToMeters(0.29) : Units.inchesToMeters(0.0);
        public static final double calibrationFactorRed  = atHQ ? calibrationFactorBlue : 0.98;
        public static final double calibrationOffsetRed  = atHQ ? calibrationOffsetBlue : Units.inchesToMeters(0.0);
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
