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
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final boolean atHQ = true;
    public static final double stickDeadband = 0.1;
    public static final double driveStickSensitivity = 1.00; 
    public static final double turnStickSensitivity = 1.00;
    public static final double aimingOverride = 0.001;

    // The robot knows who it is, because it knows who it isn't
    public static final String latchSerial = "0327B9A2";
    public static final boolean isRocky = !RobotController.getSerialNumber().toString().matches(latchSerial);

    // Elastic Notifications
    public static final int warningTime = 4000;
    public static final int errorTime = 7000;

    public static final class Swerve {
        public static final String swerveCanBus = "lynk";

        // Multipliers when speed limit is in effect;
        public static final double speedLimitRot = 0.50;

        public static final COTSTalonFXSwerveConstants chosenModule =  
            COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2_5);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(isRocky ? 21.75 : 21.75);
        /* Center to Center distance of left and right modules in meters. */
        public static final double wheelBase = Units.inchesToMeters(isRocky ? 24.75 : 15.75);
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
        public static final double maxSpeed = isRocky ? 5.0292 : 5.21208; 
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
        public static final double maxAngleError = 2.0; // Degrees before we alert that the module is not aligned

        /* Front Left Module - Module 0 */
        public static final class Mod0 { 
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 0;
            public static final String canBusID = swerveCanBus;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(isRocky ? 38.9 : 33.9);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, canBusID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int driveMotorID = 18;
            public static final int angleMotorID = 19;
            public static final int canCoderID = 1;
            public static final String canBusID = swerveCanBus;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(isRocky ? -31.46 : -72.2);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, canBusID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 2;
            public static final String canBusID = swerveCanBus;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(isRocky ? -125.76 : 162.6);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, canBusID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 3;
            public static final String canBusID = swerveCanBus;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(isRocky ? 73.3 : -25.4);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, canBusID, angleOffset);
        }
    }

    public static final class Pose {
        public static final int pigeonID = 1;

        public static final PIDController rotationPID = new PIDController(0.0070, 0.000, 0.0); // kI was 0.050 for NCCMP 2024
        public static final double rotationKS = 0.015;
        public static final double rotationIZone = 2.5; // degrees

        public static final double tiltWarning = 5.0;
        public static final double tiltError = 10.0;

        public static final double fieldWidth = Units.inchesToMeters(26*12 + 5);
        public static final double fieldLength = Units.inchesToMeters(57*12 + 6.875);

        public static enum ReefFace {
            AB,
            CD,
            EF,
            GH,
            IJ,
            KL
        }

        public static final double reefElevatorZoneRadius = Units.inchesToMeters(48.0);
        public static final double wingLength = Units.inchesToMeters(280);

        // Locations from the Blue Alliance perspective
        public static final Translation2d reefCenter = new Translation2d(Units.inchesToMeters(176.75), fieldWidth / 2.0);
    }

    public static final class Vision {
        public static final String cameraName = "AprilTagCam";
        public static final Transform3d robotToCam = new Transform3d(
            new Translation3d(-0.148, 0.005, 0.325),
            new Rotation3d(Units.degreesToRadians(1.2), Units.degreesToRadians(-30.7), Math.PI)); // As measured by PhotonVision
        public static final double centerToReferenceOffset = -Units.inchesToMeters(27.0/2.0 + 3.0); // Reference point is outside of bumper
    }

    public static final class Elevator {
        /* IDs */
        /* TODO Verify IDs */
        public static final int leftID = 4;
        public static final int rightID = 17;
        /* CANBus */
        public static final String CanBus = "rio";
        /* Motor Config Values */
        public static final double peakForwardVoltage = 3.0; // TODO Raise peak voltage
        public static final double peakReverseVoltage = -1.0; // TODO Raise peak voltage
        public static final InvertedValue motorOutputInverted = InvertedValue.CounterClockwise_Positive;
        public static final NeutralModeValue motorNeutralValue = NeutralModeValue.Brake;

        public static final double positionError = 0.2;
        public static final double slowVoltage = 1.0;

        // NOTE Elevator height is measured from the ground to top of the carriage
        public static final double thickness = 2.0; // Thickness of the elevator (only for Mechanism2d visualization)
        public static final double setback = 9.5; // Distance from front edge of robot (only for Mechanism2d visualization)
        public static final double bellyHeight = 0.755; // Height of the top surface of the belly pan from the ground
        public static final double baseHeight = 12.0 + bellyHeight; // Height of elevator in inches when it is at zero position
        public static final double maxHeight = 72.0 + bellyHeight; // Height that elevator should never exceed
        public static final double endEffectorHeight = 6.0; // Height of end effector "target" above elevator height
        public static final double rotPerInch = 0.704; // Rotations to drive elevator one inch

        // TODO Tune PID / FF
        public static final double RPSperVolt = 7.9; // RPS increase with every volt
        public static final double kP = 0.8; // output per unit of error in position (output/rotation)
        public static final double kI = 0.0; // output per unit of integrated error in position (output/(rotation*s))
        public static final double kD = 0.0; // output per unit of error in velocity (output/rps)
        public static final double kS = 0.0; // output to overcome static friction (output)
        public static final double kV = 1.0 / RPSperVolt; // output per unit of target velocity (output/rps)
        public static final double kA = 0.0; // output per unit of target acceleration (output/(rps/s))
        public static final double kG = 0.4; // output to overcome gravity
        public static final double cruiseVelocity = 20.0; // RPS
        public static final double acceleration = cruiseVelocity * 0.25; // Accelerate in 0.25 seconds
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