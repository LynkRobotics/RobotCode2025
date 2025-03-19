package frc.robot;

import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final boolean atHQ = true; //TODO: consider a field calibration option to allow NT broadcasting
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

    public static final class Swerve {
        public static final String swerveCanBus = "lynk";

        public static final COTSTalonFXSwerveConstants chosenModule =  
            COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

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
        public static final double maxSpeed = isRocky ? 4.572 : 5.21208; 
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
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(isRocky ? -97.6 : 33.9 + 180.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, canBusID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int driveMotorID = 18;
            public static final int angleMotorID = 19;
            public static final int canCoderID = 1;
            public static final String canBusID = swerveCanBus;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(isRocky ? -38.8 : -72.2 + 180.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, canBusID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 2;
            public static final String canBusID = swerveCanBus;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(isRocky ? -159.6 : 162.6 + 180.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, canBusID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 3;
            public static final String canBusID = swerveCanBus;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(isRocky ? 52.9 : -25.4 + 180.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, canBusID, angleOffset);
        }
    }

    public static final class PIDSwerve {
        public static final double translationKP = 0.065;
        public static final double roughTranslationKP = 0.09;
        public static final double positionTolerance = 1.0; // inches
        public static final double roughPositionTolerance = 2.5; // inches
        public static final double maxSpeed = Constants.Swerve.maxSpeed / 3.0;
        public static final double slowSpeed = Constants.Swerve.maxSpeed / 6.0;
        public static final double positionKS = 0.02;
        public static final double positionIZone = 4.0;
    
        public static final double rotationKP = 0.015; // Small overshoot at 0.015, more noticeable with 0.020, but still functional
        public static final double rotationTolerance = 0.5; // degrees
        public static final double roughRotatationTolerance = 1.5; // degrees
        public static final double maxAngularVelocity = Constants.Swerve.maxAngularVelocity / 2.0;    
    }

    public static final class Vision {
        public static final String cameraName = "AprilTagCam";
        public static final Transform3d robotToCam =
            isRocky ?                
                new Transform3d(
                    new Translation3d(Units.inchesToMeters(30.0/2.0 - 6.958), 0.0, Units.inchesToMeters(6.55)),
                    new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(-20.0), 0.0))
            :
                new Transform3d(
                    new Translation3d(0.192, 0.0, 0.325),
                    new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(0.0), 0.0));
    }

    public static final class Elevator {
        /* IDs */
        public static final int leftID = 4;
        public static final int rightID = 17;
        /* CANBus */
        public static final String canBus = "rio";
        /* Motor Config Values */
        public static final double peakForwardVoltage = 14;
        public static final double peakReverseVoltage = -14;
        public static final InvertedValue motorOutputInverted = InvertedValue.CounterClockwise_Positive;
        public static final NeutralModeValue motorNeutralValue = NeutralModeValue.Brake;

        // NOTE Elevator height is measured from the ground to top of the carriage
        public static final double thickness = 2.0; // Thickness of the elevator (only for Mechanism2d visualization)
        public static final double setback = 9.5; // Distance from front edge of robot (only for Mechanism2d visualization)
        public static final double bellyHeight = 0.755; // Height of the top surface of the belly pan from the ground
        public static final double baseHeight = 12.0 + bellyHeight; // Height of elevator in inches when it is at zero position
        public static final double maxHeight = 72.0 + bellyHeight; // Height that elevator should never exceed
        public static final double endEffectorHeight = 6.0; // Height of end effector "target" above elevator height
        public static final double rotPerInch = 0.704; // Rotations to drive elevator one inch

        public static final double safetyMargin = 1.5;   // How many inches away from safe mark to still be considered safe
        public static final double positionError = rotPerInch * 0.5; // Allowable rotation error to be considered in position
        public static final double positionCloseError = rotPerInch * 6.0; // Allowable rotation error to be considered in position
        public static final double stopError = 0.25;      // Allowable inches of error to be considered at a stop
        public static final double slowVoltage = 2.0;    // Volts to move slowly to zero

        public static final double RPSperVolt = 7.9; // RPS increase with every volt
        public static final double kP = 2.2; // output per unit of error in position (output/rotation)
        public static final double kI = 0.0; // output per unit of integrated error in position (output/(rotation*s))
        public static final double kD = 0.0; // output per unit of error in velocity (output/rps)
        public static final double kS = 0.0; // output to overcome static friction (output)
        public static final double kV = 1.0 / RPSperVolt; // output per unit of target velocity (output/rps)
        public static final double kA = 0.0; // output per unit of target acceleration (output/(rps/s))
        public static final double kG = 0.4; // output to overcome gravity
        public static final double cruiseVelocity = 100.0; // RPS
        public static final double acceleration = cruiseVelocity / 0.3; // Accelerate in 0.3 seconds

        public static final double speedLimitAtMax = 0.30;

        public enum Stop {
            // Intake occurs at zero
            SAFE     (Constants.Elevator.baseHeight + 5.0),
            L1       (28.5 - Constants.Elevator.endEffectorHeight),
            L2       (34.5 - Constants.Elevator.endEffectorHeight),
            L2_ALGAE (38.0 - Constants.Elevator.endEffectorHeight),
            L3       (49.5 - Constants.Elevator.endEffectorHeight),
            L3_ALGAE (53.5 - Constants.Elevator.endEffectorHeight),
            ALGAE_RELEASE(63.5 - Constants.Elevator.endEffectorHeight),
            L4       (74.5 - Constants.Elevator.endEffectorHeight),
            L4_SCORE (77.0 - Constants.Elevator.endEffectorHeight);
    
            Stop(double height) {
                this.height = height;
            }
    
            public final double height;
        }

        public static final double standoffBoost = 2.0; // In inches
    }

    public static final class EndEffector {
        /* IDs */
        public static final int motorID = 20;
        /* CANbus */
        public static final String canBus = "rio";
        /* Motor Config Values */
        public static final double peakForwardVoltage = 12.0; 
        public static final double peakReverseVoltage = -12.0; 
        public static final InvertedValue motorOutputInverted = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue motorNeutralValue = NeutralModeValue.Brake;
        /* Motor Control Values */
        public static final double feedVoltage = -3.80;
        public static final double unjamVoltage = 2.00;
        public static final double advanceVoltage = -1.20;
        public static final double scoreVoltage = -3.5;
        public static final double scoreL1Voltage = -2.0; // -2.0 pre-NCASH
        public static final double algaeVoltage = 3.00;
        public static final double algaeHoldVoltage = 1.00;
        public static final double algaeBargeVoltage = -12.00;
        public static final double algaeOutVoltage = -3.00;
        public static final double minimumAlgaeAcquireCurrent = 80.0;
        public static final double minimumAlgaeHoldCurrent = 60.0;

        public static final double L1RunTime = 1.0; // 1.0 pre-NCASH
        public static final double algaeRunTime = 0.5;
    }

    public static final class Index {
        /* IDs */
        public static final int motorID = 13;
        /* CANbus */
        public static final String canBus = "rio";
        /* Motor Config Values */
        public static final double peakForwardVoltage = 12.0; 
        public static final double peakReverseVoltage = -12.0; 
        public static final InvertedValue motorOutputInverted = InvertedValue.CounterClockwise_Positive;
        public static final NeutralModeValue motorNeutralValue = NeutralModeValue.Brake;
        /* Motor Control Values */
        public static final double intakeVoltage = -2.00;
        public static final double unjamVoltage = 1.00;
        public static final double rejectVoltage = 0.50;
        
        public static final double minIntakeVelocity = 12.0;
        public static final int maxStallCount = 15;
        public static final double unjamTime = 0.4;
    }

    public static final class Climber {
        /* IDs */
        public static final int motorID = 46;
        /* CANbus */
        public static final String canBus = "rio";
        /* Motor Config Values */
        public static final double peakForwardVoltage = 12.0; 
        public static final double peakReverseVoltage = -12.0; 
        public static final InvertedValue motorOutputInverted = InvertedValue.CounterClockwise_Positive;
        public static final NeutralModeValue motorNeutralValue = NeutralModeValue.Brake;
        /* Motor Control Values */
        public static final double fastDeployVoltage = 12.0;
        public static final double slowDeployVoltage = 3.0;
        public static final double engageRetractVoltage = -4.0;
        public static final double fastRetractVoltage = -8.0;
        public static final double slowRetractVoltage = -3.0;
        public static final double holdVoltage = -0.5;

        public static final double fastDeployedPosition = 110.0;
        public static final double fullyDeployedPosition = 135.5;
        public static final double engageRetractedPosition = 85.0;
        public static final double fastRetractedPosition = -1.0;
        public static final double fullyRetractedPosition = -30.0;

        public static final int timeCutoff = 30;
    }

    public static final class Pose {
        public static final int pigeonID = 1;

        public static final PIDController rotationPID = new PIDController(0.003, 0.0, 0.0); // kI was 0.050 for NCCMP 2024
        public static final double rotationKS = 0.02;
        public static final double rotationIZone = 2.0; // degrees

        public static final double tiltWarning = 10.0;
        public static final double tiltError = 30.0;

        // TODO What about AndyMark field?
        // NOTE That FlippingUtil might need to be impacted
        public static final double fieldWidth = FlippingUtil.fieldSizeY; // Units.inchesToMeters(26*12 + 5);
        public static final double fieldLength = FlippingUtil.fieldSizeX; // Units.inchesToMeters(57*12 + 6.875);

        public static final double reefElevatorZoneRadius = Units.inchesToMeters(80.0); // TODO Revisit
        public static final double autoUpDistance = Units.inchesToMeters(44.0);
        public static final double wingLength = Units.inchesToMeters(280);
        public static final double processorAreaY = fieldWidth / 2.0 - 0.5;

        public static final double robotFrameLength = Units.inchesToMeters(30);
        public static final double bumperWidth = Units.inchesToMeters(3.2);
        public static final double reefStandoff = Units.inchesToMeters(1.5);
        public static final double reefOffset = robotFrameLength / 2.0 + bumperWidth + reefStandoff;
        public static final double reefExtraOffset = Units.inchesToMeters(9.0);
        public static final double bonusStandoff = Units.inchesToMeters(4.0);

        // Locations from the Blue Alliance perspective
        public static final Translation2d reefCenter = new Translation2d(Units.inchesToMeters(176.75), fieldWidth / 2.0);
        public static final double reefToFaceDistance = reefCenter.getX() - Units.inchesToMeters(144.0);
        public static final double branchSeparation = Units.inchesToMeters(12.0 + 15.0 / 16.0);
        public static final double bargeShotDistanceFromCenter = Units.inchesToMeters(52.0);
        public static final double bargeShotX = fieldLength / 2.0 - bargeShotDistanceFromCenter - robotFrameLength / 2.0 - bumperWidth;

        // Offset to the reef face, not at the branches, but on the faces directly in front
        public static final Translation2d centerOffset = new Translation2d(reefToFaceDistance + reefOffset - reefStandoff, 0.0); // NOTE: Undo reef standoff for algae
        private static final Translation2d leftOffset = new Translation2d(reefToFaceDistance + reefOffset, -branchSeparation / 2.0);
        private static final Translation2d rightOffset = new Translation2d(reefToFaceDistance + reefOffset, branchSeparation / 2.0);
        private static final Translation2d extraOffset = new Translation2d(reefExtraOffset, 0.0);
        private static final Translation2d centerApproachOffset = centerOffset.plus(extraOffset);
        private static final Translation2d leftApproachOffset = leftOffset.plus(extraOffset);
        private static final Translation2d rightApproachOffset = rightOffset.plus(extraOffset);
        private static final Translation2d bonusOffset = new Translation2d(bonusStandoff, 0.0);
        private static final Translation2d leftBonusOffset = leftOffset.plus(bonusOffset);
        private static final Translation2d rightBonusOffset = rightOffset.plus(bonusOffset);
        private static final Transform2d leftL1Transform = new Transform2d(Units.inchesToMeters(-1.0), Units.inchesToMeters(3.0), Rotation2d.kZero);
        private static final Transform2d rightL1Transform = new Transform2d(Units.inchesToMeters(-1.0), Units.inchesToMeters(-3.0), Rotation2d.kZero);

        public static final double elevatorNoDownDistance = reefToFaceDistance + reefOffset + Units.inchesToMeters(12.0);

        public static enum ReefFace {
            AB(-180, true),
            CD(-120, false),
            EF(-60, true),
            GH(0, false),
            IJ(60, true),
            KL(120, false);

            ReefFace(double directionDegrees, boolean algaeHigh) {
                directionFromCenter = Rotation2d.fromDegrees(directionDegrees);
                alignMiddle = new Pose2d(reefCenter.plus(centerOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
                alignLeft = new Pose2d(reefCenter.plus(leftOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
                leftL1 = alignLeft.transformBy(leftL1Transform);
                alignRight = new Pose2d(reefCenter.plus(rightOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
                rightL1 = alignRight.transformBy(rightL1Transform);
                approachMiddle = new Pose2d(reefCenter.plus(centerApproachOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
                approachLeft = new Pose2d(reefCenter.plus(leftApproachOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
                approachRight = new Pose2d(reefCenter.plus(rightApproachOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
                alignBonusLeft = new Pose2d(reefCenter.plus(leftBonusOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
                alignBonusRight = new Pose2d(reefCenter.plus(rightBonusOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
                this.algaeHigh = algaeHigh;
            }

            public final Rotation2d directionFromCenter;
            public final Pose2d alignLeft, alignMiddle, alignRight;
            public final Pose2d leftL1, rightL1;
            public final Pose2d approachLeft, approachMiddle, approachRight;
            public final Pose2d alignBonusLeft, alignBonusRight;
            public final boolean algaeHigh;
        }

        // Cage locations from 6328
        public static enum Cage {
            CLOSE(199.947), // 5.079 m
            MIDDLE(242.855), // 6.169 m
            FAR(286.779), // 7.284 m
            HQ(8.16, 2.37);

            private final Translation2d location;
            private static final Translation2d redFudge = new Translation2d(0.0, 0.05);

            Cage(double yInches) {
                // 8.774 m
                location = new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(yInches));
            }

            Cage(double xMeters, double yMeters) {
                location = new Translation2d(xMeters, yMeters);
            }

            public Translation2d location() {
                if (Robot.isRed()) {
                    return location.plus(redFudge);
                } else {
                    return location;
                }        
            }
        }

        // 30 in / 2 + 3.2 in - 2.0 in = 16.2 in = 0.4115 m

        // fieldSizeX = Units.feetToMeters(57.573); 17.548 m
        // fieldSizeY = Units.feetToMeters(26.417); 8.052 m

        // Hypothetical
        // Blue CLOSE = 8.363, 5.08
        // Blue MIDDLE = 8.363, 6.17
        // Blue FAR = 8.363, 7.28
        // Red CLOSE = 9.19, 2.97
        // Red MIDDLE = 9.19, 1.883
        // Red FAR = 9.19, 0.768

        // Ashville
        // Want X diff of 0.30 m?
        // Blue CLOSE = 8.36 X 8.49, 5.23 X 5.09 
        // Blue MIDDLE = 8.47, 6.16
        // Blue FAR = 
        // Red CLOSE = 9.11, 2.90
        // Red MIDDLE = 9.07, 1.84
        // Red FAR = X

        // public static final Transform2d cageOffset = new Transform2d(robotFrameLength / 2.0 + bumperWidth - Units.inchesToMeters(2.0), 0, Rotation2d.kZero);
        public static final Transform2d cageOffset = new Transform2d(0.30, 0, Rotation2d.kZero);
        public static final Transform2d cageApproachOffset = new Transform2d(Units.inchesToMeters(16.0), 0, Rotation2d.kZero);
    }

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
                Swerve.wheelCircumference / (Math.PI * 2.0),
                Swerve.maxSpeed * 0.95, // Leave a little headroom for inefficiencies
                1.916, // 3847 Spectrum Vex GripLock v2 CoF
                DCMotor.getKrakenX60Foc(1),
                Swerve.chosenModule.driveGearRatio,
                Swerve.driveCurrentLimit,
                1),
            new Translation2d(Swerve.wheelBase / 2.0, Swerve.trackWidth / 2.0),
            new Translation2d(Swerve.wheelBase / 2.0, -Swerve.trackWidth / 2.0),
            new Translation2d(-Swerve.wheelBase / 2.0, Swerve.trackWidth / 2.0),
            new Translation2d(-Swerve.wheelBase / 2.0, -Swerve.trackWidth / 2.0));
    }

    public static final class LEDs {
        /* IDs */
        public static final int leftCandle = 0;
        public static final int rightCandle = 1;
        /* CANbus */
        public static final String canBus = "rio";
        /* LED arrangement */
        public static final int startIdx = 8;
        public static final int numLEDs = 86;
        public static final int totalLEDs = startIdx + numLEDs;
        public static final double brightness = Constants.atHQ ? 0.60 : 1.00;
        /* Animations */
        public static final FireAnimation fireAnimation = new FireAnimation(1.0, 0.38, numLEDs, 0.8, 0.2, false, startIdx);
        public static final RainbowAnimation rainbowAnimation = new RainbowAnimation(1.0, 0.7, numLEDs, false, startIdx);
        public static final LarsonAnimation larsonAnimation = new LarsonAnimation(255, 64, 0, 0, 0.7, numLEDs, BounceMode.Front, 7, startIdx);
        /* Misc */
        public static final double blinkRate = 0.2; // Regular blink rate
        public static final double errorBlinkRate = 0.1; // Blink rate for errors and warnings
        public static final double tempStateTime = 0.70; // How long for warnings and errors
    }
}