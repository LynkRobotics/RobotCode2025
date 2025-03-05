// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.PathPlannerLogging;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableOption;
import frc.robot.Constants;
import frc.robot.Constants.Pose;
import frc.robot.Constants.Pose.ReefFace;
import frc.robot.Robot;

public class PoseSubsystem extends SubsystemBase {
    private static PoseSubsystem instance;
    private final Swerve s_Swerve;
    private final VisionSubsystem s_Vision;

    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field;
    private final Pigeon2 gyro;

    private static final TunableOption optUpdatePoseWithVisionAuto = new TunableOption("Pose/Update with vision in Auto", true);

    public enum Zone {
        SPEAKER,
        MIDDLE,
        FAR
    }

    public PoseSubsystem(Swerve s_Swerve, VisionSubsystem s_Vision) {
        assert(instance == null);
        instance = this;
        
        this.s_Swerve = s_Swerve;
        this.s_Vision = s_Vision;

        gyro = new Pigeon2(Pose.pigeonID, Constants.Swerve.swerveCanBus);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);        

        Pose.rotationPID.enableContinuousInput(-180.0, 180.0);
        Pose.rotationPID.setIZone(Pose.rotationIZone); // Only use Integral term within this range
        Pose.rotationPID.reset();

        poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(), s_Swerve.getModulePositions(), new Pose2d());

        field = new Field2d();
        SmartDashboard.putData("Pose/Field", field);

        AutoBuilder.configure(
            this::getPose,
            this::setPose,
            s_Swerve::getSpeeds, 
            (speeds, feedforwards) -> s_Swerve.driveRobotRelativeAuto(speeds),
            // TODO Configure PIDs
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5, 0.0, 0.0)  // Rotation PID constants
            ),
            Constants.PathPlanner.robotConfig,
            Robot::isRed,
            s_Swerve // Reference to Swerve subsystem to set requirements
        );

        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
            DogLog.log("Pose/Auto Target Pose", targetPose);
        });
        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            DogLog.log("Pose/Active Path", activePath.toArray(Pose2d[]::new)); //we have to convert the List of poses PathPlanner gives us to an array because DogLog does not support list, fourtunetely aScope doesn't care whether its a list or an array
        });
        PathPlannerLogging.setLogCurrentPoseCallback((currentPose) -> {
            DogLog.log("Pose/PP Current Pose", currentPose);
        });

        SmartDashboard.putData("Pose/Heading", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("Gyro");
                builder.addDoubleProperty("Value", () -> getHeading().getDegrees(), null);
            }
        });

        SmartDashboard.putData("Pose/Reef Bearing", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("Gyro");
                builder.addDoubleProperty("Value", () -> reefBearing(getPose().getTranslation()).getDegrees(), null);
            }
        });
    }

    public static PoseSubsystem getInstance() {
        return instance;
    }
    
    public static String prettyPose(Pose2d pose) {
        return String.format("(%01.2f, %01.2f @ %01.1f)", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }
    
    public Rotation2d getGyroYaw() {
        return new Rotation2d(gyro.getYaw().getValue());
    }

    public Rotation2d getGyroRoll() {
        return new Rotation2d(gyro.getRoll().getValue());
    }

    public Rotation2d getGyroPitch() {
        return new Rotation2d(gyro.getPitch().getValue());
    }

    public void zeroGyro() {
        gyro.setYaw(0);
        DogLog.log("Pose/Gyro/Status", "Zeroed Gyro Yaw");
    }

    public static void angleErrorReset() {
        angleErrorReset(Pose.rotationPID);
    }

    public static void angleErrorReset(PIDController pid) {
        pid.reset();
    }
    public static double angleErrorToSpeed(Rotation2d angleError) {
        return angleErrorToSpeed(angleError, Pose.rotationPID);
    }

    public static double angleErrorToSpeed(Rotation2d angleError, PIDController pid) {
        double angleErrorDeg = angleError.getDegrees();
        double correction = pid.calculate(angleErrorDeg);
        double feedForward = Pose.rotationKS * Math.signum(correction);
        double output = MathUtil.clamp(correction + feedForward, -1.0, 1.0);

        DogLog.log("Pose/Angle Error", angleErrorDeg);
        DogLog.log("Pose/Angle PID correction", correction);
        DogLog.log("Pose/Angle feedforward", feedForward);
        DogLog.log("Pose/Angle output", output);
        
        // Invert due to use as joystick controls
        return -output;
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), s_Swerve.getModulePositions(), pose);
        DogLog.log("Pose/Status/Setting Pose", pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        poseEstimator.resetPosition(getGyroYaw(), s_Swerve.getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        setHeading(new Rotation2d());
        DogLog.log("Pose/Gyro/Status", "Zeroed Gyro Heading");
    }

    public void resetHeading() {
        if (Robot.isRed()) {
            setHeading(new Rotation2d(Math.PI));
        } else {
            setHeading(new Rotation2d());
        }
    }

    public static Translation2d flipIfRed(Translation2d position) {
        return Robot.isRed() ? FlippingUtil.flipFieldPosition(position) : position;
    }

    public static Pose2d flipIfRed(Pose2d pose) {
        return Robot.isRed() ? FlippingUtil.flipFieldPose(pose) : pose;
    }

    public static Rotation2d flipIfRed(Rotation2d rotation) {
        return Robot.isRed() ? FlippingUtil.flipFieldRotation(rotation) : rotation;
    }

    public static Rotation2d reefBearing(Translation2d position) {
        Translation2d reefCenter = flipIfRed(Constants.Pose.reefCenter);
        Translation2d relativePosition = reefCenter.minus(position);

        return relativePosition.getAngle();
    }

    public static ReefFace nearestFace(Translation2d position) {
        Rotation2d reefBearing = flipIfRed(reefBearing(position));
        double bearingAngle = MathUtil.inputModulus(reefBearing.getDegrees(), -180, 180);

        if (bearingAngle > 150 || bearingAngle < -150) {
            return ReefFace.GH;
        } else if (bearingAngle > 90) {
            return ReefFace.EF;
        } else if (bearingAngle > 30) {
            return ReefFace.CD;
        } else if (bearingAngle > -30) {
            return ReefFace.AB;
        } else if (bearingAngle > -90) {
            return ReefFace.KL;
        } else { // bearingAngle > -150
            return ReefFace.IJ;
        }
    }

    public static Rotation2d facePerpendicular(ReefFace face) {
        // TODO Fix for red
        // Or removed as unused
        if (face == ReefFace.CD) {
            return Rotation2d.fromDegrees(60);
        } else if (face == ReefFace.EF) {
            return Rotation2d.fromDegrees(120);
        } else if (face == ReefFace.GH) {
            return Rotation2d.k180deg;
        } else if (face == ReefFace.IJ) {
            return Rotation2d.fromDegrees(-120);
        } else if (face == ReefFace.KL) {
            return Rotation2d.fromDegrees(-60);
        } else { // face == ReefFace.AB
            return Rotation2d.kZero;
        }
    }

    public static double reefDistance(Translation2d position) {
        Translation2d reefCenter = flipIfRed(Constants.Pose.reefCenter);
        return position.getDistance(reefCenter);
    }

    public static double distanceTo(Translation2d target) {
        return PoseSubsystem.getInstance().getPose().getTranslation().getDistance(target);
    }

    public static boolean inReefElevatorZone(Translation2d position) {
        return reefDistance(position) <= Constants.Pose.reefElevatorZoneRadius;
    }

    public boolean inReefElevatorZone() {
        return inReefElevatorZone(getPose().getTranslation());
    }

    public static boolean elevatorDownAllowed(Translation2d position) {
        return reefDistance(position) >= Constants.Pose.elevatorNoDownDistance;
    }

    public boolean elevatorDownAllowed() {
        return elevatorDownAllowed(getPose().getTranslation());
    }

    public static boolean inWing(Translation2d position) {
        return flipIfRed(position).getX() <= Constants.Pose.wingLength;
    }

    public boolean isUpright() {
        double roll = getGyroRoll().getDegrees();
        double pitch = getGyroPitch().getDegrees();
        return (Math.abs(roll) < Pose.tiltError && Math.abs(pitch) < Pose.tiltError);
    }

    public boolean nearProcessor() {
        Translation2d position = flipIfRed(getPose().getTranslation());

        return position.getY() < Constants.Pose.processorAreaY;
    }

    public Pose2d bargeShotPose() {
        Pose2d currentPose = flipIfRed(getPose());
        Pose2d targetPose;

        if (currentPose.getX() > Constants.Pose.fieldLength / 2.0) {
            targetPose = new Pose2d(Constants.Pose.fieldLength - Constants.Pose.bargeShotX, currentPose.getY(), Rotation2d.k180deg);
        } else {
            targetPose = new Pose2d(Constants.Pose.bargeShotX, currentPose.getY(), Rotation2d.kZero);
        }

        return flipIfRed(targetPose);
    }

    @Override
    public void periodic() {
        poseEstimator.update(getGyroYaw(), s_Swerve.getModulePositions());
        if (!DriverStation.isAutonomousEnabled() || optUpdatePoseWithVisionAuto.get()) {
            s_Vision.updatePoseEstimate(poseEstimator);
        } else {
            s_Vision.updatePoseEstimate(null);
        }

        Pose2d pose = getPose();
        field.setRobotPose(pose);

        SmartDashboard.putString("Pose/Pose", prettyPose(pose));
        SmartDashboard.putData("Pose/Gyro (raw)", gyro);
 
        double roll = getGyroRoll().getDegrees();
        double pitch = getGyroPitch().getDegrees();
        SmartDashboard.putNumber("Pose/Gyro Roll", -roll);
        SmartDashboard.putNumber("Pose/Gyro Pitch", -pitch);
        if (Math.abs(roll) > Pose.tiltError || Math.abs(pitch) > Pose.tiltError) {
            SmartDashboard.putString("Pose/Tilt State", "#D61E1E");
        } else if (Math.abs(roll) > Pose.tiltWarning || Math.abs(pitch) > Pose.tiltWarning) {
            SmartDashboard.putString("Pose/Tilt State", "#D6BE1E");
        } else {
            SmartDashboard.putString("Pose/Tilt State", "#266336");
        }

        Translation2d position = pose.getTranslation();
        SmartDashboard.putString("Pose/Nearest Face", nearestFace(position).toString());
        SmartDashboard.putNumber("Pose/Reef Center Distance", reefDistance(position));
        SmartDashboard.putBoolean("Pose/Reef Elevator Zone", inReefElevatorZone(position));
        SmartDashboard.putBoolean("Pose/In Wing", inWing(position));

        DogLog.log("Pose/Pose", pose);
        DogLog.log("Pose/Gyro/Heading", getHeading().getDegrees());
        DogLog.log("Pose/Gyro/Raw Yaw", getGyroYaw());
    }
}