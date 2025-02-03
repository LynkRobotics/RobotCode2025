// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

    private static final TunableOption optUpdatePoseWithVisionAuto = new TunableOption("pose/Update with vision in Auto", false);

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

        // Pose.rotationPID.enableContinuousInput(-180.0, 180.0);
        // Pose.rotationPID.setIZone(Pose.rotationIZone); // Only use Integral term within this range
        // Pose.rotationPID.reset();

        // Pose.maintainPID.enableContinuousInput(-180.0, 180.0);
        // Pose.maintainPID.setIZone(Pose.rotationIZone); // Only use Integral term within this range
        // Pose.maintainPID.reset();

        poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(), s_Swerve.getModulePositions(), new Pose2d());

        field = new Field2d();
        SmartDashboard.putData("pose/Field", field);

        // RobotConfig config;
        // try {
            // config = RobotConfig.fromGUISettings();
        // } catch (Exception e) {
            // e.printStackTrace();
            // return;
        // }

        // AutoBuilder.configure(
            // this::getPose,
            // this::setPose,
            // s_Swerve::getSpeeds, 
            // (speeds, feedforwards) -> s_Swerve.driveRobotRelativeAuto(speeds),
            // TODO Configure PIDs
            // new PPHolonomicDriveController(
                // new PIDConstants(8.0, 0.0, 0.0), // Translation PID constants
                // new PIDConstants(1.5, 0.0, 0.0)  // Rotation PID constants
            // ),
            // config,
            // Robot::isRed,
            // s_Swerve // Reference to Swerve subsystem to set requirements
        // );

        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
            DogLog.log("Pose/Auto Target Pose", targetPose);
        });
        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            DogLog.log("Pose/Active Path", activePath.toArray(Pose2d[]::new)); //we have to convert the List of poses PathPlanner gives us to an array because DogLog does not support list, fourtunetely aScope doesn't care whether its a list or an array
        });
        PathPlannerLogging.setLogCurrentPoseCallback((currentPose) -> {
            DogLog.log("Pose/PP Current Pose", currentPose);
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

    public void zeroGyro() {
        gyro.setYaw(0);
        DogLog.log("Pose/Gyro/Status", "Zeroed Gyro Yaw");
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

    public static Translation2d otherAlliance(Translation2d position) {
        // Rotationally symmetric this year
        return new Translation2d(Constants.Pose.fieldLength - position.getX(), Constants.Pose.fieldWidth - position.getY());
    }

    public static Translation2d flipIfRed(Translation2d position) {
        return Robot.isRed() ? otherAlliance(position) : position;
    }

    public static Rotation2d reefBearing(Translation2d position) {
        Translation2d reefCenter = flipIfRed(Constants.Pose.reefCenter);
        Translation2d relativePosition = position.minus(reefCenter);

        return relativePosition.getAngle();
    }

    public static ReefFace nearestFace(Translation2d position) {
        Rotation2d reefBearing = reefBearing(position);
        if (Robot.isRed()) {
            reefBearing.plus(Rotation2d.k180deg);
        }
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

        SmartDashboard.putNumber("Pose/Gyro", getHeading().getDegrees());
        SmartDashboard.putString("Pose/Pose", prettyPose(pose));
        SmartDashboard.putNumber("Pose/Reef Bearing", reefBearing(pose.getTranslation()).getDegrees());
        SmartDashboard.putString("Pose/Nearest Face", nearestFace(pose.getTranslation()).toString());

        DogLog.log("Pose/Pose", pose);
        DogLog.log("Pose/Gyro/Heading", getHeading().getDegrees());
        DogLog.log("Pose/Gyro/Raw Yaw", getGyroYaw());
    }
}