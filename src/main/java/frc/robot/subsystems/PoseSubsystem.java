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
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Elastic;
import frc.lib.util.Elastic.Notification;
import frc.lib.util.Elastic.Notification.NotificationLevel;
import frc.lib.util.TunableOption;
import frc.robot.Constants;
import frc.robot.Constants.Pose;
import frc.robot.Robot;
import frc.robot.commands.LoggedCommands;

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

        Pose.rotationPID.enableContinuousInput(-180.0, 180.0);
        Pose.rotationPID.setIZone(Pose.rotationIZone); // Only use Integral term within this range
        Pose.rotationPID.reset();

        Pose.maintainPID.enableContinuousInput(-180.0, 180.0);
        Pose.maintainPID.setIZone(Pose.rotationIZone); // Only use Integral term within this range
        Pose.maintainPID.reset();

        poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(), s_Swerve.getModulePositions(), new Pose2d());

        field = new Field2d();
        SmartDashboard.putData("pose/Field", field);

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        AutoBuilder.configure(
            this::getPose,
            this::setPose,
            s_Swerve::getSpeeds, 
            (speeds, feedforwards) -> s_Swerve.driveRobotRelativeAuto(speeds),
            // TODO Configure PIDs
            new PPHolonomicDriveController(
                new PIDConstants(8.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(1.5, 0.0, 0.0)  // Rotation PID constants
            ),
            config,
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

    public Command zeroGyro() {
        return LoggedCommands.runOnce("Zeroed Gyro",
            () -> {
                gyro.setYaw(0);
            },
            this);
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

    public Command zeroHeading() {
        return LoggedCommands.runOnce("Zeroed Heading",
            () -> {
                setHeading(new Rotation2d());
            },
            this);
    }

    public Command resetHeading() {
        return LoggedCommands.runOnce("Resetting Heading",
            () -> {
                if (Robot.isRed()) {
                    setHeading(new Rotation2d(Math.PI));
                } else {
                    setHeading(new Rotation2d());
                }
            },
            this);
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

        SmartDashboard.putNumber("pose/Gyro", getHeading().getDegrees());
        SmartDashboard.putString("pose/Pose", prettyPose(pose));

        DogLog.log("Pose/Pose", pose);
        DogLog.log("Pose/Gyro/Heading", getHeading().getDegrees());
        DogLog.log("Pose/Gyro/Raw Yaw", getGyroYaw());
    }
}