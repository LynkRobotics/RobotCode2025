// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pose;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.PathPlannerLogging;

import dev.doglog.DogLog;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.Robot;
import frc.robot.autos.AutoConstants;

public class Pose extends SubsystemBase {
    public static final Pose instance = new Pose();

    private final SwerveDrivePoseEstimator poseEstimator;
    private final Pigeon2 gyro;

    public Pose() {
        gyro = new Pigeon2(0, SwerveConstants.swerveCanBus);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);        

        poseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.swerveKinematics, getGyroYaw(), Swerve.instance.getModulePositions(), new Pose2d());

        RobotConfig config;
        // try {
        //     config = RobotConfig.fromGUISettings();
        //     DogLog.log("Pose/Status", "Loaded robot config from GUI file");
        // } catch (Exception e) {
        //     DriverStation.reportError(e.getMessage(), false);
            config = AutoConstants.robotConfig;
            DogLog.log("Pose/Status", "Using robot config from code");
        // }

        AutoBuilder.configure(
            this::getPose,
            this::setPose,
            Swerve.instance::getSpeeds,
            (speeds, feedforwards) -> Swerve.instance.driveRobotRelativeAuto(speeds),
            // TODO Configure PIDs
            new PPHolonomicDriveController(
                new PIDConstants(10.0, 0.0, 0.0), //Translation PID constants
                new PIDConstants(5, 0.0, 0.0)  // Rotation PID constants
            ),
            // AutoConstants.robotConfig,
            config,
            Robot::isRed,
            Swerve.instance // Reference to Swerve subsystem to set requirements
        );

        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
            DogLog.log("Pose/Auto Target Pose", targetPose);
        });
        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            DogLog.log("Pose/Active Path", activePath.toArray(Pose2d[]::new)); //we have to convert the List of poses PathPlanner gives us to an array because DogLog does not support list, fourtunetely aScope doesn't care whether its a list or an array
        });
        PathPlannerLogging.setLogCurrentPoseCallback((currentPose) -> {
            DogLog.log("Pose/PP Current Pose", currentPose);
        });    }
    
    public Rotation2d getGyroYaw() {
        return new Rotation2d(gyro.getYaw().getValue());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), Swerve.instance.getModulePositions(), pose);
        DogLog.log("Pose/Status/Setting Pose", pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    @Override
    public void periodic() {
        poseEstimator.update(getGyroYaw(), Swerve.instance.getModulePositions());
        Pose2d pose = getPose();
        Robot.field.setRobotPose(pose);

        DogLog.log("Pose/Pose", pose);
    }
}