// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.PathPlannerLogging;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.autos.AutoConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

public class Pose extends SubsystemBase {
    public static final Pose instance = new Pose();

    public Pose() {
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
            () -> Drive.mInstance.getState().Speeds,
            (desiredSpeeds, feedforwards) -> Drive.mInstance.setSwerveRequest(DriveConstants.PIDToPoseRequest
                .withVelocityX(desiredSpeeds.vxMetersPerSecond)
				.withVelocityY(desiredSpeeds.vyMetersPerSecond)
				.withRotationalRate(desiredSpeeds.omegaRadiansPerSecond)),
            // TODO Configure PIDs
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0), //Translation PID constants
                new PIDConstants(5, 0.0, 0.0)  // Rotation PID constants
            ),
            // AutoConstants.robotConfig,
            config,
            Robot::isRed,
            Drive.mInstance
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
        return new Rotation2d(Drive.mInstance.getGeneratedDrive().getPigeon2().getYaw().getValue());
    }

    public Pose2d getPose() {
        return Drive.mInstance.getPose();
    }

    public void setPose(Pose2d pose) {
        Drive.mInstance.getGeneratedDrive().resetPose(pose);
        DogLog.log("Pose/Status/Setting Pose", pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    @Override
    public void periodic() {
        Pose2d pose = getPose();
        Robot.field.setRobotPose(pose);

        DogLog.log("Pose/Pose", pose);
    }
}