// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableOption;
import frc.robot.Constants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import dev.doglog.DogLog;

public class VisionSubsystem extends SubsystemBase {
    private static VisionSubsystem instance;
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private AprilTagFieldLayout kTagLayout;
    private final Field2d field = new Field2d();
    private Pose2d lastPose = new Pose2d();
    private PhotonPipelineResult lastResult = null;
    private boolean updateDashboard = true;
    private static final TunableOption optUpdateVisionDashboard = new TunableOption("Update vision dashboard", false);

    public VisionSubsystem() {
        assert (instance == null);
        instance = this;

        camera = new PhotonCamera(Constants.Vision.cameraName);

        kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

        photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                Constants.Vision.robotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        SmartDashboard.putData("Vision/Field", field);
    }

    public static VisionSubsystem getInstance() {
        return instance;
    }

    public boolean updatePoseEstimate(PoseEstimator<SwerveModulePosition[]> poseEstimator) {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        boolean newResult = !results.isEmpty();
        boolean updated = false;

        // TODO Consider updating standard deviations

        for (PhotonPipelineResult result : results) {
            DogLog.log("Vision/Targets", result.getTargets().stream().mapToInt(tgt -> tgt.getFiducialId()).toArray());
            lastResult = result;

            Optional<EstimatedRobotPose> optRobotPose = photonEstimator.update(result);

            if (!optRobotPose.isPresent()) {
                continue;
            }

            EstimatedRobotPose robotPose = optRobotPose.get();
            lastPose = robotPose.estimatedPose.toPose2d();
            field.setRobotPose(lastPose);
            DogLog.log("Vision/Pose Difference", PoseSubsystem.getInstance().getPose().getTranslation().getDistance(lastPose.getTranslation()));
            if (poseEstimator != null) {
                poseEstimator.addVisionMeasurement(lastPose, robotPose.timestampSeconds);
                updated = true;
            }
        }

        if (updateDashboard) {
            SmartDashboard.putBoolean("Vision/New result", newResult);
        }

        return updated;
    }

    @Override
    public void periodic() {
        boolean haveTarget = lastResult != null && lastResult.hasTargets();

        // TODO Belongs in periodic() or elsewhere?
        if (lastResult != null) {
            DogLog.log("Vision/Result", lastResult.toString());
        }
        DogLog.log("Vision/Have target(s)", haveTarget);
        DogLog.log("Vision/Pose", lastPose);

        if (optUpdateVisionDashboard.get()) {
            SmartDashboard.putString("Vision/Result", lastResult.toString());
            SmartDashboard.putBoolean("Vision/Have target(s)", haveTarget);
            SmartDashboard.putString("Vision/Last pose", PoseSubsystem.prettyPose(lastPose));
        }
    }
}