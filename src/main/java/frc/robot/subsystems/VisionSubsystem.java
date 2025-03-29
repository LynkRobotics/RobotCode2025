// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedAlert;
// import frc.lib.util.TunableOption;
import frc.robot.Constants;
import frc.robot.Constants.Vision.Camera;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Options.optUseTrigVision;

import java.util.EnumMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import dev.doglog.DogLog;

public class VisionSubsystem extends SubsystemBase {
    private static VisionSubsystem instance;
    private Pose2d lastPose = new Pose2d();
    // private static final TunableOption optUpdateVisionDashboard = new TunableOption("Update vision dashboard", false);
    private static PoseEstimator<SwerveModulePosition[]> poseEstimator = null;
    private static Supplier<Rotation2d> headingProvider = null;
    // TODO Combine into record
    private static final EnumMap<Camera, PhotonCamera> cameras = new EnumMap<>(Camera.class);
    private static final EnumMap<Camera, PhotonPoseEstimator> photonEstimator = new EnumMap<>(Camera.class);
    private static final EnumMap<Camera, Field2d> field = new EnumMap<>(Camera.class);
    private static final Camera[] cameraTypes = Camera.values();

    private static record PoseResult(double timestamp, Pose3d pose, List<Short> fiducialIDs, double ambiguity, double averageTagDistance) {
    }

    static {
        for (Camera cameraType : Camera.values()) {
            cameras.put(cameraType, new PhotonCamera(cameraType.name));
            photonEstimator.put(cameraType, new PhotonPoseEstimator(Constants.fieldLayout, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE, cameraType.robotToCamera));
            field.put(cameraType, new Field2d());
            SmartDashboard.putData("Vision/" + cameraType + " Field", field.get(cameraType));
        }
    }

    public VisionSubsystem() {
        assert (instance == null);
        instance = this;
    }

    public static VisionSubsystem getInstance() {
        return instance;
    }

    public static void setPoseEstimator(PoseEstimator<SwerveModulePosition[]> poseEstimator) {
        VisionSubsystem.poseEstimator = poseEstimator;
    }

    public static void setHeadingProvider(Supplier<Rotation2d> headingProvider) {
        VisionSubsystem.headingProvider = headingProvider;
    }

    public static boolean poseIsReasonable(Pose3d pose) {
        if (pose.getX() < -Constants.Vision.fieldBorderMargin
            || pose.getX() > Constants.Pose.fieldLength + Constants.Vision.fieldBorderMargin
            || pose.getY() < -Constants.Vision.fieldBorderMargin
            || pose.getY() > Constants.Pose.fieldWidth + Constants.Vision.fieldBorderMargin
            || pose.getZ() < -Constants.Vision.maxZError
            || pose.getZ() > Constants.Vision.maxZError) {
            return false;
        }

        return true;
    }

    private List<PoseResult> processCamera(Camera cameraType) {
        List<PoseResult> poseResults = new LinkedList<>();
        PhotonCamera camera = cameras.get(cameraType);
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        Pose3d robotPose;
        String logPrefix = "Vision/" + cameraType + "/";

        DogLog.log(logPrefix + "Connected", camera.isConnected());

        for (PhotonPipelineResult result : results) {
            double timestamp = result.getTimestampSeconds();
            String tsString = String.format("%1.3f", timestamp);

            if (result.multitagResult.isPresent()) {
                MultiTargetPNPResult multitagResult = result.multitagResult.get();
                
                robotPose = Pose3d.kZero.plus(multitagResult.estimatedPose.best).plus(cameraType.robotToCamera.inverse());

                // Calculate average tag distance
                double totalTagDistance = 0.0;
                for (var target : result.targets) {
                    totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
                }

                poseResults.add(new PoseResult(timestamp, robotPose, multitagResult.fiducialIDsUsed, multitagResult.estimatedPose.ambiguity, totalTagDistance / result.targets.size()));
                DogLog.log(logPrefix + "Status", "Using multitag result from " + tsString);
            } else if (!result.targets.isEmpty()) {
                PhotonTrackedTarget target = result.targets.get(0);
                if (target.poseAmbiguity > Constants.Vision.maxAmbiguity) {
                    DogLog.log(logPrefix + "Status", "Rejected ambiguous (" + String.format("%1.2f", target.poseAmbiguity) + ") single tag pose from " + tsString);
                    continue;
                }

                Optional<Pose3d> tagPose = Constants.fieldLayout.getTagPose(target.fiducialId);
                if (!tagPose.isPresent()) {
                    LoggedAlert.Warning("Vision", "Invalid AprilTag", "Invalid AprilTag ID " + target.fiducialId);
                    DogLog.log(logPrefix + "Status", "Rejected invalid AprilTag ID (" + target.fiducialId + ") from " + tsString);
                    continue;
                }

                double distance = target.bestCameraToTarget.getTranslation().getNorm();

                if (optUseTrigVision.get()) {
                    Optional<EstimatedRobotPose> newPose = photonEstimator.get(cameraType).update(result);
                    if (!newPose.isPresent()) {
                        continue;
                    }
                    robotPose = newPose.get().estimatedPose;
                    DogLog.log(logPrefix + "Status", "Using estimator result from " + tsString);
                } else {
                    robotPose = tagPose.get().plus(target.bestCameraToTarget.inverse()).plus(cameraType.robotToCamera.inverse());

                    // Choose the alternate pose if it's better aligned with the current robot pose
                    if (target.poseAmbiguity > Constants.Vision.autoAcceptAmbiguity && headingProvider != null) {
                        Pose3d altPose = tagPose.get().plus(target.altCameraToTarget.inverse()).plus(cameraType.robotToCamera.inverse());
                        Rotation2d heading = headingProvider.get();

                        if (Math.abs(altPose.getRotation().toRotation2d().minus(heading).getRadians()) <
                            Math.abs(robotPose.getRotation().toRotation2d().minus(heading).getRadians())) {
                            robotPose = altPose;
                            distance = target.altCameraToTarget.getTranslation().getNorm();
                            DogLog.log(logPrefix + "Status", "Using alternate result from " + tsString);
                        } else {
                            DogLog.log(logPrefix + "Status", "Using best result from " + tsString);
                        }
                    } else {
                        DogLog.log(logPrefix + "Status", "Rejecting result from " + tsString);
                    }
                }

                poseResults.add(new PoseResult(result.getTimestampSeconds(), robotPose, List.of((short)target.fiducialId), target.poseAmbiguity, distance));
            }
        }

        return poseResults;
    }

    public Pose2d lastPose() {
        return lastPose;
    }

    @Override
    public void periodic() {
        Rotation2d heading = null;
        double timestamp = 0;

        if (headingProvider != null) {
            heading = headingProvider.get();
            timestamp = Timer.getFPGATimestamp();
        }
        for (var cameraType : cameraTypes) {
            if (heading != null) {
                photonEstimator.get(cameraType).addHeadingData(timestamp, heading);
            }

            String logPrefix = "Vision/" + cameraType + "/";
            for (PoseResult poseResult : processCamera(cameraType)) {
                DogLog.log(logPrefix + "Timestamp", poseResult.timestamp);
                DogLog.log(logPrefix + "Pose", poseResult.pose);
                DogLog.log(logPrefix + "Ambiguity", poseResult.ambiguity);
                DogLog.log(logPrefix + "Distance", poseResult.averageTagDistance);
                DogLog.log(logPrefix + "Tag Poses", (Pose3d[])poseResult.fiducialIDs.stream().map(tag -> Constants.fieldLayout.getTagPose(tag).get()).toArray(size -> new Pose3d[size]));
                // DogLog.log(logPrefix + "Pose Difference", PoseSubsystem.getInstance().getPose().getTranslation().getDistance(poseResult.pose.getTranslation()));
                if (poseIsReasonable(poseResult.pose)) {
                    if (poseEstimator != null) {
                        double stdDevFactor = poseResult.averageTagDistance * poseResult.averageTagDistance / poseResult.fiducialIDs.size();
                        double linearStdDev = Constants.Vision.linearStdDevBaseline * stdDevFactor;
                        double angularStdDev = Constants.Vision.angularStdDevBaseline * stdDevFactor;
                        // NOTE Could possibly scale by camera, too

                        Pose2d pose = poseResult.pose.toPose2d();
                        poseEstimator.addVisionMeasurement(pose, poseResult.timestamp, VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
                        field.get(cameraType).setRobotPose(pose);
                    }
                }
            }
        }
    }
}