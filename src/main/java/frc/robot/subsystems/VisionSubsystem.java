// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.lib.util.TunableOption;
import frc.robot.Constants;
import frc.robot.Constants.Vision.Camera;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.EnumMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import dev.doglog.DogLog;

public class VisionSubsystem extends SubsystemBase {
    private static VisionSubsystem instance;
    private final Field2d field = new Field2d();
    private Pose2d lastPose = new Pose2d();
    // private static final TunableOption optUpdateVisionDashboard = new TunableOption("Update vision dashboard", false);
    private static PoseEstimator<SwerveModulePosition[]> poseEstimator = null;
    private static Supplier<Rotation2d> headingProvider = null;
    public static final EnumMap<Camera, PhotonCamera> cameras = new EnumMap<>(Camera.class);
    public static final Camera[] cameraTypes = Camera.values();

    private static record PoseResult(double timestamp, Pose3d pose, List<Short> fiducialIDs, double averageTagDistance) {
    }

    static {
        for (Camera camera : Camera.values()) {
            cameras.put(camera, new PhotonCamera(camera.name));
        }
    }

    public VisionSubsystem() {
        assert (instance == null);
        instance = this;

        SmartDashboard.putData("Vision/Field", field);
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

        DogLog.log("Vision/" + cameraType + "/Connected", camera.isConnected());

        for (PhotonPipelineResult result : results) {
            if (result.multitagResult.isPresent()) {
                MultiTargetPNPResult multitagResult = result.multitagResult.get();
                
                robotPose = Pose3d.kZero.plus(multitagResult.estimatedPose.best).plus(cameraType.robotToCamera.inverse());

                // Calculate average tag distance
                double totalTagDistance = 0.0;
                for (var target : result.targets) {
                    totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
                }

                poseResults.add(new PoseResult(result.getTimestampSeconds(), robotPose, multitagResult.fiducialIDsUsed, totalTagDistance / result.targets.size()));
            } else if (!result.targets.isEmpty()) {
                PhotonTrackedTarget target = result.targets.get(0);
                if (target.poseAmbiguity > Constants.Vision.maxAmbiguity) {
                    // TODO Log
                    continue;
                }

                Optional<Pose3d> tagPose = Constants.fieldLayout.getTagPose(target.fiducialId);
                if (!tagPose.isPresent()) {
                    // TODO Log warning
                    continue;
                }
                robotPose = tagPose.get().plus(target.bestCameraToTarget.inverse()).plus(cameraType.robotToCamera.inverse());
                double distance = target.bestCameraToTarget.getTranslation().getNorm();

                // Choose the alternate pose if it's better aligned with the current robot pose
                if (target.getPoseAmbiguity() > Constants.Vision.autoAcceptAmbiguity && headingProvider != null) {
                    Pose3d altPose = tagPose.get().plus(target.altCameraToTarget.inverse()).plus(cameraType.robotToCamera.inverse());
                    Rotation2d heading = headingProvider.get();

                    if (Math.abs(altPose.getRotation().toRotation2d().minus(heading).getRadians()) <
                        Math.abs(robotPose.getRotation().toRotation2d().minus(heading).getRadians())) {
                        robotPose = altPose;
                        distance = target.altCameraToTarget.getTranslation().getNorm();
                    }
                }

                poseResults.add(new PoseResult(result.getTimestampSeconds(), robotPose, List.of((short)target.fiducialId), distance));
            }

            // TODO Track all tags

            // DogLog.log("Vision/TargetPoses", (Pose3d[])result.getTargets().stream().map(tgt -> robotPose.estimatedPose.plus(Constants.Vision.robotToCam).plus(tgt.getBestCameraToTarget())).toArray(size -> new Pose3d[size]));
            // DogLog.log("Vision/Pose Difference", PoseSubsystem.getInstance().getPose().getTranslation().getDistance(lastPose.getTranslation()));
        }

        return poseResults;
    }

    public Pose2d lastPose() {
        return lastPose;
    }

    @Override
    public void periodic() {
        for (var camera : cameraTypes) {
            List<PoseResult> poseResults = processCamera(camera);

            for (var poseResult : poseResults) {
                DogLog.log("Vision/" + camera.toString() + "/Pose", poseResult.pose);
                if (poseIsReasonable(poseResult.pose)) {
                    if (poseEstimator != null) {
                        double stdDevFactor = Math.pow(poseResult.averageTagDistance, 2.0) / poseResult.fiducialIDs.size();
                        double linearStdDev = Constants.Vision.linearStdDevBaseline * stdDevFactor;
                        double angularStdDev = Constants.Vision.angularStdDevBaseline * stdDevFactor;
                        // NOTE Could possibly scale by camera, too

                        poseEstimator.addVisionMeasurement(poseResult.pose.toPose2d(), poseResult.timestamp, VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
                        // field.setRobotPose(lastPose);
                    }
                }
            }
        }
    }
}