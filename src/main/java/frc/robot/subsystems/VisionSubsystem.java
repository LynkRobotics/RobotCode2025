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
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;
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
  private double lastEstTimestamp = 0.0;
  private boolean haveTarget = false;
  private boolean haveSpeakerTarget = false;
  private boolean haveAmpTarget = false;
  private boolean haveSourceTarget = false;
  private Pose2d lastPose  = new Pose2d();
  private boolean updateDashboard = true;
  private static final TunableOption optUpdateVisionDashboard = new TunableOption("Update vision dashboard", false);

  public VisionSubsystem() {
    assert(instance == null);
    instance = this;

    camera = new PhotonCamera(Constants.Vision.cameraName);

    kTagLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

    photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.robotToCam);
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    SmartDashboard.putData("vision/Field", field);
  }
  public static VisionSubsystem getInstance() {
    return instance;
  }

  public boolean updatePoseEstimate(PoseEstimator<SwerveModulePosition[]> poseEstimator) {
    PhotonPipelineResult result = camera.getLatestResult(); // TODO Switch to getAllUnreadResults(), but need to revisit periodic(), too
    Optional<EstimatedRobotPose> optVisionEst = photonEstimator.update(result);
    EstimatedRobotPose visionEst;
    double latestTimestamp;
    boolean newResult;
    
    // TODO Consider updating standard deviations

    if (!optVisionEst.isPresent()) {
      return false;
    }
    visionEst = optVisionEst.get();
    latestTimestamp = visionEst.timestampSeconds;
    newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    if (updateDashboard) {
      SmartDashboard.putBoolean("vision/New result", newResult);
    }
    if (!newResult) {
      return false;
    }
    lastEstTimestamp = latestTimestamp;
    lastPose = visionEst.estimatedPose.toPose2d();
    field.setRobotPose(lastPose);
    if (poseEstimator != null) {
      poseEstimator.addVisionMeasurement(lastPose, lastEstTimestamp);
    }
    return true;
  }

  public Pose2d lastPose() {
    return lastPose;
  }

  @Override
  public void periodic() {
    PhotonPipelineResult result = camera.getLatestResult();

    haveTarget = result.hasTargets();

    DogLog.log("Vision/Result", result.toString());
    DogLog.log("Vision/Have target(s)", haveTarget);
    DogLog.log("Vision/Pose", lastPose);

    if (optUpdateVisionDashboard.get()) {
      SmartDashboard.putString("vision/Result", result.toString());
      SmartDashboard.putBoolean("vision/Have target(s)", haveTarget);
      SmartDashboard.putBoolean("vision/Have speaker target", haveSpeakerTarget);
      SmartDashboard.putBoolean("vision/Have amp target", haveAmpTarget);
      SmartDashboard.putBoolean("vision/Have source target", haveSourceTarget);
      SmartDashboard.putString("vision/Last pose", String.format("%01.2f, %01.2f @ %01.1f", lastPose.getX(), lastPose.getY(), lastPose.getRotation().getDegrees()));
    }
  }
}