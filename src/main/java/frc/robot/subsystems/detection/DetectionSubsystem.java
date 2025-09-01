// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.detection;

import frc.robot.subsystems.detection.DetectionConstants.Camera;
import frc.robot.subsystems.detection.DetectionConstants.CameraMode;

import java.util.EnumMap;
import java.util.LinkedList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DetectionSubsystem extends SubsystemBase {
  public static final DetectionSubsystem instance = new DetectionSubsystem();

  private static final EnumMap<Camera, PhotonCamera> cameras = new EnumMap<>(Camera.class); 

  public record ObjectTargetData(double timestamp, int objectId, String objectName, double confidence, Transform3d transform) {
  }

  static {
    for (Camera cameraType : DetectionConstants.camerasAvailable) {
      cameras.put(cameraType, new PhotonCamera(cameraType.name()));
    }
  }

  public DetectionSubsystem() {

  }

  private List<ObjectTargetData> processCamera(Camera cameraType){
    List<ObjectTargetData> objectTargetData = new LinkedList<>();
    PhotonCamera camera = cameras.get(cameraType);
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    for (PhotonPipelineResult result : results) {
      double timestamp = result.getTimestampSeconds();

      //todo add object target data
    }
    return objectTargetData;
  }

  @Override
  public void periodic() {
    for (Camera cameraType : DetectionConstants.camerasAvailable) {
      cameras.put(cameraType, new PhotonCamera(cameraType.name()));
    }
  }
}
