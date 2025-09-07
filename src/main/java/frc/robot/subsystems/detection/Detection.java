// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.detection;

import java.util.LinkedList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedCommands;
import frc.robot.subsystems.swerve.Swerve;

public class Detection extends SubsystemBase {
    public static final Detection instance = new Detection();

    private static final PhotonCamera camera = new PhotonCamera(DetectionConstants.cameraName);

    public record ObjectTargetData(double timestamp, double confidence, double pitch, double yaw) {}

    private ObjectTargetData recentObject = null;

    private List<ObjectTargetData> processCamera() {
        List<ObjectTargetData> objectTargetData = new LinkedList<>();
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        for (PhotonPipelineResult result : results) {
            PhotonTrackedTarget target = result.getBestTarget();
            if (target == null) continue;
            if (target.getDetectedObjectConfidence() < DetectionConstants.requiredConfidence) continue;
            if (target.objDetectId != 1) continue; // Only track coral
            objectTargetData.add(new ObjectTargetData(result.getTimestampSeconds(), target.getDetectedObjectConfidence(), target.pitch, target.yaw));
        }
        return objectTargetData;
    }

    public boolean haveRecentObject() {
        return recentObject != null && (Timer.getTimestamp() - recentObject.timestamp) < DetectionConstants.recentObjectTimeout;
    }

    public ObjectTargetData getRecentObject() {
        return recentObject;
    } 

    public Command WaitForObject() {
        return LoggedCommands.waitUntil("Wait for object", this::haveRecentObject);
    }

    public Command StopUntilObject() {
        return Commands.either(
            LoggedCommands.log("Already have recent object"),
            LoggedCommands.sequence("Stop to wait for object",
                Swerve.instance.Stop(),
                LoggedCommands.runOnce("Take photo without object", camera::takeOutputSnapshot),
                LoggedCommands.waitUntil("Wait for object", this::haveRecentObject)),
            this::haveRecentObject);
    }

    @Override
    public void periodic() {
        String logPrefix = "Detection/";
        for (ObjectTargetData objectTargetData : processCamera()) { // TODO Just most recent?
            DogLog.log(logPrefix + "Timestamp", objectTargetData.timestamp);
            DogLog.log(logPrefix + "Pitch", objectTargetData.pitch);
            DogLog.log(logPrefix + "Yaw", objectTargetData.yaw);
            DogLog.log(logPrefix + "Confidence", objectTargetData.confidence);
            recentObject = objectTargetData;
        }
        DogLog.log(logPrefix + "Recent", haveRecentObject());
    }
}