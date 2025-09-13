package frc.robot.commands.pidswerve;

import frc.lib.util.LoggedCommandBase;
import frc.robot.subsystems.pose.PoseConstants;
import frc.robot.subsystems.detection.Detection;
import frc.robot.subsystems.detection.Detection.ObjectTargetData;
import frc.robot.subsystems.pose.Pose;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class SwerveToObject extends LoggedCommandBase {
    private final PIDController rotationPID = new PIDController(PIDSwerveConstants.objRotationKP, 0, PIDSwerveConstants.objRotationKD);
    private boolean locked = false;
    private Translation2d lockedPosition;

    public SwerveToObject() {
        super();

        addRequirements(Swerve.instance);
        rotationPID.enableContinuousInput(-180.0, 180.0);
    }

    @Override
    public void initialize() {
        super.initialize();

        rotationPID.reset();
        locked = false;
        lockedPosition = null;
        DogLog.log("SwerveToObject/Status", "Starting");
    }

    double locationToSpeed(double pitch, double yaw) {
        double speed = 0.20;
        double absyaw = Math.abs(yaw);

        if (pitch > 10.0) {
            speed += (pitch - 10.0) / 15.0 * 0.10;
        }

        if (absyaw > 7.0) {
            if (absyaw > 11.0) {
                if (absyaw > 17.0) {
                    speed *= 0.15;
                } else {
                    speed *= 0.5;
                }
            } else {
                speed *= 0.75;
            }
        }

        return speed;
    }

    @Override
    public void execute() {
        Pose2d pose = Pose.instance.getPose();
        Translation2d position = pose.getTranslation();

        ObjectTargetData recentObject = Detection.instance.getRecentObject();
        double pitch = recentObject.pitch();
        double yaw = recentObject.yaw();
        double correction = rotationPID.calculate(yaw);
        double feedForward = PoseConstants.rotationKS * Math.signum(correction);
        double rotationVal = MathUtil.clamp(correction + feedForward, -1.0, 1.0);
        double speed = locationToSpeed(pitch, yaw);

        DogLog.log("SwerveToObject/Rot correction", correction);
        DogLog.log("SwerveToObject/Rot feedforward", feedForward);
        DogLog.log("SwerveToObject/Rot value", rotationVal);
        DogLog.log("SwerveToObject/Rot error", rotationPID.getError());
        DogLog.log("SwerveToObject/Rot error derivative", rotationPID.getErrorDerivative());
        DogLog.log("SwerveToObject/Locked", locked);
        DogLog.log("SwerveToObject/Locked Position", lockedPosition);
        DogLog.log("SwerveToObject/Object pitch", pitch);
        DogLog.log("SwerveToObject/Object yaw", yaw);
        DogLog.log("SwerveToObject/Speed", speed);

        if (!locked && pitch < PIDSwerveConstants.lockPitch && Math.abs(yaw) < PIDSwerveConstants.lockYaw) {
            DogLog.log("SwerveToObject/Status", "Locked");
            locked = true;
            lockedPosition = position;
        }

        if (locked) {
            rotationVal = 0.0;
        } else if (!Detection.instance.haveRecentObject()) {
            speed = rotationVal = 0.0;
        }

        /* Drive */
        Swerve.instance.driveRobotRelativeAuto(
            new ChassisSpeeds(speed * SwerveConstants.maxSpeed, 0.0, rotationVal * PIDSwerveConstants.maxAngularVelocity));
    }

    @Override
    public boolean isFinished() {
        return (locked && (Math.abs(Pose.instance.getPose().getTranslation().getDistance(lockedPosition)) >= PIDSwerveConstants.distanceAfterLock));
    }
}