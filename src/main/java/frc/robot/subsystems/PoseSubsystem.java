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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableOption;
import frc.robot.Constants;
import frc.robot.Constants.Pose;
import frc.robot.Robot;

public class PoseSubsystem extends SubsystemBase {
    private static PoseSubsystem instance;
    private final Swerve s_Swerve;
    private final VisionSubsystem s_Vision;

    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field;
    private final Pigeon2 gyro;
    private static Rotation2d targetAngle = null;
    private static Zone zone = Zone.SPEAKER;

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

    public void zeroGyro() {
        gyro.setYaw(0);
        DogLog.log("Pose/Gyro/Status", "Zeroed Gyro Yaw");
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

    public void zeroHeading() {
        setHeading(new Rotation2d());
        DogLog.log("Pose/Gyro/Status", "Zeroed Gyro Heading");
    }

    public void resetHeading() {
        if (Robot.isRed()) {
            setHeading(new Rotation2d(Math.PI));
        } else {
            setHeading(new Rotation2d());
        }
    }

    public Translation2d ampLocation() {
        return (Robot.isRed() ? Pose.redAmpLocation : Pose.blueAmpLocation);
    }

    public Translation2d speakerLocation() {
        return (Robot.isRed() ? Pose.redSpeakerLocation : Pose.blueSpeakerLocation);
    }

    public Translation2d shuttleLocation() {
        return (Robot.isRed() ? Pose.redShuttleLocation : Pose.blueShuttleLocation);
    }

    public Translation2d farShuttleLocation() {
        return (Robot.isRed() ? Pose.redFarShuttleLocation : Pose.blueFarShuttleLocation);
    }

    public double distanceToSpeaker() {
        double distance = getPose().getTranslation().getDistance(PoseSubsystem.getInstance().speakerLocation()); // distance from center of robot to speaker 
        distance += Constants.Vision.centerToReferenceOffset; // distance from center of robot to reference point
        return distance;
    }

    public double distanceToShuttle() {
        double distance = getPose().getTranslation().getDistance(PoseSubsystem.getInstance().shuttleLocation()); // distance from center of robot to shuttle location
        distance += Constants.Vision.centerToReferenceOffset; // distance from center of robot to reference point
        return distance;
    }

    public double distanceToFarShuttle() {
        double distance = getPose().getTranslation().getDistance(PoseSubsystem.getInstance().farShuttleLocation()); // distance from center of robot to far shuttle location
        distance += Constants.Vision.centerToReferenceOffset; // distance from center of robot to reference point
        return distance;
    }

    public Rotation2d dumpShotError() {
        Rotation2d robotAngle = getPose().getRotation();
        if (Robot.isRed()){
            return Pose.redDumpAngle.minus(robotAngle);
        } else {
            return Pose.blueDumpAngle.minus(robotAngle);
        }
    }

    public boolean dumpShotAligned() {
        if (Math.abs(dumpShotError().getDegrees()) < Pose.maxDumpError) {
            if (Math.abs(Pose.rotationPID.getVelocityError()) < Constants.Shooter.dumpShotVelocityErrorMax) {
                return true;
            } else {
                return false;
            }
        }
        return false;
    }

    public Rotation2d slideShotError() {
        Rotation2d robotAngle = getPose().getRotation();
        if (Robot.isRed()){
            return Pose.redSlideAngle.minus(robotAngle);
        } else {
            return Pose.blueSlideAngle.minus(robotAngle);
        }
    }

    public boolean slideShotAligned() {
        if (Math.abs(slideShotError().getDegrees()) < Pose.maxSlideError) {
            if (Math.abs(Pose.rotationPID.getVelocityError()) < Constants.Shooter.slideShotVelocityErrorMax) {
                return true;
            } else {
                return false;
            }
        }
        return false;
    }

    public Rotation2d shuttleShotError() {
        Rotation2d robotAngle = getPose().getRotation();
        Rotation2d targetAngle = angleToShuttle();
        return targetAngle.minus(robotAngle);
    }

    public boolean shuttleShotAligned() {
        if (Math.abs(shuttleShotError().getDegrees()) < Pose.maxShuttleError) {
            if (Math.abs(Pose.rotationPID.getVelocityError()) < Constants.Shooter.shuttleShotVelocityErrorMax) {
                return true;
            } else {
                return false;
            }
        }
        return false;
    }

    public Rotation2d farShuttleShotError() {
        Rotation2d robotAngle = getPose().getRotation();
        Rotation2d targetAngle = angleToFarShuttle();
        return targetAngle.minus(robotAngle);
    }

    public boolean farShuttleShotAligned() {
        if (Math.abs(farShuttleShotError().getDegrees()) < Pose.maxShuttleError) {
            if (Math.abs(Pose.rotationPID.getVelocityError()) < Constants.Shooter.shuttleShotVelocityErrorMax) {
                return true;
            } else {
                return false;
            }
        }
        return false;
    }

    public static void angleErrorReset() {
        angleErrorReset(Pose.rotationPID);
    }

    public static void angleErrorReset(PIDController pid) {
        pid.reset();
    }

    private Translation2d speakerOffset() {
        return getPose().getTranslation().minus(speakerLocation());
    }

    public Rotation2d angleToSpeaker() {
        return speakerOffset().getAngle();
    }

    public Rotation2d angleToAmp() {
        return ampOffset().getAngle();
    }

    private Translation2d ampOffset() {
        return getPose().getTranslation().minus(ampLocation());
    }

    private Translation2d shuttleOffset() {
        return getPose().getTranslation().minus(shuttleLocation());
    }

    public Rotation2d angleToShuttle() {
        return shuttleOffset().getAngle();
    }

    private Translation2d farShuttleOffset() {
        return getPose().getTranslation().minus(farShuttleLocation());
    }

    public Rotation2d angleToFarShuttle() {
        return farShuttleOffset().getAngle();
    }

    public Rotation2d angleError() {
        Rotation2d speakerAngle = angleToSpeaker();
        Rotation2d robotAngle = getPose().getRotation();

        return speakerAngle.minus(robotAngle);
    }

    public static void setTargetAngle(Rotation2d angle) {
        targetAngle = angle;
    }
    
    public static Rotation2d getTargetAngle() {
        return targetAngle;
    }

    public static double angleErrorToSpeed(Rotation2d angleError) {
        return angleErrorToSpeed(angleError, Pose.rotationPID);
    }

    public static double angleErrorToSpeed(Rotation2d angleError, PIDController pid) {
        double angleErrorDeg = angleError.getDegrees();
        double correction = pid.calculate(angleErrorDeg);
        double feedForward = Pose.rotationKS * Math.signum(correction);
        double output = MathUtil.clamp(correction + feedForward, -1.0, 1.0);

        DogLog.log("Pose/Angle Error", angleErrorDeg);
        DogLog.log("Pose/Angle PID correction", correction);
        DogLog.log("Pose/Angle feedforward", feedForward);
        DogLog.log("Pose/Angle output", output);
        
        // Invert due to use as joystick controls
        return -output;
    }

    public static Zone getZone() {
        return zone;
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
        double distance = pose.getTranslation().getX();
        if (Robot.isRed()) {
            distance = Constants.Pose.fieldLength - distance;
        }
        if (distance > Constants.Pose.zoneSourceStart) {
            if (distance > Constants.Pose.zoneMiddleEnd || zone != Zone.MIDDLE) {
                zone = Zone.FAR;
            }
        } else if (distance < Constants.Pose.zoneSpeakerEnd) {
            if (distance < Constants.Pose.zoneMiddleStart || zone != Zone.MIDDLE) {
                zone = Zone.SPEAKER;
            }
        } else {
            zone = Zone.MIDDLE;
        }
        DogLog.log("Pose/Zone", zone);
        SmartDashboard.putString("pose/Zone", zone.toString());

        SmartDashboard.putNumber("pose/Gyro", getHeading().getDegrees());
        SmartDashboard.putString("pose/Pose", prettyPose(pose));

        DogLog.log("Pose/Pose", pose);
        DogLog.log("Pose/Distance to speaker", Units.metersToInches(distanceToSpeaker()));
        DogLog.log("Pose/Distance to shuttle", Units.metersToInches(distanceToShuttle()));
        DogLog.log("Pose/Distance to far shuttle", Units.metersToInches(distanceToFarShuttle()));
        DogLog.log("Pose/Gyro/Heading", getHeading().getDegrees());
        DogLog.log("Pose/Gyro/Raw Yaw", getGyroYaw());
    }
}