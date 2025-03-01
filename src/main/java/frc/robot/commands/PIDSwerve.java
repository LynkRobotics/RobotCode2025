package frc.robot.commands;

import frc.lib.util.LoggedCommandBase;
import frc.robot.Constants.Pose;
import frc.robot.subsystems.PoseSubsystem;
import frc.robot.subsystems.Swerve;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import static frc.robot.Constants.PIDSwerve.*;

public class PIDSwerve extends LoggedCommandBase {
    private final Swerve s_Swerve;
    private final PoseSubsystem s_Pose;
    private final Pose2d targetPose;
    private final boolean precise;
    private final PIDController xPID, yPID;
    private final PIDController rotationPID = new PIDController(rotationKP, 0, 0);

    public PIDSwerve(Swerve s_Swerve, PoseSubsystem s_Pose, Pose2d targetPose, boolean flipIfRed, boolean precise) {
        super();

        if (flipIfRed) {
            targetPose = PoseSubsystem.flipIfRed(targetPose);
        }

        this.s_Swerve = s_Swerve;
        this.s_Pose = s_Pose;
        this.targetPose = targetPose;
        this.precise = precise;
        addRequirements(s_Swerve);

        xPID = new PIDController(precise ? translationKP : roughTranslationKP, 0, 0);
        yPID = new PIDController(precise ? translationKP : roughTranslationKP, 0, 0);

        xPID.setIZone(positionIZone); // Only use Integral term within this range
        xPID.setIntegratorRange(-positionKS * 2, positionKS * 2);
        xPID.setSetpoint(Units.metersToInches(targetPose.getX()));
        xPID.setTolerance(precise ? positionTolerance : roughPositionTolerance);

        yPID.setIZone(positionIZone); // Only use Integral term within this range
        yPID.setIntegratorRange(-positionKS * 2, positionKS * 2);
        yPID.setSetpoint(Units.metersToInches(targetPose.getY())); // TODO Set derivative, too
        yPID.setTolerance(precise ? positionTolerance : roughPositionTolerance);

        rotationPID.enableContinuousInput(-180.0, 180.0);
        rotationPID.setIZone(Pose.rotationIZone); // Only use Integral term within this range
        rotationPID.setIntegratorRange(-Pose.rotationKS * 2, Pose.rotationKS * 2);
        rotationPID.setSetpoint(targetPose.getRotation().getDegrees());
        rotationPID.setTolerance(precise ? rotationTolerance : roughRotatationTolerance); // TODO Set derivative, too
    }

    @Override
    public void initialize() {
        super.initialize();

        xPID.reset();
        yPID.reset();
        rotationPID.reset();

        DogLog.log("PIDSwerve/Pose target", targetPose);
    }

    @Override
    public void execute() {
        Pose2d pose = s_Pose.getPose();
        Translation2d position = pose.getTranslation();
        Rotation2d rotation = pose.getRotation();

        /* TODO Consider a potential need to rotate most of the way first, then translate */

        double xCorrection = xPID.calculate(Units.metersToInches(position.getX()));
        double xFeedForward = positionKS * Math.signum(xCorrection);
        double xVal = MathUtil.clamp(xCorrection + xFeedForward, -1.0, 1.0);
        DogLog.log("PIDSwerve/X position", position.getX());
        DogLog.log("PIDSwerve/X correction", xCorrection);
        DogLog.log("PIDSwerve/X feedforward", xFeedForward);
        DogLog.log("PIDSwerve/X value", xVal);

        double yCorrection = yPID.calculate(Units.metersToInches(position.getY()));
        double yFeedForward = positionKS * Math.signum(yCorrection);
        double yVal = MathUtil.clamp(yCorrection + yFeedForward, -1.0, 1.0);
        DogLog.log("PIDSwerve/Y position", position.getY());
        DogLog.log("PIDSwerve/Y correction", yCorrection);
        DogLog.log("PIDSwerve/Y feedforward", yFeedForward);
        DogLog.log("PIDSwerve/Y value", yVal);

        double correction = rotationPID.calculate(rotation.getDegrees());
        double feedForward = Pose.rotationKS * Math.signum(correction);
        double rotationVal = MathUtil.clamp(correction + feedForward, -1.0, 1.0);
        DogLog.log("PIDSwerve/Rot position", rotation.getDegrees());
        DogLog.log("PIDSwerve/Rot correction", correction);
        DogLog.log("PIDSwerve/Rot feedforward", feedForward);
        DogLog.log("PIDSwerve/Rot value", rotationVal);

        /* Drive */
        s_Swerve.drive(
            new Translation2d(xVal, yVal).times(maxSpeed),
            rotationVal * maxAngularVelocity,
            true
        );
    }

    @Override
    public boolean isFinished() {
        return xPID.atSetpoint() && yPID.atSetpoint() && rotationPID.atSetpoint();
    }

    @Override
    public String getName() {
        return "PID Swerve to " + PoseSubsystem.prettyPose(targetPose) + (precise ? " (precise)" : " (rough)");
    }
}