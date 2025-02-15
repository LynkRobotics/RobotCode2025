package frc.robot.commands;

import frc.robot.Constants;
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

public class PIDSwerve extends LoggedCommandBase {
    private final Swerve s_Swerve;
    private final PoseSubsystem s_Pose;
    private final Pose2d targetPose;

    private final PIDController xPID = new PIDController(0.08, 0, 0); //TODO Make Constants
    private final PIDController yPID = new PIDController(0.08, 0, 0);
    private final double positionTolerance = 1.0; // inches
    private final double maxSpeed = Constants.Swerve.maxSpeed / 5.0;
    private final double positionKS = 0.02;
    private final double positionIZone = 4.0;

    private final PIDController rotationPID = new PIDController(0.003, 0, 0);
    private final double rotationTolerance = 1.0; // degrees
    private final double maxAngularVelocity = Constants.Swerve.maxAngularVelocity / 2.0;

    public PIDSwerve(Swerve s_Swerve, PoseSubsystem s_Pose, Pose2d targetPose) {
        super();

        this.s_Swerve = s_Swerve;
        this.s_Pose = s_Pose;
        this.targetPose = targetPose;
        addRequirements(s_Swerve);

        xPID.setIZone(positionIZone); // Only use Integral term within this range
        xPID.setIntegratorRange(-positionKS * 2, positionKS * 2);
        xPID.setSetpoint(Units.metersToInches(targetPose.getX()));
        xPID.setTolerance(positionTolerance);

        yPID.setIZone(positionIZone); // Only use Integral term within this range
        yPID.setIntegratorRange(-positionKS * 2, positionKS * 2);
        yPID.setSetpoint(Units.metersToInches(targetPose.getY())); // TODO Set derivative, too
        yPID.setTolerance(positionTolerance);

        rotationPID.enableContinuousInput(-180.0, 180.0);
        rotationPID.setIZone(Pose.rotationIZone); // Only use Integral term within this range
        rotationPID.setIntegratorRange(-Pose.rotationKS * 2, Pose.rotationKS * 2);
        rotationPID.setSetpoint(targetPose.getRotation().getDegrees());
        rotationPID.setTolerance(rotationTolerance); // TODO Set derivative, too
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
        DogLog.log("PIDSwerve/X position", Units.metersToInches(position.getX()));
        DogLog.log("PIDSwerve/X correction", xCorrection);
        DogLog.log("PIDSwerve/X feedforward", xFeedForward);
        DogLog.log("PIDSwerve/X value", xVal);

        double yCorrection = yPID.calculate(Units.metersToInches(position.getY()));
        double yFeedForward = positionKS * Math.signum(yCorrection);
        double yVal = MathUtil.clamp(yCorrection + yFeedForward, -1.0, 1.0);
        DogLog.log("PIDSwerve/Y position", Units.metersToInches(position.getY()));
        DogLog.log("PIDSwerve/Y correction", yCorrection);
        DogLog.log("PIDSwerve/Y feedforward", yFeedForward);
        DogLog.log("PIDSwerve/Y value", yVal);

        double correction = rotationPID.calculate(rotation.getDegrees());
        double feedForward = Pose.rotationKS * Math.signum(correction);
        double rotationVal = MathUtil.clamp(correction + feedForward, -1.0, 1.0);

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
}