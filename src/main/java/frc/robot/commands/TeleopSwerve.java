package frc.robot.commands;

import frc.lib.util.LoggedCommandBase;
import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimbState;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.pose.Pose;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Options.*;

public class TeleopSwerve extends LoggedCommandBase {
    private final Swerve s_Swerve;
    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;
    private final DoubleSupplier rotationSup;
    private DoubleSupplier speedLimitSupplier;
    private boolean autoAiming = false;
    private Rotation2d lastAngle = null;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, DoubleSupplier speedLimitSupplier) {
        super();

        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.speedLimitSupplier = speedLimitSupplier;
    }

    @Override
    public void execute() {
        super.execute();

        if (optServiceMode.get()) {
            return;
        }
        
        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        // TODO Get *every* time?
        double translationExpo = SmartDashboard.getNumber("TeleOp Translation Expo", 1.0);
        double teleOpMult = SmartDashboard.getNumber("TeleOp Speed Governor", 1.0);
        double rotationExpo = SmartDashboard.getNumber("TeleOp Rotation Expo", 1.0);
        
        if (translationExpo != 1.0) {
            translationExpo = Math.abs(Math.pow(Math.abs(translationVal), translationExpo)) * Math.signum(rotationVal);
        }
        translationVal *= teleOpMult;
        
        if (rotationExpo != 1.0) {
            rotationVal = Math.abs(Math.pow(Math.abs(rotationVal), rotationExpo)) * Math.signum(rotationVal);
        }

        // Driver position is inverted for Red alliance, so adjust field-oriented controls
        if (Robot.isRed()) {
            translationVal *= -1.0;
            strafeVal *= -1.0;
        }

        // Automatically aim at reef when holding coral
        if (optAutoReefAiming.get() && Math.abs(rotationVal) < Constants.aimingOverride && EndEffector.instance.haveCoral()) {
            Pose2d pose = Pose.instance.getPose();
            Translation2d position = pose.getTranslation();
            Rotation2d rotation = pose.getRotation();
            if (Pose.inWing(position)) {
                Rotation2d bearing = Pose.reefBearing(position);
                
                if (!autoAiming) {
                    Pose.angleErrorReset();
                    autoAiming = true;
                } else {
                    Rotation2d angleError = bearing.minus(lastAngle);
                    rotationVal = Pose.angleErrorToSpeed(angleError);
                }
                lastAngle = rotation;
            } else {
                autoAiming = false;
            }
        } else {
            autoAiming = false;
        }

        /* Drive */
        s_Swerve.drive(
            // new Translation2d(translationVal, strafeVal).times(speedLimitSupplier.getAsDouble()).times(SwerveConstants.maxSpeed).times((RobotState.intakingAlgae() || (RobotState.getClimbState() != ClimbState.NONE)) ? SwerveConstants.slowMode : 1.0),
            // rotationVal * SwerveConstants.maxAngularVelocity * speedLimitSupplier.getAsDouble() * (RobotState.haveAlgae() ? Constants.algaeSlowRot : 1.0) * ((RobotState.intakingAlgae() || (RobotState.getClimbState() != ClimbState.NONE)) ? SwerveConstants.slowMode : 1.0),
            new Translation2d(translationVal, strafeVal).times(speedLimitSupplier.getAsDouble()).times(SwerveConstants.maxSpeed)
                .times((Elevator.instance.shouldLimitSpeed() || Climber.instance.getClimbState() != ClimbState.NONE) ? SwerveConstants.slowMode : 1.0)
                .times(Intake.instance.intaking() ? IntakeConstants.intakeSlowMode : 1.0),
            rotationVal * SwerveConstants.maxAngularVelocity * speedLimitSupplier.getAsDouble(),
            true
        );
    }
}