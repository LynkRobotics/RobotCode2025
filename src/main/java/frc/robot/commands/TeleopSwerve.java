package frc.robot.commands;

import frc.lib.util.TunableOption;
import frc.robot.Constants;
import frc.robot.subsystems.PoseSubsystem;
import frc.robot.Robot;

import frc.robot.subsystems.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TeleopSwerve extends LoggedCommandBase {
    private final Swerve s_Swerve;
    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;
    private final DoubleSupplier rotationSup;
    private DoubleSupplier speedLimitRotSupplier;
    private PoseSubsystem s_Pose = null;
    private boolean autoAiming = false;
    private Rotation2d lastAngle = null;
    private static final TunableOption optAutoReefAiming = new TunableOption("Automatically Aim at Reef", false);

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, DoubleSupplier speedLimitRotSupplier) {
        super();

        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.speedLimitRotSupplier = speedLimitRotSupplier;
    }

    @Override
    public void execute() {
        super.execute();
        
        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        double teleOpMult = SmartDashboard.getNumber("TeleOp Speed Governor", 1.0);
        translationVal *= teleOpMult;

        if (s_Pose == null) {
            s_Pose = PoseSubsystem.getInstance();
        }

        // Driver position is inverted for Red alliance, so adjust field-oriented controls
        if (Robot.isRed()) {
            translationVal *= -1.0;
            strafeVal *= -1.0;
        }

        // Automatically aim at reef when applicable
        // TODO Add manual override ability to disable automatic reef aiming
        if (optAutoReefAiming.get() && Math.abs(rotationVal) < Constants.aimingOverride) {
            Pose2d pose = s_Pose.getPose();

            // TODO Add override when working with algae
            Translation2d position = pose.getTranslation();
            Rotation2d rotation = pose.getRotation();
            if (PoseSubsystem.inWing(position)) {
                Rotation2d bearing = PoseSubsystem.reefBearing(position);
                
                if (!autoAiming) {
                    PoseSubsystem.angleErrorReset();
                    autoAiming = true;
                } else {
                    Rotation2d angleError = bearing.minus(lastAngle);
                    rotationVal = PoseSubsystem.angleErrorToSpeed(angleError);
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
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
            rotationVal * Constants.Swerve.maxAngularVelocity * speedLimitRotSupplier.getAsDouble(), 
            true
        );
        // SmartDashboard.putNumber("rotationValue", speedLimitRotSupplier.getAsDouble());
    }
}