package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.PoseSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AimCommand extends Command {
    private final Swerve s_Swerve;
    private final VisionSubsystem s_Vision;

    public AimCommand(Swerve s_Swerve, VisionSubsystem s_Vision) {
        this.s_Swerve = s_Swerve;
        this.s_Vision = s_Vision;
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        PoseSubsystem.angleErrorReset();
    }

    @Override
    public void execute() {
        // TODO Remove code duplication with TeleopSwerve (or need for this entirely)
        Rotation2d angleError = s_Vision.angleError();
        double rotationVal = PoseSubsystem.angleErrorToSpeed(angleError);
        if (!s_Vision.haveTarget()) {
            rotationVal = 0.0;
        }
        /* Drive */
        s_Swerve.drive(
            new Translation2d(0, 0),
            rotationVal * Constants.Swerve.maxAngularVelocity,
            true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        s_Swerve.drive(new Translation2d(0, 0), 0.0, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}