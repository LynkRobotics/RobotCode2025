package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

import dev.doglog.DogLog;

import frc.robot.subsystems.Swerve;

public class CoastAfterAuto extends Command {
    private final Swerve s_Swerve;
    private final double cutoff = 0.1;
    private boolean triggered = false;

    public CoastAfterAuto(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        // NOTE: Do NOT use Swerve as a requirement
    }

    @Override
    public void execute() {
        if (!triggered && DriverStation.isAutonomousEnabled() && DriverStation.getMatchTime() <= cutoff) {
            DogLog.log("Auto/Status", "Coasting swerve drive motors");
            triggered = true;
            s_Swerve.setDriveMotorsToCoast();
            DogLog.log("Auto/Status", "Coasted swerve drive motors");
        }
    }

    @Override
    public boolean isFinished() {
        return (triggered || !DriverStation.isAutonomousEnabled());
    }
}