package frc.robot.subsystems.algaeroller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedCommands;

public class AlgaeRoller extends SubsystemBase {
    public static final AlgaeRoller instance = new AlgaeRoller();
    
    AlgaeRoller() {

    }

    public static boolean fullyDeployed() {
        return false; // TODO
    }

    public static Command Intake() {
        return LoggedCommands.print("Intake algae", "TODO Implement intake algae");
    }

    public static Command GuideL1Coral() {
        return LoggedCommands.print("Guide L1 coral", "TODO Implement guide L1 coral");
    }

    public static Command ClearElevatorPath() {
        return LoggedCommands.print("Clear elevator path", "TODO Implement clear elevator path");
    }
}