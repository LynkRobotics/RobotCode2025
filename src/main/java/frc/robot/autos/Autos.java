package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.util.LoggedCommands;

import frc.robot.subsystems.swerve.Swerve;

public class Autos extends SubsystemBase {
    public static final Autos instance = new Autos();
    private Command autoCommand;

    public Autos() {
        autoCommand = LoggedCommands.sequence("Debug Drive",
            LoggedCommands.proxy(PathCommand("Debug Drive")),
            LoggedCommands.proxy(Swerve.instance.Stop()));    


        FollowPathCommand.warmupCommand().schedule();
    }
    
    public Command getAutonomousCommand() {
        return autoCommand;
    }

    private Command PathCommand(String pathName) {
        Command pathCommand;
        
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

            pathCommand = AutoBuilder.followPath(path);
            pathCommand.setName("Follow PathPlanner path \"" + pathName + "\"");
        } catch (Exception exception) {
            return LoggedCommands.log("Missing PathPlanner path due to failure to load \"" + pathName + "\": " + exception.getMessage());
        }

        return LoggedCommands.logWithName("Path: " + pathName, pathCommand);
    }
}