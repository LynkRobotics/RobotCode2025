package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;

import choreo.auto.AutoFactory;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.util.LoggedCommands;
import frc.robot.subsystems.pose.Pose;
import frc.robot.subsystems.swerve.Swerve;

public class Autos extends SubsystemBase {
    public static final Autos instance = new Autos();
    private Command autoCommand;
    private final AutoFactory autoFactory;

    public Autos() {
        autoCommand = LoggedCommands.sequence("Debug Drive",
            LoggedCommands.proxy(PathCommand("Debug Drive")),
            LoggedCommands.proxy(Swerve.instance.Stop()));    


        FollowPathCommand.warmupCommand().schedule();

        //CHOREO
        autoFactory = new AutoFactory(
        Pose.instance::getPose,
        Pose.instance::setPose, 
        Swerve.instance::followTrajectory, 
        true, 
        Swerve.instance);
    }
    
    public Command getAutonomousCommand() {
        return ChoreoTest();
    }

    //PATHPLANNER
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

    private Command ChoreoTest() {
        return LoggedCommands.sequence("Auto/Status/DebugDrive Running", 
            autoFactory.resetOdometry("DebugDrive"), //TODO: choreo has the option of resetting your odom to a certain pose, instead of hacking at robotInit
            LoggedCommands.log("Auto/Status/Running Debug Drive Choreo Path"),
            autoFactory.trajectoryCmd("DebugDrive"),
            LoggedCommands.log("Auto/Status/Finished Debug Drive Choreo Path"),
            Commands.runOnce(Swerve.instance::stopSwerve)
        );
    }
}