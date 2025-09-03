package frc.robot.autos;

import java.util.HashMap;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Options.optBackupPush;

import frc.lib.util.LoggedAlert;
import frc.lib.util.LoggedCommands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.pose.Pose;
import frc.robot.subsystems.pose.PoseConstants.ReefFace;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants.CameraMode;
import frc.robot.Field.ReefLevel;
import frc.robot.autos.AutoConstants.AutoPose;

public class Autos extends SubsystemBase {
    public static final Autos instance = new Autos();

    private final SendableChooser<Command> autoChooser;
    private final HashMap<Command, String> startingPaths = new HashMap<>();
    private final HashMap<String, Pose2d> startingPoses = new HashMap<>();

    public Autos() {
        // Build an autoChooser (defaults to none)
        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
            (stream) -> stream.filter(auto -> !auto.getName().startsWith("Dummy")));
        SmartDashboard.putData("auto/Auto Chooser", autoChooser);
        buildAutos(autoChooser);
        
        // Default named commands for PathPlanner
        SmartDashboard.putNumber("auto/Startup delay", 0.0);
        Autos.autoNamedCommand("Startup delay", Commands.defer(() -> Commands.waitSeconds(SmartDashboard.getNumber("auto/Startup delay", 0.0)), Set.of()));
        Autos.autoNamedCommand("Stop", Commands.runOnce(Swerve.instance::stopSwerve));
        
        SmartDashboard.putData("auto/Debug Drive", Commands.sequence(
            LoggedCommands.log("Before Path Command"),
            LoggedCommands.proxy(PathCommand("Debug Drive")),
            LoggedCommands.log("After Path Command"),
            LoggedCommands.proxy(Swerve.instance.Stop())));
    }
    
    public static void autoNamedCommand(String name, Command command) {
        NamedCommands.registerCommand(name, LoggedCommands.logWithName(name + " (auto)", command));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void addAutoCommand(SendableChooser<Command> chooser, Command command) {
        chooser.addOption(command.getName(), command);
    }

    public void buildAutos(SendableChooser<Command> chooser) {
       Command autoDebug = LoggedCommands.sequence("Debug Drive",
            LoggedCommands.proxy(PathCommand("Debug Drive")),
            LoggedCommands.proxy(Swerve.instance.Stop()));

        startingPaths.put(autoDebug, "Debug Drive");
        addAutoCommand(chooser, autoDebug);

        FollowPathCommand.warmupCommand().schedule();
    }
    private Command PathCommand(String pathName) {
        Command pathCommand;
        
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

            pathCommand = AutoBuilder.followPath(path);
            pathCommand.setName("Follow PathPlanner path \"" + pathName + "\"");
            startingPoses.put(pathName, new Pose2d(path.getPathPoses().get(0).getTranslation(), path.getIdealStartingState().rotation()));
        } catch (Exception exception) {
            LoggedAlert.Error("PathPlanner", "Failed to load path \"" + pathName + "\"", exception.getMessage());
            return LoggedCommands.log("Missing PathPlanner path due to failure to load \"" + pathName + "\": " + exception.getMessage());
        }

        return LoggedCommands.logWithName("Path: " + pathName, pathCommand);
    }

    @Override
    public void periodic() {
        if (DriverStation.isEnabled()) {
            return;
        }

        Command autoCommand = getAutonomousCommand();
        String poseDifference = "N/A";
        boolean differenceOK = false;

        SmartDashboard.putString("autoSetup/Starting Pose Error", poseDifference);
        SmartDashboard.putBoolean("autoSetup/Starting Pose OK", differenceOK);
        SmartDashboard.putBoolean("autoSetup/Red Alliance?", Robot.isRed());
    }
}