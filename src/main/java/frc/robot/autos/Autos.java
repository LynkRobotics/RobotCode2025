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

import static frc.robot.Options.optAutoCoralWait;
import static frc.robot.Options.optBackupPush;

import frc.lib.util.LoggedAlert;
import frc.lib.util.LoggedCommands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.pidswerve.PIDSwerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.Stop;
import frc.robot.subsystems.pose.Pose;
import frc.robot.subsystems.pose.PoseConstants.ReefFace;
import frc.robot.subsystems.robotstate.RobotState;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants.CameraMode;
import frc.robot.superstructure.Superstructure;

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

    private Command BackUpCommand() {
        Transform2d transform = new Transform2d(-AutoConstants.backUpPushDistance, 0.0, Rotation2d.kZero); 
        return
            LoggedCommands.race("Backup with timeout",
                LoggedCommands.waitSeconds("Backup timeout", 3), // TODO Make constant
                new PIDSwerve(Swerve.instance, Pose.instance, Pose.instance.getPose().transformBy(transform), false, true));
    }

    @SuppressWarnings ("unused")
    private Command BackUpAndWaitForCoral() {
        Transform2d transform = new Transform2d(-AutoConstants.backUpCSDistance, 0.0, Rotation2d.kZero); 
        return LoggedCommands.deadline("Backup and wait for Coral",
            RobotState.WaitForCoral(),
            Commands.sequence(
                Commands.defer(() -> new PIDSwerve(Swerve.instance, Pose.instance, Pose.instance.getPose().transformBy(transform), false, false), Set.of(Swerve.instance)),
                Commands.defer(() -> new PIDSwerve(Swerve.instance, Pose.instance, Pose.instance.getPose().transformBy(transform), false, false), Set.of(Swerve.instance))
            ));
    }

    @SuppressWarnings ("unused")
    private Command PathWithRaise(String pathName, ReefFace face, boolean left) {
        return LoggedCommands.deadline("Follow Path with Raise",
            PathCommand(pathName),
            Elevator.instance.AutoElevatorUp(left ? face.alignLeft.getTranslation() : face.alignRight.getTranslation())
        ); 
    }

    private Command WaitForReefDistance(double distance) {
        return LoggedCommands.waitUntil("Wait until within " + String.format("%1.2f", distance) + "m of reef center",
            () -> Pose.reefDistance(Pose.instance.getPose().getTranslation()) <= distance);
    }

    private Command RaiseElevatorAtDistance(double distance) {
        return LoggedCommands.sequence("Raise elevator within " + String.format("%1.2f", distance) + "m of reef center",
            WaitForReefDistance(distance),
            Commands.either(
                RobotState.WaitForCoralReady(),
                LoggedCommands.log("Missing coral"),
                RobotState::haveCoral),
            LoggedCommands.proxy(Elevator.instance.GoToNext()));
    }

    private Command WaitForTowardsNext() {
        return LoggedCommands.either("Ensure towards next stop",
            Commands.none(),
            Commands.sequence(
                LoggedCommands.proxy(Swerve.instance.Stop()),
                Elevator.instance.WaitForTowardsNext()),
                Elevator.instance::towardsNextStop);
    }

    private Command FastScoreCoral(String path, ReefFace face, boolean left, double raiseDistance) {
        return LoggedCommands.sequence("Fast coral score following " + path,
            Commands.deadline(
                Commands.sequence(
                    LoggedCommands.proxy(PathCommand(path)),
                    WaitForTowardsNext(),
                    Commands.either(
                        LoggedCommands.proxy(new PIDSwerve(Swerve.instance, Pose.instance, left ? AutoConstants.mirroredFaces.get(face).alignRight : AutoConstants.mirroredFaces.get(face).alignLeft, true, true).fastAlign()),
                        LoggedCommands.proxy(new PIDSwerve(Swerve.instance, Pose.instance, left ? face.alignLeft : face.alignRight, true, true).fastAlign()),
                        Superstructure.instance::shouldMirror
                    ),
                    LoggedCommands.proxy(Swerve.instance.Stop()),
                    Commands.either(
                        Commands.none(),
                        Elevator.instance.WaitForNext(),
                        Elevator.instance::atNextStop)),
                LoggedCommands.proxy(RaiseElevatorAtDistance(raiseDistance))),
            RobotState.ScoreGamePiece());
    }

    private Command MaybeWaitForCoral() {
        return Commands.either(
            LoggedCommands.proxy(RobotState.WaitForCoral()),
            LoggedCommands.log("Will not wait for Coral"),
            optAutoCoralWait::get);
    }

    private Command GoGetCoral(String path) {
        return LoggedCommands.sequence("Go Get Coral following " + path,
            Vision.SwitchToRearVision(),
            Commands.race(
                LoggedCommands.proxy(PathCommand(path)),
                RobotState.WaitForCoral()),
            MaybeWaitForCoral(),
            Vision.SwitchToFrontVision());
    }

    public Command ScoreCoralMaybeMirror(ReefFace face, boolean left) {
        ReefFace mirroredFace = AutoConstants.mirroredFaces.get(face);

        return Commands.either(
            Superstructure.ScoreCoral(mirroredFace, !left),
            Superstructure.ScoreCoral(face, left),
            Superstructure.instance::shouldMirror);
    }

    public Command DealgaefyMaybeMirror(ReefFace face, boolean extendedBackup) {
        ReefFace mirroredFace = AutoConstants.mirroredFaces.get(face);

        return Commands.either(
            Superstructure.DeAlgaefy(mirroredFace, extendedBackup),
            Superstructure.DeAlgaefy(face, extendedBackup),
            Superstructure.instance::shouldMirror);
    }

    public void buildAutos(SendableChooser<Command> chooser) {
        Command autoECDB = LoggedCommands.sequence("Regular Three Piece (ECD+B)",
            Vision.SwitchToFrontVision(),
            LoggedCommands.defer("Startup delay", () -> Commands.waitSeconds(SmartDashboard.getNumber("auto/Startup delay", 0.0)), Set.of()),
            Commands.either(
                LoggedCommands.deferredProxy("Back up push", this::BackUpCommand),
                LoggedCommands.log("Skip back up option"),
                optBackupPush::get),
            Superstructure.instance.SetStop(Stop.L4),
            LoggedCommands.proxy(PathCommand("Start towards EF")),
            LoggedCommands.proxy(ScoreCoralMaybeMirror(ReefFace.EF, true)),
            GoGetCoral("E to CS"),
            LoggedCommands.proxy(PathCommand("CS towards C")),
            LoggedCommands.proxy(ScoreCoralMaybeMirror(ReefFace.CD, true)),
            GoGetCoral("C to CS"),
            LoggedCommands.proxy(PathCommand("CS towards D")),
            LoggedCommands.proxy(ScoreCoralMaybeMirror(ReefFace.CD, false)),
            Swerve.instance.CoastDriveMotors(),
            GoGetCoral("D to CS"),
            LoggedCommands.proxy(PathCommand("CS to near B")),
            LoggedCommands.proxy(ScoreCoralMaybeMirror(ReefFace.AB, false)),
            LoggedCommands.proxy(new PIDSwerve(Swerve.instance, Pose.instance, ReefFace.AB.approachRight, true, false)))
        .handleInterrupt(() -> Vision.setCameraMode(CameraMode.DEFAULT));

        startingPaths.put(autoECDB, "Start towards EF");
        addAutoCommand(chooser, autoECDB);

        Command fastFour = LoggedCommands.sequence("Fast Four Piece (ECDB)",
            LoggedCommands.runOnce("Disable waiting for coral for fast four piece auto", optAutoCoralWait::disable),
            Superstructure.instance.SetStop(Stop.L4),
            Vision.SwitchToFrontVision(),
            LoggedCommands.proxy(FastScoreCoral("Fast - Start to E", ReefFace.EF, true, 2.52)),
            GoGetCoral("Fast - E to CS"),
            LoggedCommands.proxy(FastScoreCoral("Fast - CS to C", ReefFace.CD, true, 3.46)),
            GoGetCoral("Fast - C to CS"),
            LoggedCommands.proxy(FastScoreCoral("Fast - CS to D", ReefFace.CD, false, 3.56)),
            GoGetCoral("Fast - D to CS"),
            LoggedCommands.proxy(FastScoreCoral("Fast - CS to B", ReefFace.AB, false, 2.91)),
            LoggedCommands.proxy(new PIDSwerve(Swerve.instance, Pose.instance, ReefFace.AB.approachMiddle, true, false)))
        .handleInterrupt(() -> Vision.setCameraMode(CameraMode.DEFAULT));

        startingPaths.put(fastFour, "Fast - Start to E");
        addAutoCommand(chooser, fastFour);

        Command autoBA = LoggedCommands.sequence("BA (Outside)",
            LoggedCommands.defer("Startup delay", () -> Commands.waitSeconds(SmartDashboard.getNumber("auto/Startup delay", 0.0)), Set.of()),
            Commands.either(
                LoggedCommands.deferredProxy("Back up push", this::BackUpCommand),
                LoggedCommands.log("Skip back up option"),
                optBackupPush::get),
            Superstructure.instance.SetStop(Stop.L4),
            Vision.SwitchToFrontVision(),
            LoggedCommands.proxy(PathCommand("Start to near B")),
            LoggedCommands.proxy(ScoreCoralMaybeMirror(ReefFace.AB, false)),
            GoGetCoral("B to CS2"),
            Commands.either(
                // At HQ, we need to score on L2 B instead of L4 A, due to space constraints
                Commands.sequence(
                    Superstructure.instance.SetStop(Stop.L2),
                    LoggedCommands.proxy(PathCommand("CS2 to near B")),
                    LoggedCommands.proxy(ScoreCoralMaybeMirror(ReefFace.AB, false))
                ),
                Commands.sequence(
                    LoggedCommands.proxy(PathCommand("CS2 to near A")),
                    LoggedCommands.proxy(ScoreCoralMaybeMirror(ReefFace.AB, true))
                ),
                () -> Constants.atHQ),
            LoggedCommands.proxy(new PIDSwerve(Swerve.instance, Pose.instance, ReefFace.AB.approachMiddle, true, false)),
            LoggedCommands.proxy(Swerve.instance.Stop()))
        .handleInterrupt(() -> Vision.setCameraMode(CameraMode.DEFAULT));

        startingPaths.put(autoBA, "Start to near B");
        addAutoCommand(chooser, autoBA);

        Command autoGBA = LoggedCommands.sequence("GBA (Inside)",
            LoggedCommands.defer("Startup delay", () -> Commands.waitSeconds(SmartDashboard.getNumber("auto/Startup delay", 0.0)), Set.of()),
            Commands.either(
                LoggedCommands.deferredProxy("Back up push", this::BackUpCommand),
                LoggedCommands.log("Skip back up option"),
                optBackupPush::get),
            Superstructure.instance.SetStop(Stop.L4),
            Vision.SwitchToFrontVision(),
            LoggedCommands.proxy(PathCommand("Start to near G")),
            LoggedCommands.proxy(ScoreCoralMaybeMirror(ReefFace.GH, true)),
            GoGetCoral("G to CS2"),
            LoggedCommands.proxy(PathCommand("CS2 to near B")),
            LoggedCommands.proxy(ScoreCoralMaybeMirror(ReefFace.AB, false)),
            GoGetCoral("B to CS2"),
            Commands.either(
                // At HQ, we need to score on L2 B instead of L4 A, due to space constraints
                Commands.sequence(
                    Superstructure.instance.SetStop(Stop.L2),
                    LoggedCommands.proxy(PathCommand("CS2 to near B")),
                    LoggedCommands.proxy(ScoreCoralMaybeMirror(ReefFace.AB, false))
                ),
                Commands.sequence(
                    LoggedCommands.proxy(PathCommand("CS2 to near A")),
                    LoggedCommands.proxy(ScoreCoralMaybeMirror(ReefFace.AB, true))
                ),
                () -> Constants.atHQ),
            LoggedCommands.proxy(new PIDSwerve(Swerve.instance, Pose.instance, ReefFace.AB.approachMiddle, true, false)),
            LoggedCommands.proxy(Swerve.instance.Stop()))
        .handleInterrupt(() -> Vision.setCameraMode(CameraMode.DEFAULT));

        startingPaths.put(autoGBA, "Start to near G");
        addAutoCommand(chooser, autoGBA);

        // NOTE: Do not mirror this auto!
        Command autoG = LoggedCommands.sequence("G + Barge Shots (don't mirror!)",
            LoggedCommands.defer("Startup delay", () -> Commands.waitSeconds(SmartDashboard.getNumber("auto/Startup delay", 0.0)), Set.of()),
            Commands.either(
                LoggedCommands.deferredProxy("Back up push", this::BackUpCommand),
                LoggedCommands.log("Skip back up option"),
                optBackupPush::get),
                Superstructure.instance.SetStop(Stop.L4),
            LoggedCommands.proxy(PathCommand("Start to near G")),
            LoggedCommands.proxy(ScoreCoralMaybeMirror(ReefFace.GH, true)),
            LoggedCommands.proxy(new PIDSwerve(Swerve.instance, Pose.instance, ReefFace.GH.approachMiddle, true, false)),
            LoggedCommands.proxy(DealgaefyMaybeMirror(ReefFace.GH, false)),
            LoggedCommands.proxy(PathCommand("GH to Barge Shot")),
            LoggedCommands.proxy(Superstructure.instance.BargeShot()),
            LoggedCommands.proxy(PathCommand("Barge Shot to near IJ")),
            LoggedCommands.proxy(DealgaefyMaybeMirror(ReefFace.IJ, false)), 
            LoggedCommands.proxy(Superstructure.instance.BargeShot(-Units.inchesToMeters(5))), // Ensure we are shy of the line at the end of auto
            LoggedCommands.deferredProxy("Backup after barge shot", 
                () -> new PIDSwerve(Swerve.instance, Pose.instance, Pose.instance.getPose().transformBy(new Transform2d(-Units.inchesToMeters(18.0), 0.0, Rotation2d.kZero)), false, false)),
            LoggedCommands.proxy(Swerve.instance.Stop()));

        startingPaths.put(autoG, "Start to near G");
        addAutoCommand(chooser, autoG);

        FollowPathCommand.warmupCommand().schedule();
    }

    private Command PathCommand(String pathName) {
        Command pathCommand, mirrorCommand;
        
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            PathPlannerPath mirror = path.mirrorPath();

            pathCommand = AutoBuilder.followPath(path);
            pathCommand.setName("Follow PathPlanner path \"" + pathName + "\"");
            startingPoses.put(pathName, new Pose2d(path.getPathPoses().get(0).getTranslation(), path.getIdealStartingState().rotation()));

            mirrorCommand = AutoBuilder.followPath(mirror);
            mirrorCommand.setName("Follow Mirrored PathPlanner path \"" + pathName + "\"");
            startingPoses.put(pathName + " - Mirror", new Pose2d(mirror.getPathPoses().get(0).getTranslation(), mirror.getIdealStartingState().rotation()));
        } catch (Exception exception) {
            LoggedAlert.Error("PathPlanner", "Failed to load path \"" + pathName + "\"", exception.getMessage());
            return LoggedCommands.log("Missing PathPlanner path due to failure to load \"" + pathName + "\": " + exception.getMessage());
        }

        return LoggedCommands.either("Choosing auto path for " + pathName,
            LoggedCommands.logWithName("Mirrored Path: " + pathName, mirrorCommand),
            LoggedCommands.logWithName("Path: " + pathName, pathCommand),
            Superstructure.instance::shouldMirror);
    }

    @Override
    public void periodic() {
        if (DriverStation.isEnabled()) {
            return;
        }

        Command autoCommand = getAutonomousCommand();
        String poseDifference = "N/A";
        boolean differenceOK = false;

        if (autoCommand != null) {
            String firstPath = startingPaths.get(autoCommand);

            if (firstPath != null) {
                Pose2d startingPose = startingPoses.get(firstPath + (Superstructure.instance.shouldMirror() ? " - Mirror" : ""));

                if (startingPose != null) {
                    Pose2d currentPose = Pose.instance.getPose();
                   
                    poseDifference = String.format("(%1.1f, %1.1f) @ %1.0f deg",
                        Units.metersToInches(currentPose.getX() - startingPose.getX()),
                        Units.metersToInches(currentPose.getY() - startingPose.getY()),
                        startingPose.getRotation().minus(currentPose.getRotation()).getDegrees());

                    if (Math.abs(currentPose.getX() - startingPose.getX()) < AutoConstants.maxSetupXError &&
                        Math.abs(currentPose.getY() - startingPose.getY()) < AutoConstants.maxSetupYError &&
                        Math.abs(startingPose.getRotation().minus(currentPose.getRotation()).getDegrees()) < AutoConstants.maxSetupDegError) {
                        differenceOK = true;
                    }
                }
            }
        }

        SmartDashboard.putString("autoSetup/Starting Pose Error", poseDifference);
        SmartDashboard.putBoolean("autoSetup/Starting Pose OK", differenceOK);
        SmartDashboard.putBoolean("autoSetup/Red Alliance?", Robot.isRed());
    }
}