package frc.robot;

import static frc.robot.Options.optAutoReefAiming;
import static frc.robot.Options.optBackupPush;
import static frc.robot.Options.optBonusCoralStandoff;
import static frc.robot.Options.optMirrorAuto;

import java.util.Collections;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.LoggedAlert;
import frc.lib.util.LoggedCommands;
import frc.robot.Constants.Pose.ReefFace;
import frc.robot.Constants.Elevator.Stop;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LEDSubsystem.TempState;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);

    /* Drive Controls */
    private final Supplier<Double> translation = driver::getLeftY;
    private final Supplier<Double> strafe = driver::getLeftX;
    private final Supplier<Double> rotation = driver::getRightX;

    /* Subsystems */
    @SuppressWarnings ("unused")
    private final RobotState s_RobotState;
    private final Swerve s_Swerve;
    @SuppressWarnings ("unused")
    private final LEDSubsystem s_LED;
    private final VisionSubsystem s_Vision;
    private final PoseSubsystem s_Pose;
    private final ElevatorSubsystem s_Elevator;
    @SuppressWarnings ("unused")
    private final EndEffectorSubsystem s_EndEffector;
    @SuppressWarnings ("unused")
    private final IndexSubsystem s_Index;

    /* Autonomous Control */
    private final SendableChooser<Command> autoChooser;

    EnumMap<ReefFace, Command> alignLeftCommands = new EnumMap<>(ReefFace.class);
    EnumMap<ReefFace, Command> alignRightCommands = new EnumMap<>(ReefFace.class);
    EnumMap<ReefFace, Command> deAlgaefyLeftCommands = new EnumMap<>(ReefFace.class);
    EnumMap<ReefFace, Command> deAlgaefyRightCommands = new EnumMap<>(ReefFace.class);

    private final HashMap<Command, String> startingPaths = new HashMap<>();
    private final HashMap<String, Pose2d> startingPoses = new HashMap<>();

    private static void autoNamedCommand(String name, Command command) {
        NamedCommands.registerCommand(name, LoggedCommands.logWithName(name + " (auto)", command));
    }

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        DogLog.setOptions(
            new DogLogOptions()
            .withCaptureConsole(true)
            .withCaptureDs(true)
            .withCaptureNt(true)
            .withLogEntryQueueCapacity(1000)
            .withLogExtras(true)
            .withNtPublish(Constants.atHQ));

        DogLog.log("Misc/RIO Serial Number", RobotController.getSerialNumber());
        DogLog.log("Misc/Is Rocky?", Constants.isRocky);

        // Initial Subsystems
        s_Swerve = new Swerve();
        s_Vision = new VisionSubsystem();
        s_Pose = new PoseSubsystem(s_Swerve, s_Vision);
        s_RobotState = new RobotState();
        s_Elevator = new ElevatorSubsystem();
        s_EndEffector = new EndEffectorSubsystem();
        s_LED = new LEDSubsystem();
        s_Index = new IndexSubsystem();

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> -translation.get() * Constants.driveStickSensitivity,
                () -> -strafe.get() * Constants.driveStickSensitivity,
                () -> -rotation.get() * Constants.turnStickSensitivity,
                this::speedLimitFactor
            ));

        SmartDashboard.putData("Command scheduler", CommandScheduler.getInstance());

        // Default named commands for PathPlanner
        SmartDashboard.putNumber("auto/Startup delay", 0.0);
        autoNamedCommand("Startup delay", Commands.defer(() -> Commands.waitSeconds(SmartDashboard.getNumber("auto/Startup delay", 0.0)), Set.of()));
        autoNamedCommand("Stop", Commands.runOnce(s_Swerve::stopSwerve));

        SmartDashboard.putNumber("TeleOp Speed Governor", 1.0);

        SmartDashboard.putData(LoggedCommands.runOnce("Zero Gyro", s_Pose::zeroGyro, s_Swerve));
        SmartDashboard.putData(LoggedCommands.runOnce("Reset heading", s_Pose::resetHeading, s_Swerve));

        SmartDashboard.putData(LoggedCommands.runOnce("autoSetup/Set Swerve Coast", s_Swerve::setMotorsToCoast, s_Swerve).ignoringDisable(true));
        SmartDashboard.putData(LoggedCommands.runOnce("autoSetup/Set Swerve Brake", s_Swerve::setMotorsToBrake, s_Swerve).ignoringDisable(true));
        SmartDashboard.putData(LoggedCommands.run("autoSetup/Set Swerve Aligned", s_Swerve::alignStraight, s_Swerve).ignoringDisable(true));

        for (ReefFace face: ReefFace.values()) {
            setFaceCommands(face);
        }

        configureButtonBindings();

        // Build an autoChooser (defaults to none)
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("auto/Auto Chooser", autoChooser);
        buildAutos(autoChooser);
    }

    private Command ScoreCoral(ReefFace face, boolean left) {
        return Commands.either(
            LoggedCommands.sequence("Auto Align " + (left ? "Left " : "Right ") + face.toString() + " & Score",
                LoggedCommands.parallel("PID Align " + (left ? "Left " : "Right ") + face.toString(),
                    Commands.sequence(
                        new PIDSwerve(s_Swerve, s_Pose, left ? face.approachLeft : face.approachRight, false),
                        Commands.either(
                            LoggedCommands.log("Elevator reached stop in time"),
                            LoggedCommands.sequence("Pause to wait for elevator to catch up",
                                s_Swerve.Stop(),
                                s_Elevator.WaitForNearNext()),
                            s_Elevator::nearNextStop),
                        Commands.either(
                            new PIDSwerve(s_Swerve, s_Pose, left ? face.alignBonusLeft : face.alignBonusRight, true),
                            new PIDSwerve(s_Swerve, s_Pose, left ? face.alignLeft : face.alignRight, true),
                            optBonusCoralStandoff::get),
                        s_Swerve.Stop()),
                    Commands.sequence(
                        RobotState.WaitForCoralReady(),
                        LoggedCommands.deadline("Wait for auto up",
                            s_Elevator.WaitForNext(),
                            s_Elevator.AutoElevatorUp(left ? face.alignLeft.getTranslation() : face.alignRight.getTranslation())))),
                RobotState.ScoreGamePiece()),
            LoggedCommands.log("Cannot score coral without coral"),
            RobotState::haveCoral);
    }

    private static final Map<ReefFace, ReefFace> mirroredFaces = Collections.unmodifiableMap(Map.ofEntries(
        Map.entry(ReefFace.AB, ReefFace.AB),
        Map.entry(ReefFace.CD, ReefFace.KL),
        Map.entry(ReefFace.EF, ReefFace.IJ),
        Map.entry(ReefFace.GH, ReefFace.GH),
        Map.entry(ReefFace.IJ, ReefFace.EF),
        Map.entry(ReefFace.KL, ReefFace.CD)));

    private Command ScoreCoralMaybeMirror(ReefFace face, boolean left) {
        ReefFace mirroredFace = mirroredFaces.get(face);

        return Commands.either(
            ScoreCoral(mirroredFace, !left),
            ScoreCoral(face, left),
            this::shouldMirror);
    }

    private Command DeAlgaefy(ReefFace face) {
        Stop algaeStop = face.algaeHigh ? Stop.L3_ALGAE: Stop.L2_ALGAE;

        return LoggedCommands.sequence("Auto Align Middle " + face.toString(),
            RobotState.IntakeAlgae(),
            LoggedCommands.parallel("PID Align Middle " + face.toString(),
                Commands.sequence(
                    new PIDSwerve(s_Swerve, s_Pose, face.approachMiddle, false),
                    new PIDSwerve(s_Swerve, s_Pose, face.alignMiddle, true),
                    s_Swerve.Stop()),
                LoggedCommands.deadline("Wait for auto up",
                    s_Elevator.WaitForStop(algaeStop),
                    s_Elevator.AutoElevatorUp(face.alignMiddle.getTranslation(), algaeStop))));
    }
    
    private void setFaceCommands(ReefFace face) {
        alignLeftCommands.put(face, ScoreCoral(face, true));
        alignRightCommands.put(face, ScoreCoral(face, false));
        deAlgaefyLeftCommands.put(face, DeAlgaefy(face));
        deAlgaefyRightCommands.put(face, DeAlgaefy(face));
    }

    private double speedLimitFactor() {
        return 1.0 - s_Elevator.raisedPercentage() * (1.0 - Constants.Elevator.speedLimitAtMax);
    }

    private Command SetStop(Stop stop) {
        return LoggedCommands.sequence("Set stop to " + stop,
            RobotState.SetCoralMode(),
            Commands.runOnce(() -> s_Elevator.setNextStop(stop)));
    }

    private boolean shouldMirror() {
        return optMirrorAuto.get() && DriverStation.isAutonomousEnabled();
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

        return Commands.either(mirrorCommand, pathCommand, this::shouldMirror);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        final Trigger moveElevator = driver.leftBumper();
        final Trigger score = driver.rightBumper();
        final Trigger goLeft = driver.leftTrigger();
        final Trigger goRight = driver.rightTrigger();
        final Trigger L4 = driver.y();
        final Trigger L3 = driver.x();
        final Trigger L2 = driver.b();
        final Trigger L1 = driver.a();
        final Trigger zero = driver.start();
        final Trigger alignmentToggle = driver.rightStick();

        // Only used in case of automation failure
        moveElevator.whileTrue(s_Elevator.GoToNext());
        score.onTrue(RobotState.ScoreGamePiece());
        zero.onTrue(s_Elevator.Zero());

        L4.onTrue(SetStop(Stop.L4));
        L3.onTrue(SetStop(Stop.L3));
        L2.onTrue(SetStop(Stop.L2));
        L1.onTrue(SetStop(Stop.L1));

        goLeft.whileTrue(
            Commands.either(
                Commands.select(alignLeftCommands, () -> PoseSubsystem.nearestFace(s_Pose.getPose().getTranslation())),
                Commands.select(deAlgaefyLeftCommands, () -> PoseSubsystem.nearestFace(s_Pose.getPose().getTranslation())),
                RobotState::haveCoral));

        goRight.whileTrue(
            Commands.either(
                Commands.select(alignRightCommands, () -> PoseSubsystem.nearestFace(s_Pose.getPose().getTranslation())),
                Commands.select(deAlgaefyRightCommands, () -> PoseSubsystem.nearestFace(s_Pose.getPose().getTranslation())),
                RobotState::haveCoral));

        alignmentToggle.onTrue(LoggedCommands.runOnce("Toggle Alignment", optAutoReefAiming::toggle));

        driver.povUp().whileTrue(
            Commands.sequence(
                new PIDSwerve(s_Swerve, s_Pose, new Pose2d(4.48, 1.67, Rotation2d.fromDegrees(91)), false),
                s_Swerve.Stop(),
                Commands.runOnce(() -> LoggedAlert.Info("Debug", "In Position", "Reached Debug Position")),
                Commands.runOnce(() -> LEDSubsystem.setTempState(TempState.ERROR))));
        driver.povDown().whileTrue(RobotState.UnjamCoral());
    }

    /** 
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void addAutoCommand(SendableChooser<Command> chooser, Command command) {
        chooser.addOption(command.getName(), command);
    }

    private Command BackUpCommand() {
        Transform2d transform = new Transform2d(-Constants.AutoConstants.backUpPushDistance, 0.0, Rotation2d.kZero); 
        return new PIDSwerve(s_Swerve, s_Pose, s_Pose.getPose().transformBy(transform), true);
    }

    private void buildAutos(SendableChooser<Command> chooser) {
        Command autoECD = LoggedCommands.sequence("ECD",
            Commands.either(
                LoggedCommands.deferredProxy("Back up push", this::BackUpCommand),
                LoggedCommands.log("Skip back up option"),
                optBackupPush::get),
            SetStop(Stop.L4),
            LoggedCommands.proxy(PathCommand("Start to near E")),
            LoggedCommands.proxy(ScoreCoralMaybeMirror(ReefFace.EF, true)),
            LoggedCommands.proxy(PathCommand("E to CS")),
            RobotState.WaitForCoral(),
            LoggedCommands.proxy(PathCommand("CS to near C")),
            LoggedCommands.proxy(ScoreCoralMaybeMirror(ReefFace.CD, true)),
            LoggedCommands.proxy(PathCommand("C to CS")),
            RobotState.WaitForCoral(),
            LoggedCommands.proxy(PathCommand("CS to near D")),
            LoggedCommands.proxy(ScoreCoralMaybeMirror(ReefFace.CD, false)),
            LoggedCommands.runOnce("Set Swerve Drive to Coast", () -> s_Swerve.setDriveMotorsToCoast()),
            LoggedCommands.proxy(PathCommand("D to CS")));

        startingPaths.put(autoECD, "Start to near E");
        addAutoCommand(chooser, autoECD);

        Command autoBA = LoggedCommands.sequence("BA",
            Commands.either(
                LoggedCommands.deferredProxy("Back up push", this::BackUpCommand),
                LoggedCommands.log("Skip back up option"),
                optBackupPush::get),
            SetStop(Stop.L4),
            LoggedCommands.proxy(PathCommand("Start to near B")),
            LoggedCommands.proxy(ScoreCoralMaybeMirror(ReefFace.AB, false)),
            LoggedCommands.proxy(PathCommand("B to CS2")),
            RobotState.WaitForCoral(),
            LoggedCommands.proxy(PathCommand("CS2 to near A")),
            LoggedCommands.proxy(ScoreCoralMaybeMirror(ReefFace.AB, true)),
            LoggedCommands.proxy(PathCommand("A backup")));

            startingPaths.put(autoBA, "Start to near B");
            addAutoCommand(chooser, autoBA);

        FollowPathCommand.warmupCommand().schedule();
    }

    public void teleopInit() {
        s_Swerve.stopSwerve();
        s_Swerve.setDriveMotorsToBrake();
    }

    public void teleopExit() {
    }

    public void disabledPeriodic() {
        Command autoCommand = getAutonomousCommand();
        String poseDifference = "N/A";
        boolean differenceOK = false;

        if (autoCommand != null) {
            String firstPath = startingPaths.get(autoCommand);

            if (firstPath != null) {
                Pose2d startingPose = startingPoses.get(firstPath + (shouldMirror() ? " - Mirror" : ""));

                if (startingPose != null) {
                    Pose2d currentPose = s_Pose.getPose();
                    DogLog.log("Robot/Debug0", PoseSubsystem.prettyPose(currentPose));
                    DogLog.log("Robot/Debug1", PoseSubsystem.prettyPose(startingPose));
                   
                    poseDifference = String.format("(%1.1f, %1.1f) @ %1.0f deg",
                        Units.metersToInches(currentPose.getX() - startingPose.getX()),
                        Units.metersToInches(currentPose.getY() - startingPose.getY()),
                        startingPose.getRotation().minus(currentPose.getRotation()).getDegrees());

                    if (Math.abs(currentPose.getX() - startingPose.getX()) < Constants.AutoConstants.maxSetupXError &&
                        Math.abs(currentPose.getY() - startingPose.getY()) < Constants.AutoConstants.maxSetupYError &&
                        Math.abs(startingPose.getRotation().minus(currentPose.getRotation()).getDegrees()) < Constants.AutoConstants.maxSetupDegError) {
                        differenceOK = true;
                    }
                }
            }
        }

        SmartDashboard.putString("autoSetup/Starting Pose Error", poseDifference);
        SmartDashboard.putBoolean("autoSetup/Starting Pose OK", differenceOK);
    }
}