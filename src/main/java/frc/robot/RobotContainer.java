package frc.robot;

import static frc.robot.Options.optServiceMode;
import static frc.robot.Options.optAutoCoralWait;
import static frc.robot.Options.optAutoReefAiming;
import static frc.robot.Options.optBackupPush;
import static frc.robot.Options.optInvertAlgae;
import static frc.robot.Options.optMirrorAuto;
import static frc.robot.Options.optL1Outside;

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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.Constants.Pose.Cage;
import frc.robot.Constants.Pose.ReefFace;
import frc.robot.Constants.Vision.CameraMode;
import frc.robot.Constants.Elevator.Stop;
import frc.robot.Constants.PIDSwerve.PIDSpeed;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.RobotState.ClimbState;

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
    private final ClimberSubsystem s_Climber;

    /* Autonomous Control */
    private final SendableChooser<Command> autoChooser;

    EnumMap<ReefFace, Command> coralLeftCommands = new EnumMap<>(ReefFace.class);
    EnumMap<ReefFace, Command> coralRightCommands = new EnumMap<>(ReefFace.class);
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
        s_Climber = new ClimberSubsystem();

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
        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
            (stream) -> stream.filter(auto -> !auto.getName().startsWith("Dummy")));
        SmartDashboard.putData("auto/Auto Chooser", autoChooser);
        buildAutos(autoChooser);
    }

    private Command ScoreCoral(ReefFace face, boolean left) {
        return Commands.either(
            LoggedCommands.sequence("Auto Align " + (left ? "Left " : "Right ") + face.toString() + " & Score",
                LoggedCommands.parallel("PID Align " + (left ? "Left " : "Right ") + face.toString(),
                    Commands.sequence(
                        VisionSubsystem.SwitchToFrontVision(),
                        Commands.race(
                            Commands.sequence(
                                Commands.either(
                                    Commands.sequence(
                                        // Commands.either(
                                            // LoggedCommands.log("Skipping approach pose for L2/L3 due to proximity to reef"),
                                            new PIDSwerve(s_Swerve, s_Pose, left ? face.approachLeft : face.approachRight, true, false, PIDSpeed.TURBO),
                                            // new PIDSwerve(s_Swerve, s_Pose, left ? face.approachLeft : face.approachRight, true, false, PIDSpeed.FAST),
                                            // () -> s_Pose.getPose().getTranslation().getDistance(Constants.Pose.reefCenter) <= Constants.Pose.approachDistanceToReefCenter),
                                        // new PIDSwerve(s_Swerve, s_Pose, left ? face.alignBonusLeft : face.alignBonusRight, true, true)
                                        new PIDSwerve(s_Swerve, s_Pose, left ? face.alignLeft : face.alignRight, true, true)
                                    ),
                                    Commands.sequence(
                                        new PIDSwerve(s_Swerve, s_Pose, left ? face.approachLeft : face.approachRight, true, false, PIDSpeed.FAST), //, Constants.maxVisionDiffCoral),
                                        Commands.either(
                                            LoggedCommands.log("Elevator reached stop in time"),
                                            LoggedCommands.sequence("Pause to wait for elevator to catch up",
                                                s_Swerve.Stop(),
                                                s_Elevator.WaitForNearNext()),
                                            s_Elevator::nearNextStop),
                                        Commands.either(
                                            Commands.either(
                                                new PIDSwerve(s_Swerve, s_Pose, left ? face.leftL1Outside : face.rightL1Outside, true, true),
                                                new PIDSwerve(s_Swerve, s_Pose, left ? face.leftL1 : face.rightL1, true, true),
                                                optL1Outside::get),
                                            new PIDSwerve(s_Swerve, s_Pose, left ? face.alignLeft : face.alignRight, true, true),
                                            () -> RobotState.getNextStop() == Stop.L1
                                        )),
                                    () -> false)), //RobotState.getNextStop() == Stop.L2 || RobotState.getNextStop() == Stop.L3)),
                            Commands.either(
                                Commands.sequence(
                                    LoggedCommands.waitSeconds("Score coral watchdog", Constants.Auto.scoreCoralTimeout),
                                    Commands.runOnce(() -> LoggedAlert.Error("Auto", "Timed out", "Timed out moving to score coral"))
                                ),
                                Commands.idle(),
                                () -> DriverStation.isAutonomousEnabled() && DriverStation.getMatchTime() >= (Constants.Auto.scoreCoralTimeout + Constants.Auto.scoreCoralTimeLeft))),
                        s_Swerve.Stop()),
                    Commands.sequence(
                        Commands.either(
                            LoggedCommands.log("Skip waiting for coral ready in turbo mode"),
                            RobotState.WaitForCoralReady(),
                            RobotState::getTurboMode),
                        LoggedCommands.deadline("Wait for auto up",
                            s_Elevator.WaitForNext(),
                            Commands.either(
                                Commands.either(
                                    s_Elevator.AutoElevatorUp(left ? face.leftL1Outside.getTranslation() : face.rightL1Outside.getTranslation()),
                                    s_Elevator.AutoElevatorUp(left ? face.leftL1.getTranslation() : face.rightL1.getTranslation()),
                                    optL1Outside::get),
                                s_Elevator.AutoElevatorUp(left ? face.alignLeft.getTranslation() : face.alignRight.getTranslation()),
                                // s_Elevator.SmoothElevatorUp(left ? face.approachLeft.getTranslation() : face.approachRight.getTranslation()), // TODO Not always Smooth!
                                () -> RobotState.getNextStop() == Stop.L1)))),
                Commands.parallel(
                    RobotState.ScoreGamePiece(),
                    Commands.either(
                        Commands.sequence(
                            Commands.waitSeconds(Constants.Elevator.L1RaiseDelay),
                            Commands.parallel(
                                s_Elevator.TimeBasedMove(Stop.L1_SCORE, 0.25),
                                Commands.sequence(
                                    Commands.waitSeconds(0.30),
                                    Commands.either(
                                        new PIDSwerve(s_Swerve, s_Pose, (left ? face.leftL1Outside : face.rightL1Outside).transformBy(new Transform2d(Constants.Pose.L1MoveForward, left ? Units.inchesToMeters(0) : Units.inchesToMeters(0), Rotation2d.kZero)), true, true, PIDSpeed.FAST),
                                        new PIDSwerve(s_Swerve, s_Pose, (left ? face.leftL1 : face.rightL1).transformBy(new Transform2d(Constants.Pose.L1MoveForward, 0, Rotation2d.kZero)), true, true, PIDSpeed.FAST),
                                        optL1Outside)))
                        ),
                        Commands.none(),
                        () -> RobotState.getNextStop() == Stop.L1
                    )
                )),
            LoggedCommands.log("Cannot score coral without coral"),
            () -> RobotState.haveCoral() || RobotState.getTurboMode())
        .handleInterrupt(() -> VisionSubsystem.setCameraMode(CameraMode.DEFAULT));
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

    private Command DealgaefyMaybeMirror(ReefFace face, boolean extendedBackup) {
        ReefFace mirroredFace = mirroredFaces.get(face);

        return Commands.either(
            DeAlgaefy(mirroredFace, extendedBackup),
            DeAlgaefy(face, extendedBackup),
            this::shouldMirror);
    }

    private Command Rumble() {
        return Commands.deadline(
            Commands.waitSeconds(0.5),
            LoggedCommands.startEnd("Rumble",
                () -> {
                    driver.setRumble(RumbleType.kLeftRumble, 1.0);
                    driver.setRumble(RumbleType.kRightRumble, 1.0);
                },
                () -> {
                    driver.setRumble(RumbleType.kLeftRumble, 0.0);
                    driver.setRumble(RumbleType.kRightRumble, 0.0);
                }));
    }

    private Command TriggerRumble() {
        Command rumbleCommmand = Rumble();

        return Commands.runOnce(() -> rumbleCommmand.schedule());
    }

    private Command DeAlgaefy(ReefFace face) {
        return DeAlgaefy(face, true);
    }

    private Command DeAlgaefy(ReefFace face, boolean extendedBackup) {
        Stop algaeStop = face.algaeHigh ? Stop.L3_ALGAE: Stop.L2_ALGAE;
        Stop algaeInvertStop = face.algaeHigh ? Stop.L2_ALGAE : Stop.L3_ALGAE;

        return LoggedCommands.sequence("Fully acquire Algae from " + face.toString(),
            LoggedCommands.deadline("Acquire Algae from " + face.toString(),
                Commands.sequence(
                    LoggedCommands.waitUntil("Wait for Algae", RobotState::haveAlgae),
                    TriggerRumble()),
                LoggedCommands.sequence("Auto Align Middle " + face.toString(),
                    VisionSubsystem.SwitchToFrontVision(),
                    RobotState.IntakeAlgae(),
                    LoggedCommands.parallel("PID Align Middle " + face.toString(),
                        Commands.sequence(
                            new PIDSwerve(s_Swerve, s_Pose, face.approachMiddle, true, false),
                            new PIDSwerve(s_Swerve, s_Pose, face.alignMiddle, true, true),
                            s_Swerve.Stop()),
                        Commands.either(
                            LoggedCommands.deadline("Wait for auto up to " + algaeInvertStop,
                                s_Elevator.WaitForStop(algaeInvertStop),
                                s_Elevator.AutoElevatorUp(face.alignMiddle.getTranslation(), algaeInvertStop)),
                            LoggedCommands.deadline("Wait for auto up to " + algaeStop,
                                s_Elevator.WaitForStop(algaeStop),
                                s_Elevator.AutoElevatorUp(face.alignMiddle.getTranslation(), algaeStop)),
                            optInvertAlgae)))),
            new PIDSwerve(s_Swerve, s_Pose, extendedBackup ? face.algaeBackupExtended : face.algaeBackupShort, true, false))
            .handleInterrupt(() -> {
                if (!RobotState.haveAlgae()) RobotState.setNoAlgae();
                VisionSubsystem.setCameraMode(CameraMode.DEFAULT);
            });
    }

    private Command DeLollipop() {
        return LoggedCommands.deadline("Acquire Algae from lollipop",
                Commands.sequence(
                    LoggedCommands.waitUntil("Wait for Algae", RobotState::haveAlgae),
                    TriggerRumble()),
                RobotState.IntakeAlgae())
            .handleInterrupt(() -> { if (!RobotState.haveAlgae()) RobotState.setNoAlgae(); });
    }

    private void setFaceCommands(ReefFace face) {
        coralLeftCommands.put(face, ScoreCoral(face, true));
        coralRightCommands.put(face, ScoreCoral(face, false));
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

        return LoggedCommands.either("Choosing auto path for " + pathName,
            LoggedCommands.logWithName("Mirrored Path: " + pathName, mirrorCommand),
            LoggedCommands.logWithName("Path: " + pathName, pathCommand),
             this::shouldMirror);
    }

    private Command BargeShot() {
        return BargeShot(0.0);
    }

    private Command BargeShot(double adjustment) {
        return LoggedCommands.sequence("Score Algae into Barge",
                Commands.defer(() -> new PIDSwerve(s_Swerve, s_Pose, s_Pose.bargeShotPose(adjustment), false, false, PIDSpeed.TURBO).ignoreY(), Set.of(s_Swerve)),
                LoggedCommands.deadline("Toss Algae",
                    Commands.sequence(
                        s_Elevator.WaitForStop(Stop.L4_SCORE)),
                    s_Swerve.HoldX(),
                    s_Elevator.Move(Stop.L4_SCORE),
                    LoggedCommands.sequence("Wait to release Algae",
                        LoggedCommands.waitUntil("Wait for Algae Release Point", () -> s_Elevator.aboveStop(Stop.ALGAE_RELEASE)),
                        RobotState.ScoreGamePiece())),
                s_Elevator.FastZero()); // TODO Defer so that drive control returns?
    }

    private Command AlignToCage(Cage cage) {
        Pose2d cageAlignPose = new Pose2d(cage.location(), Rotation2d.k180deg).transformBy(Constants.Pose.cageOffset);

        return LoggedCommands.sequence("Align to cage " + cage,
            LoggedCommands.runOnce("Disable reef aiming", optAutoReefAiming::disable),
            VisionSubsystem.SwitchToRearVision(),
            LoggedCommands.runOnce("Enable end game mode", () -> RobotState.setClimbState(ClimbState.STARTED)),
            new PIDSwerve(s_Swerve, s_Pose, cageAlignPose.transformBy(Constants.Pose.cageApproachOffset), true, false, PIDSpeed.FAST),
            new PIDSwerve(s_Swerve, s_Pose, cageAlignPose, true, true, PIDSpeed.SLOW));
    }

    private Command ProcessorAlign() {
        return LoggedCommands.sequence("Align to processor",
            new PIDSwerve(s_Swerve, s_Pose, Constants.Pose.processorApproach, true, false, PIDSpeed.FAST),
            new PIDSwerve(s_Swerve, s_Pose, Constants.Pose.processorScore, true, true, PIDSpeed.FAST));
    }

    private Command AlignToNearestCage() {
        return Commands.defer(() -> AlignToCage(s_Pose.nearestCage()), Set.of(s_Swerve));
    }

    private Command SmartScore(boolean left) {
        return Commands.either(
            LoggedCommands.proxy(Commands.select(left ? coralLeftCommands : coralRightCommands, () -> PoseSubsystem.nearestFace(s_Pose.getPose().getTranslation()))),
            Commands.either(
                Commands.either(
                    LoggedCommands.proxy(ProcessorAlign()),
                    LoggedCommands.proxy(BargeShot()),
                    RobotState::algaeToProcessor),
                Commands.either(
                    LoggedCommands.proxy(Commands.select(left ? deAlgaefyLeftCommands : deAlgaefyRightCommands, () -> PoseSubsystem.nearestFace(s_Pose.getPose().getTranslation()))),
                    DeLollipop(),
                    optAutoReefAiming::get),
                RobotState::haveAlgae),
            RobotState::haveCoral);
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
        final Trigger unjam = driver.back();
        final Trigger alignmentToggle = driver.rightStick();

        // Only used in case of automation failure
        moveElevator.whileTrue(s_Elevator.GoToNext());
        zero.onTrue(s_Elevator.Zero());
        unjam.onTrue(RobotState.UnjamCoral());
        score.whileTrue(RobotState.ScoreGamePiece()); // Also useful to dump Algae or put it into Processor

        L4.onTrue(SetStop(Stop.L4));
        L3.onTrue(SetStop(Stop.L3));
        L2.onTrue(SetStop(Stop.L2));
        L1.onTrue(SetStop(Stop.L1));

        goLeft.whileTrue(SmartScore(true));
        goRight.whileTrue(SmartScore(false));

        alignmentToggle.onTrue(LoggedCommands.runOnce("Toggle Alignment", optAutoReefAiming::toggle));

        if (Constants.atHQ) {
            // driver.povUp().whileTrue(
            //     Commands.sequence(
            //         new PIDSwerve(s_Swerve, s_Pose, new Pose2d(4.48, 1.67, Rotation2d.fromDegrees(91)), false, false),
            //         s_Swerve.Stop(),
            //         Commands.runOnce(() -> LoggedAlert.Info("Debug", "In Position", "Reached Debug Position")),
            //         Commands.runOnce(() -> LEDSubsystem.triggerError())));
            // driver.povRight().onTrue(
            //     LoggedCommands.sequence("Test Drive -- 5 meters",
            //         Commands.runOnce(() -> s_Pose.setPose(new Pose2d(2.0, 7.0, Rotation2d.kZero))),
            //         PathCommand("Test Drive - 5m"),
            //         s_Swerve.Stop(),
            //         Commands.runOnce(() -> LoggedAlert.Info("Debug", "In Position", "Reached End of Path")),
            //         Commands.runOnce(() -> LEDSubsystem.triggerError())));
            // driver.povLeft().onTrue(
            //     LoggedCommands.sequence("Test Drive -- 2.5 meters",
            //         Commands.runOnce(() -> s_Pose.setPose(new Pose2d(2.0, 7.0, Rotation2d.kZero))),
            //         LoggedCommands.logWithName("2.5 m path", PathCommand("Test Drive - 2.5m")),
            //         LoggedCommands.logWithName("Stop", s_Swerve.Stop()),
            //         Commands.runOnce(() -> LoggedAlert.Info("Debug", "In Position", "Reached End of Path")),
            //         LoggedCommands.runOnce("Test End", () -> LEDSubsystem.triggerError())));
        }

        driver.povDown().onTrue(
            Commands.either(
                s_Climber.SlowDeploy().until(driver.povDown().negate()),
                s_Climber.Deploy(),
                optServiceMode::get));
        driver.povUp().whileTrue(
            Commands.either(
                s_Climber.SlowRetract(),
                s_Climber.Retract(),
                optServiceMode::get));
        driver.povRight().whileTrue(
            LoggedCommands.parallel("Deploy and Align",
                s_Climber.Deploy(),
                AlignToNearestCage()));
        driver.povLeft().onTrue(LoggedCommands.runOnce("Toggle L1 Inside/Outside", optL1Outside::toggle));
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
        Transform2d transform = new Transform2d(-Constants.Auto.backUpPushDistance, 0.0, Rotation2d.kZero); 
        return
            LoggedCommands.race("Backup with timeout",
                LoggedCommands.waitSeconds("Backup timeout", 3), // TODO Make constant
                new PIDSwerve(s_Swerve, s_Pose, s_Pose.getPose().transformBy(transform), false, true));
    }

    @SuppressWarnings ("unused")
    private Command BackUpAndWaitForCoral() {
        Transform2d transform = new Transform2d(-Constants.Auto.backUpCSDistance, 0.0, Rotation2d.kZero); 
        return LoggedCommands.deadline("Backup and wait for Coral",
            RobotState.WaitForCoral(),
            Commands.sequence(
                Commands.defer(() -> new PIDSwerve(s_Swerve, s_Pose, s_Pose.getPose().transformBy(transform), false, false), Set.of(s_Swerve)),
                Commands.defer(() -> new PIDSwerve(s_Swerve, s_Pose, s_Pose.getPose().transformBy(transform), false, false), Set.of(s_Swerve))
            ));
    }

    @SuppressWarnings ("unused")
    private Command PathWithRaise(String pathName, ReefFace face, boolean left) {
        return LoggedCommands.deadline("Follow Path with Raise",
            PathCommand(pathName),
            s_Elevator.AutoElevatorUp(left ? face.alignLeft.getTranslation() : face.alignRight.getTranslation())
        ); 
    }

    private Command WaitForReefDistance(double distance) {
        return LoggedCommands.waitUntil("Wait until within " + String.format("%1.2f", distance) + "m of reef center",
            () -> PoseSubsystem.reefDistance(s_Pose.getPose().getTranslation()) <= distance);
    }

    private Command RaiseElevatorAtDistance(double distance) {
        return LoggedCommands.sequence("Raise elevator within " + String.format("%1.2f", distance) + "m of reef center",
            WaitForReefDistance(distance),
            Commands.either(
                RobotState.WaitForCoralReady(),
                LoggedCommands.log("Missing coral"),
                RobotState::haveCoral),
            LoggedCommands.proxy(s_Elevator.GoToNext()));
    }

    private Command WaitForTowardsNext() {
        return LoggedCommands.either("Ensure towards next stop",
            Commands.none(),
            Commands.sequence(
                LoggedCommands.proxy(s_Swerve.Stop()),
                s_Elevator.WaitForTowardsNext()),
            s_Elevator::towardsNextStop);
    }

    private Command FastScoreCoral(String path, ReefFace face, boolean left, double raiseDistance, double timeCutoff) {
        return LoggedCommands.deadline("Fast coral score following " + path,
            Commands.sequence(
                LoggedCommands.proxy(PathCommand(path)),
                Commands.race(
                    Commands.sequence(
                        Commands.waitUntil(() -> Timer.getMatchTime() <= timeCutoff),
                        Commands.either(
                            Commands.none(),
                            s_Elevator.WaitForNext(),
                            s_Elevator::atNextStop),
                        RobotState.ScoreGamePiece(),
                        LoggedCommands.proxy(s_Swerve.Stop())
                    ),
                    Commands.sequence(
                        WaitForTowardsNext(),
                        Commands.either(
                            LoggedCommands.proxy(new PIDSwerve(s_Swerve, s_Pose, left ? mirroredFaces.get(face).alignRight : mirroredFaces.get(face).alignLeft, true, true).fastAlign()),
                            LoggedCommands.proxy(new PIDSwerve(s_Swerve, s_Pose, left ? face.alignLeft : face.alignRight, true, true).fastAlign()),
                            this::shouldMirror
                        ),
                        LoggedCommands.proxy(s_Swerve.Stop()),
                        Commands.either(
                            Commands.none(),
                            s_Elevator.WaitForNext(),
                            s_Elevator::atNextStop),
                        RobotState.ScoreGamePiece()))),
            LoggedCommands.proxy(RaiseElevatorAtDistance(raiseDistance)));
    }

    private Command MaybeWaitForCoral() {
        return Commands.either(
            LoggedCommands.proxy(RobotState.WaitForCoral()),
            LoggedCommands.log("Will not wait for Coral"),
            optAutoCoralWait::get);
    }

    private Command GoGetCoral(String path) {
        return LoggedCommands.sequence("Go Get Coral following " + path,
            VisionSubsystem.SwitchToRearVision(),
            Commands.race(
                LoggedCommands.proxy(PathCommand(path)),
                RobotState.WaitForCoral()),
            MaybeWaitForCoral(),
            VisionSubsystem.SwitchToFrontVision());
    }

    private void buildAutos(SendableChooser<Command> chooser) {
        Command autoECDB = LoggedCommands.sequence("Regular Three Piece (ECD+B)",
            VisionSubsystem.SwitchToFrontVision(),
            LoggedCommands.defer("Startup delay", () -> Commands.waitSeconds(SmartDashboard.getNumber("auto/Startup delay", 0.0)), Set.of()),
            Commands.either(
                LoggedCommands.deferredProxy("Back up push", this::BackUpCommand),
                LoggedCommands.log("Skip back up option"),
                optBackupPush::get),
            SetStop(Stop.L4),
            LoggedCommands.proxy(PathCommand("Start towards EF")),
            LoggedCommands.proxy(ScoreCoralMaybeMirror(ReefFace.EF, true)),
            GoGetCoral("E to CS"),
            LoggedCommands.proxy(PathCommand("CS towards C")),
            LoggedCommands.proxy(ScoreCoralMaybeMirror(ReefFace.CD, true)),
            GoGetCoral("C to CS"),
            LoggedCommands.proxy(PathCommand("CS towards D")),
            LoggedCommands.proxy(ScoreCoralMaybeMirror(ReefFace.CD, false)),
            s_Swerve.CoastDriveMotors(),
            GoGetCoral("D to CS"),
            LoggedCommands.proxy(PathCommand("CS to near B")),
            LoggedCommands.proxy(ScoreCoralMaybeMirror(ReefFace.AB, false)),
            LoggedCommands.proxy(new PIDSwerve(s_Swerve, s_Pose, ReefFace.AB.approachRight, true, false)))
        .handleInterrupt(() -> VisionSubsystem.setCameraMode(CameraMode.DEFAULT));

        startingPaths.put(autoECDB, "Start towards EF");
        addAutoCommand(chooser, autoECDB);

        Command fastFour = LoggedCommands.sequence("Fast Four Piece (ECDB)",
            LoggedCommands.runOnce("Disable waiting for coral for fast four piece auto", optAutoCoralWait::disable),
            SetStop(Stop.L4),
            VisionSubsystem.SwitchToFrontVision(),
            LoggedCommands.proxy(FastScoreCoral("Fast - Start to E", ReefFace.EF, true, 2.52, 0.0)),
            GoGetCoral("Fast - E to CS"),
            LoggedCommands.proxy(FastScoreCoral("Fast - CS to C", ReefFace.CD, true, 3.46, 0.0)),
            GoGetCoral("Fast - C to CS"),
            LoggedCommands.proxy(FastScoreCoral("Fast - CS to D", ReefFace.CD, false, 3.56, 0.0)),
            GoGetCoral("Fast - D to CS"),
            LoggedCommands.proxy(FastScoreCoral("Fast - CS to B", ReefFace.AB, false, 2.91, 0.25)),
            LoggedCommands.proxy(new PIDSwerve(s_Swerve, s_Pose, ReefFace.AB.approachMiddle, true, false)))
        .handleInterrupt(() -> VisionSubsystem.setCameraMode(CameraMode.DEFAULT));

        startingPaths.put(fastFour, "Fast - Start to E");
        addAutoCommand(chooser, fastFour);

        Command autoBA = LoggedCommands.sequence("BA (Outside)",
            LoggedCommands.defer("Startup delay", () -> Commands.waitSeconds(SmartDashboard.getNumber("auto/Startup delay", 0.0)), Set.of()),
            Commands.either(
                LoggedCommands.deferredProxy("Back up push", this::BackUpCommand),
                LoggedCommands.log("Skip back up option"),
                optBackupPush::get),
            SetStop(Stop.L4),
            VisionSubsystem.SwitchToFrontVision(),
            LoggedCommands.proxy(PathCommand("Start to near B")),
            LoggedCommands.proxy(ScoreCoralMaybeMirror(ReefFace.AB, false)),
            GoGetCoral("B to CS2"),
            Commands.either(
                // At HQ, we need to score on L2 B instead of L4 A, due to space constraints
                Commands.sequence(
                    SetStop(Stop.L2),
                    LoggedCommands.proxy(PathCommand("CS2 to near B")),
                    LoggedCommands.proxy(ScoreCoralMaybeMirror(ReefFace.AB, false))
                ),
                Commands.sequence(
                    LoggedCommands.proxy(PathCommand("CS2 to near A")),
                    LoggedCommands.proxy(ScoreCoralMaybeMirror(ReefFace.AB, true))
                ),
                () -> Constants.atHQ),
            LoggedCommands.proxy(new PIDSwerve(s_Swerve, s_Pose, ReefFace.AB.approachMiddle, true, false)),
            LoggedCommands.proxy(s_Swerve.Stop()))
        .handleInterrupt(() -> VisionSubsystem.setCameraMode(CameraMode.DEFAULT));

        startingPaths.put(autoBA, "Start to near B");
        addAutoCommand(chooser, autoBA);

        Command autoGBA = LoggedCommands.sequence("GBA (Inside)",
            LoggedCommands.defer("Startup delay", () -> Commands.waitSeconds(SmartDashboard.getNumber("auto/Startup delay", 0.0)), Set.of()),
            Commands.either(
                LoggedCommands.deferredProxy("Back up push", this::BackUpCommand),
                LoggedCommands.log("Skip back up option"),
                optBackupPush::get),
            SetStop(Stop.L4),
            VisionSubsystem.SwitchToFrontVision(),
            LoggedCommands.proxy(PathCommand("Start to near G")),
            LoggedCommands.proxy(ScoreCoralMaybeMirror(ReefFace.GH, true)),
            GoGetCoral("G to CS2"),
            LoggedCommands.proxy(PathCommand("CS2 to near B")),
            LoggedCommands.proxy(ScoreCoralMaybeMirror(ReefFace.AB, false)),
            GoGetCoral("B to CS2"),
            Commands.either(
                // At HQ, we need to score on L2 B instead of L4 A, due to space constraints
                Commands.sequence(
                    SetStop(Stop.L2),
                    LoggedCommands.proxy(PathCommand("CS2 to near B")),
                    LoggedCommands.proxy(ScoreCoralMaybeMirror(ReefFace.AB, false))
                ),
                Commands.sequence(
                    LoggedCommands.proxy(PathCommand("CS2 to near A")),
                    LoggedCommands.proxy(ScoreCoralMaybeMirror(ReefFace.AB, true))
                ),
                () -> Constants.atHQ),
            LoggedCommands.proxy(new PIDSwerve(s_Swerve, s_Pose, ReefFace.AB.approachMiddle, true, false)),
            LoggedCommands.proxy(s_Swerve.Stop()))
        .handleInterrupt(() -> VisionSubsystem.setCameraMode(CameraMode.DEFAULT));

        startingPaths.put(autoGBA, "Start to near G");
        addAutoCommand(chooser, autoGBA);

        // NOTE: Do not mirror this auto!
        Command autoG = LoggedCommands.sequence("G + Barge Shots (don't mirror!)",
            LoggedCommands.defer("Startup delay", () -> Commands.waitSeconds(SmartDashboard.getNumber("auto/Startup delay", 0.0)), Set.of()),
            Commands.either(
                LoggedCommands.deferredProxy("Back up push", this::BackUpCommand),
                LoggedCommands.log("Skip back up option"),
                optBackupPush::get),
            SetStop(Stop.L4),
            LoggedCommands.proxy(PathCommand("Start to near G")),
            LoggedCommands.proxy(ScoreCoralMaybeMirror(ReefFace.GH, true)),
            LoggedCommands.proxy(new PIDSwerve(s_Swerve, s_Pose, ReefFace.GH.approachMiddle, true, false)),
            LoggedCommands.proxy(DealgaefyMaybeMirror(ReefFace.GH, false)),
            LoggedCommands.proxy(PathCommand("GH to Barge Shot")),
            LoggedCommands.proxy(BargeShot()),
            LoggedCommands.proxy(PathCommand("Barge Shot to near IJ")),
            LoggedCommands.proxy(DealgaefyMaybeMirror(ReefFace.IJ, false)), 
            LoggedCommands.proxy(BargeShot(-Units.inchesToMeters(5))), // Ensure we are shy of the line at the end of auto
            LoggedCommands.deferredProxy("Backup after barge shot", 
                () -> new PIDSwerve(s_Swerve, s_Pose, s_Pose.getPose().transformBy(new Transform2d(-Units.inchesToMeters(18.0), 0.0, Rotation2d.kZero)), false, false)),
            LoggedCommands.proxy(s_Swerve.Stop()));

        startingPaths.put(autoG, "Start to near G");
        addAutoCommand(chooser, autoG);

        FollowPathCommand.warmupCommand().schedule();
    }

    public void autonomousInit() {
        // Ensure the Swerve subsystem doesn't run a default command, in case we previously were in teleop mode
        Command oldDefault = s_Swerve.getDefaultCommand();

        s_Swerve.removeDefaultCommand();
        if (oldDefault != null && oldDefault.isScheduled()) {
            oldDefault.cancel();
        }
    }

    public void teleopInit() {
        s_Swerve.stopSwerve();
        CommandScheduler.getInstance().schedule(s_Swerve.BrakeDriveMotors());
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> -translation.get() * Constants.driveStickSensitivity,
                () -> -strafe.get() * Constants.driveStickSensitivity,
                () -> -rotation.get() * Constants.turnStickSensitivity,
                this::speedLimitFactor
            ));
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
                   
                    poseDifference = String.format("(%1.1f, %1.1f) @ %1.0f deg",
                        Units.metersToInches(currentPose.getX() - startingPose.getX()),
                        Units.metersToInches(currentPose.getY() - startingPose.getY()),
                        startingPose.getRotation().minus(currentPose.getRotation()).getDegrees());

                    if (Math.abs(currentPose.getX() - startingPose.getX()) < Constants.Auto.maxSetupXError &&
                        Math.abs(currentPose.getY() - startingPose.getY()) < Constants.Auto.maxSetupYError &&
                        Math.abs(startingPose.getRotation().minus(currentPose.getRotation()).getDegrees()) < Constants.Auto.maxSetupDegError) {
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