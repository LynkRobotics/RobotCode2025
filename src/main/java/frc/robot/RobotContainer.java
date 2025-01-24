package frc.robot;

import java.util.Set;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import static frc.robot.Options.*;
import frc.robot.subsystems.ClimberSubsystem.ClimberSelection;
import frc.robot.subsystems.ShooterSubsystem.Speed;

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
    //private final Joystick driver = new Joystick(0);
    private final CommandXboxController driver = new CommandXboxController(0);

    /* Drive Controls */
    private final Supplier<Double> translation = driver::getLeftY;
    private final Supplier<Double> strafe = driver::getLeftX;
    private final Supplier<Double> rotation = driver::getRightX;

    /* Driver Buttons */
    private final Trigger intakeButton = driver.leftBumper();
    private final Trigger shooterButton = driver.rightBumper();
    private final Trigger ejectButton = driver.start();
    private final Trigger leftClimberButton = driver.leftTrigger();
    private final Trigger rightClimberButton = driver.rightTrigger();
    
    /* Different Position Test Buttons */
    private final Trigger ampButton = driver.a();
    private final Trigger dumpShotButton = driver.b();
    private final Trigger defaultShotButton = driver.back();
    private final Trigger slideShotButton = driver.x();
    private final Trigger climberExtendButton = driver.y();
    private final Trigger ampShotButton = driver.povDown();
    private final Trigger sourceAlignButton = driver.povUp();

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final IntakeSubsystem s_Intake = new IntakeSubsystem();
    private final ShooterSubsystem s_Shooter = new ShooterSubsystem();
    private final IndexSubsystem s_Index = new IndexSubsystem();
    private final ClimberSubsystem s_LeftClimber = new ClimberSubsystem(ClimberSelection.LEFT);
    private final ClimberSubsystem s_RightClimber = new ClimberSubsystem(ClimberSelection.RIGHT);
    @SuppressWarnings ("unused")
    private final LEDSubsystem s_Led = new LEDSubsystem();
    private final VisionSubsystem s_Vision = new VisionSubsystem();
    @SuppressWarnings ("unused")
    private final PoseSubsystem s_Pose = new PoseSubsystem(s_Swerve, s_Vision);

    private final SendableChooser<Command> autoChooser;

    private static void autoNamedCommand(String name, Command command) {
        NamedCommands.registerCommand(name, LoggedCommands.logWithName(name + " (auto)", command));
    }

    private Command setShotCommand(Speed speed) {
        return LoggedCommands.runOnce("Set " + (speed == null ? "default" : speed) + " shot", () -> { s_Shooter.setNextShot(speed); });
    }

    private Command fixedShotCommand(Speed speed) {
        return 
            LoggedCommands.sequence(
                "Fixed " + speed + " shot",
                setShotCommand(speed),
                LoggedCommands.race(
                    "Shoot with timeout",
                    new ShootCommand(s_Shooter, s_Index, false),
                    LoggedCommands.waitSeconds("Shoot timer", 1.5)));
    }

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        s_Shooter,
                        s_Vision,
                        () -> -translation.get() * Constants.driveStickSensitivity,
                        () -> -strafe.get() * Constants.driveStickSensitivity,
                        () -> -rotation.get() * Constants.turnStickSensitivity,
                        s_Swerve::getSpeedLimitRot
                        ));

        s_Shooter.setDefaultCommand(LoggedCommands.startEnd("Shooter Idle", s_Shooter::idle, () -> {}, s_Shooter));
        s_Index.setDefaultCommand(LoggedCommands.startEnd("Index Stop", s_Index::stop, () -> {}, s_Index));

        SmartDashboard.putData("Command scheduler", CommandScheduler.getInstance());
        SmartDashboard.putData(new ShootCommand(s_Shooter, s_Index, s_Swerve).withTimeout(3.0).withName("Shoot Commmand"));

        // Default named commands for PathPlanner
        SmartDashboard.putNumber("auto/Startup delay", 0.0);
        autoNamedCommand("Done", Commands.print("Done"));
        autoNamedCommand("Start", Commands.print("Starting"));
        autoNamedCommand("Startup delay", Commands.defer(() -> Commands.waitSeconds(SmartDashboard.getNumber("auto/Startup delay", 0.0)), Set.of()));
        autoNamedCommand("Shoot",
            Commands.race(
                new ShootCommand(s_Shooter, s_Index, s_Swerve),
                new AimCommand(s_Swerve, s_Vision),
                LoggedCommands.waitSeconds("Shoot timeout", 2.50)));
        autoNamedCommand("Shoot without aiming",
            Commands.race(
            new ShootCommand(s_Shooter, s_Index, s_Swerve, false),
            LoggedCommands.waitSeconds("Shoot w/o aiming timeout", 1.50)));
        autoNamedCommand("Fixed SW shot", fixedShotCommand(Speed.SUBWOOFER));
        autoNamedCommand("Fixed AS shot", fixedShotCommand(Speed.AMPSIDE));
        autoNamedCommand("Shoot OTF", fixedShotCommand(Speed.OTF));
        autoNamedCommand("Amp-side OTF Shot", fixedShotCommand(Speed.AMPSIDEOTF));
        autoNamedCommand("Source-side OTF Shot", fixedShotCommand(Speed.SOURCESIDEOTF));
        autoNamedCommand("Intake note", new IntakeCommand(s_Intake, s_Index, driver.getHID()));
        autoNamedCommand("Amp Shot", fixedShotCommand(Speed.AMP));
        autoNamedCommand("Bloop Shot", fixedShotCommand(Speed.BLOOP));
        autoNamedCommand("Slide Shot", fixedShotCommand(Speed.SLIDE));
        autoNamedCommand("Short Slide Shot", fixedShotCommand(Speed.SHORTSLIDE));
        autoNamedCommand("Special Shot", fixedShotCommand(Speed.SPECIAL));
        autoNamedCommand("Override rotation", Commands.runOnce(s_Vision::enableRotationTargetOverride));
        autoNamedCommand("Restore rotation", Commands.runOnce(s_Vision::disableRotationTargetOverride));
        autoNamedCommand("Stop", Commands.runOnce(s_Swerve::stopSwerve));
        autoNamedCommand("Set Instant Pose", Commands.runOnce(() ->
            {
                if (s_Vision.haveSpeakerTarget()) {
                    Pose2d pose = s_Vision.lastPose();
                    s_Pose.setPose(pose);
                    DogLog.log("Auto/Status", "Pose updated from vision: " + PoseSubsystem.prettyPose(pose));
                } else {
                    DogLog.log("Auto/Status", "Refusing to update pose from vision without a current speaker target");
                }
            } ));
        autoNamedCommand("Coast after auto", new CoastAfterAuto(s_Swerve));
        autoNamedCommand("Coast drive motors", Commands.runOnce(s_Swerve::setDriveMotorsToCoast));

        // Build an autoChooser (defaults to none)
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("auto/Auto Chooser", autoChooser);
        buildAutos(autoChooser);

        // During calibration allow for direct control
        // SmartDashboard.putNumber("Shooter voltage direct", 0.0);
        // SmartDashboard.putData(s_Shooter.runOnce(() -> { s_Shooter.setVoltage(SmartDashboard.getNumber("Shooter voltage direct", 0)); }).withName("Set shooter voltage"));
        // SmartDashboard.putData(s_Shooter.runOnce(() -> { s_Shooter.setVoltage(0); }).withName("Stop shooter"));

        SmartDashboard.putNumber("TeleOp Speed Governor", 1.0);

        // Allow for direct RPM setting
        SmartDashboard.putNumber("Shooter top RPM", 1000.0);
        SmartDashboard.putNumber("Shooter bottom RPM", 1000.0);
        SmartDashboard.putData(s_Shooter.runOnce(() -> { s_Shooter.setRPM(500); }).withName("Idle shooter"));
        SmartDashboard.putData(LoggedCommands.runOnce("Zero Gyro", s_Pose::zeroGyro, s_Swerve));
        SmartDashboard.putData(LoggedCommands.runOnce("Reset heading", s_Pose::resetHeading, s_Swerve));

        // Allow for direct climber control
        SmartDashboard.putData(LoggedCommands.runOnce("Stop climbers", () -> { s_LeftClimber.stop(); s_RightClimber.stop(); }, s_LeftClimber, s_RightClimber));
        SmartDashboard.putData(s_LeftClimber.runOnce(() -> { s_LeftClimber.applyVoltage(Constants.Climber.slowVoltage); }).withName("Left down slow"));
        SmartDashboard.putData(s_RightClimber.runOnce(() -> { s_RightClimber.applyVoltage(Constants.Climber.slowVoltage); }).withName("Right down slow"));

        SmartDashboard.putNumber("Left climber voltage", 0.0);
        SmartDashboard.putNumber("Right climber voltage", 0.0);
        SmartDashboard.putData(LoggedCommands.runOnce("Set climber voltage", () -> { s_LeftClimber.applyVoltage(SmartDashboard.getNumber("Left climber voltage", 0.0)); s_RightClimber.applyVoltage(SmartDashboard.getNumber("Right climber voltage", 0.0));}, s_LeftClimber, s_RightClimber));
        SmartDashboard.putData(LoggedCommands.runOnce("Zero climbers", () -> { s_LeftClimber.zero(); s_RightClimber.zero(); }, s_LeftClimber, s_RightClimber));

        SmartDashboard.putNumber("Left climber target position", 0.0);
        SmartDashboard.putData(new ClimberPositionCommand(SmartDashboard.getNumber("Left climber target position", 0.0), LEDSubsystem.TempState.RETRACTING, s_LeftClimber).withName("Set left climber position"));
        SmartDashboard.putNumber("Right climber target position", 0.0);
        SmartDashboard.putData(new ClimberPositionCommand(SmartDashboard.getNumber("Right climber target position", 0.0), LEDSubsystem.TempState.RETRACTING, s_RightClimber).withName("Set right climber position"));

        SmartDashboard.putData(LoggedCommands.runOnce("autoSetup/SetSwerveCoast", s_Swerve::setMotorsToCoast, s_Swerve).ignoringDisable(true));
        SmartDashboard.putData(LoggedCommands.runOnce("autoSetup/SetSwerveBrake", s_Swerve::setMotorsToBrake, s_Swerve).ignoringDisable(true));
        SmartDashboard.putData(LoggedCommands.run("autoSetup/SetSwerveAligned", s_Swerve::alignStraight, s_Swerve).ignoringDisable(true));

        DogLog.setOptions(
            new DogLogOptions()
            .withCaptureConsole(true)
            .withCaptureDs(true)
            .withCaptureNt(true)
            .withLogEntryQueueCapacity(1000)
            .withLogExtras(true)
            .withNtPublish(true));

        // Configure the button bindings
        configureButtonBindings();
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
        intakeButton.whileTrue(
            LoggedCommands.sequence(
                "Intake",
                LoggedCommands.runOnce("Enable speed limit", s_Swerve::enableSpeedLimit),
                Commands.either(
                    new ShooterIntakeCommand(s_Shooter, s_Index, driver.getHID()),
                    new IntakeCommand(s_Intake, s_Index, driver.getHID()),
                    optShooterIntake),
                LoggedCommands.runOnce("Disable speed limit", s_Swerve::disableSpeedLimit))
            .handleInterrupt(s_Swerve::disableSpeedLimit));
        shooterButton.whileTrue(
            LoggedCommands.either(
                "Shoot",
                new ShootCommand(s_Shooter, s_Index,
                    () -> SmartDashboard.getNumber("Shooter top RPM", 0.0),
                    () -> SmartDashboard.getNumber("Shooter bottom RPM", 0.0)),
                new ShootCommand(s_Shooter, s_Index, s_Swerve),
                optDirectRPM));
        climberExtendButton.onTrue(
            LoggedCommands.sequence(
                "Extend climbers",
                LoggedCommands.runOnce("Enable speed limit", s_Swerve::enableSpeedLimit),
                Commands.parallel(
                    LoggedCommands.logWithName("Extend left climber", new ClimberPositionCommand(Constants.Climber.extendedPosition, LEDSubsystem.TempState.EXTENDING, s_LeftClimber)),
                    LoggedCommands.logWithName("Extend right climber", new ClimberPositionCommand(Constants.Climber.extendedPosition, LEDSubsystem.TempState.EXTENDING, s_RightClimber)))));
        SmartDashboard.putData("Disable speed limit", Commands.runOnce(s_Swerve::disableSpeedLimit));

        leftClimberButton.whileTrue(new ClimberPositionCommand(Constants.Climber.retractedPosition, LEDSubsystem.TempState.RETRACTING, s_LeftClimber));
        rightClimberButton.whileTrue(new ClimberPositionCommand(Constants.Climber.retractedPosition, LEDSubsystem.TempState.RETRACTING, s_RightClimber));

        ejectButton.whileTrue(new EjectCommand(s_Intake, s_Index, s_Shooter));

        SmartDashboard.putData("pose/Align to zero", Commands.runOnce(() -> { PoseSubsystem.setTargetAngle(new Rotation2d()); }).withName("Align to zero"));
        SmartDashboard.putData("pose/Align to 90", Commands.runOnce(() -> { PoseSubsystem.setTargetAngle(new Rotation2d(Math.PI / 2.0)); }).withName("Align to 90"));
        SmartDashboard.putData("pose/Clear target angle", Commands.runOnce(() -> { PoseSubsystem.setTargetAngle(null); }).withName("Clear target angle"));
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

    private void buildAutos(SendableChooser<Command> chooser) {
        Command smartHG =
            Commands.sequence(
                new PathPlannerAuto("SS Angled Start to H"),
                Commands.runOnce(() -> { DogLog.log("Auto/Status", "Ready for conditional part: " + s_Index.haveNote());}),
                Commands.either(
                    Commands.runOnce(() -> { DogLog.log("Auto/Status", "Running H-Shoot-G-Shoot");})
                    .andThen(new PathPlannerAuto("H-Shoot-G-Shoot")),
                    Commands.runOnce(() -> { DogLog.log("Auto/Status", "H-G-Shoot");}),
                    s_Index::haveNote
                ),
                Commands.runOnce(() -> { DogLog.log("Auto/Status", "Conditional part over");})
            ).withName("Smart HG");
        chooser.addOption("Smart HG", smartHG);

        // Command smartOTFHG =
        //     Commands.sequence(
        //         new PathPlannerAuto("Source-side OTF to H"),
        //         Commands.either(
        //             Commands.print("Running H-Shoot-G-Shoot").andThen(new PathPlannerAuto("H-Shoot-G-Shoot")),
        //             Commands.print("Running H-G-Shoot").andThen(new PathPlannerAuto("H-G-Shoot")),
        //             s_Index::haveNote
        //         )
        //     ).withName("Smart OTF HG");
        // chooser.addOption("Smart OTF HG", smartOTFHG);

        Command smartADEClose =
            Commands.sequence(
                new PathPlannerAuto("AS Angled + AD"),
                Commands.runOnce(() -> { DogLog.log("Auto/Status", "Ready for conditional part: " + s_Index.haveNote());}),
                Commands.either(
                    Commands.runOnce(() -> { DogLog.log("Auto/Status", "Running DE from close");}).andThen(new PathPlannerAuto("DE from close")),
                    Commands.runOnce(() -> { DogLog.log("Auto/Status", "Running D-E-Shoot");}).andThen(new PathPlannerAuto("D-E-Shoot")),
                    s_Index::haveNote
                ),
                Commands.runOnce(() -> { DogLog.log("Auto/Status", "Conditional part over");})
            ).withName("Smart ADE from Close");
        chooser.addOption("Smart ADE from Close", smartADEClose);

        Command smartADE =
            Commands.sequence(
                new PathPlannerAuto("AS Angled + AD"),
                Commands.runOnce(() -> { DogLog.log("Auto/Status", "Ready for conditional part: " + s_Index.haveNote());}),
                Commands.either(
                    Commands.runOnce(() -> { DogLog.log("Auto/Status", "Running DE from A");}).andThen(new PathPlannerAuto("DE from A")),
                    Commands.runOnce(() -> { DogLog.log("Auto/Status", "Running D-E-Shoot");}).andThen(new PathPlannerAuto("D-E-Shoot")),
                    s_Index::haveNote
                ),
                Commands.runOnce(() -> { DogLog.log("Auto/Status", "Conditional part over");})
            ).withName("Smart ADE");
        chooser.addOption("Smart ADE", smartADE);

        Command smartBCAD =
        Commands.sequence(
            new PathPlannerAuto("BCAD start"),
            Commands.runOnce(() -> { DogLog.log("Auto/Status", "Ready for conditional part: " + s_Index.haveNote());}),
            Commands.either(
                Commands.runOnce(() -> { DogLog.log("Auto/Status", "Running DE from A");}).andThen(new PathPlannerAuto("DE from A")),
                Commands.runOnce(() -> { DogLog.log("Auto/Status", "Running D-E-Shoot");}).andThen(new PathPlannerAuto("D-E-Shoot")),
                s_Index::haveNote
            ),
            Commands.runOnce(() -> { DogLog.log("Auto/Status", "Conditional part over");})
        ).withName("Smart BCAD");
        chooser.addOption("Smart BCAD", smartBCAD);

        Command smartBCdirectAD =
        Commands.sequence(
            new PathPlannerAuto("BC-direct-AD start"),
            Commands.runOnce(() -> { DogLog.log("Auto/Status", "Ready for conditional part: " + s_Index.haveNote());}),
            Commands.either(
                Commands.runOnce(() -> { DogLog.log("Auto/Status", "Running DE from A");}).andThen(new PathPlannerAuto("DE from A")),
                Commands.runOnce(() -> { DogLog.log("Auto/Status", "Running D-E-Shoot");}).andThen(new PathPlannerAuto("D-E-Shoot")),
                s_Index::haveNote
            ),
            Commands.runOnce(() -> { DogLog.log("Auto/Status", "Conditional part over");})
        ).withName("Smart BC-direct-AD");
        chooser.addOption("Smart BC-direct-AD", smartBCdirectAD);

        Command smartADEOTF =
            Commands.sequence(
                new PathPlannerAuto("Amp-side OTF + AD"),
                Commands.either(
                    new PathPlannerAuto("DE from A"),
                    new PathPlannerAuto("D-E-Shoot"),
                    s_Index::haveNote
                )
            ).withName("Smart ADE OTF");
        chooser.addOption("Smart ADE OTF", smartADEOTF);
    }

    public void teleopInit() {
        s_Swerve.stopSwerve();
        s_Vision.disableRotationTargetOverride();
        s_Swerve.setDriveMotorsToCoast();
    }

    public void teleopExit() {
        if (optBrakeAfterTeleop.get()) {
            s_Swerve.setDriveMotorsToBrake();
        }
    }
}