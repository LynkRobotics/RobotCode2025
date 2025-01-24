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
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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

        s_Shooter.setDefaultCommand(Commands.startEnd(s_Shooter::idle, () -> {}, s_Shooter).withName("Shooter Idle"));
        s_Index.setDefaultCommand(Commands.startEnd(s_Index::stop, () -> {}, s_Index).withName("Index Stop"));

        SmartDashboard.putData("Command scheduler", CommandScheduler.getInstance());
        SmartDashboard.putData("Shoot Command", new ShootCommand(s_Shooter, s_Index, s_Swerve).withTimeout(3.0));

        // Default named commands for PathPlanner
        SmartDashboard.putNumber("auto/Startup delay", 0.0);
        NamedCommands.registerCommand("Done", new PrintCommand("Done"));
        NamedCommands.registerCommand("Start", new PrintCommand("Starting"));
        NamedCommands.registerCommand("Startup delay", new DeferredCommand(() ->Commands.waitSeconds(SmartDashboard.getNumber("auto/Startup delay", 0.0)), Set.of()));
        NamedCommands.registerCommand("Shoot",
            Commands.runOnce(() -> { DogLog.log("Auto/Status", "Named 'Shoot' command starting");})
            .andThen(
                (Commands.runOnce(() -> { DogLog.log("Auto/Status", "Before ShootCommand");}).andThen(new ShootCommand(s_Shooter, s_Index, s_Swerve)).andThen(Commands.runOnce(() -> { DogLog.log("Auto/Status", "After ShootCommand");})))
                 .raceWith(Commands.runOnce(() -> { DogLog.log("Auto/Status", "Before AimCommand");}).andThen(new AimCommand(s_Swerve, s_Vision)).andThen(Commands.runOnce(() -> { DogLog.log("Auto/Status", "After AimCommand");})))
                 .raceWith(Commands.runOnce(() -> { DogLog.log("Auto/Status", "Before waitSeconds");}).andThen(Commands.waitSeconds(2.50)).andThen(Commands.runOnce(() -> { DogLog.log("Auto/Status", "After waitSeconds");}))))
            .andThen(Commands.runOnce(() -> { DogLog.log("Auto/Status", "After race group");}))
            .andThen(Commands.runOnce(() -> { DogLog.log("Auto/Status", "Named 'Shoot' command ending");}))
        );
        NamedCommands.registerCommand("Shoot without aiming",
            Commands.runOnce(() -> { DogLog.log("Auto/Status", "Begin shot w/o aim");})
            .andThen(
                (new ShootCommand(s_Shooter, s_Index, s_Swerve, false)
                .raceWith(Commands.waitSeconds(1.50))))
            .andThen(Commands.runOnce(() -> { DogLog.log("Auto/Status", "Shot w/o aim complete");}))  
        );
        NamedCommands.registerCommand("Fixed SW shot",
            Commands.runOnce(() -> { DogLog.log("Auto/Status", "Begin SW shot");})
            .andThen(Commands.runOnce(() -> { s_Shooter.setNextShot(Speed.SUBWOOFER); }))
            .andThen(
                (new ShootCommand(s_Shooter, s_Index, false)
                .raceWith(Commands.waitSeconds(1.50))))
            .andThen(Commands.runOnce(() -> { DogLog.log("Auto/Status", "SW complete");}))
        );
        NamedCommands.registerCommand("Fixed AS shot",
            Commands.runOnce(() -> { DogLog.log("Auto/Status", "Begin AS shot");})
            .andThen(Commands.runOnce(() -> { s_Shooter.setNextShot(Speed.AMPSIDE); }))
            .andThen(
                (new ShootCommand(s_Shooter, s_Index, false)
                .raceWith(Commands.waitSeconds(1.50))))
            .andThen(Commands.runOnce(() -> { DogLog.log("Auto/Status", "AS complete");}))
        );
        NamedCommands.registerCommand("Shoot OTF",
            Commands.runOnce(() -> { DogLog.log("Auto/Status", "Begin OTF");})
            .andThen(Commands.runOnce(() -> { s_Shooter.setNextShot(Speed.OTF); }))
            .andThen(
                (new ShootCommand(s_Shooter, s_Index, false)
                .raceWith(Commands.waitSeconds(1.50))))
            .andThen(Commands.runOnce(() -> { DogLog.log("Auto/Status", "Shot OTF complete");}))
        );
        NamedCommands.registerCommand("Amp-side OTF Shot",
            Commands.runOnce(() -> { DogLog.log("Auto/Status", "Begin Amp-side OTF Shot");})
            .andThen(Commands.runOnce(() -> { s_Shooter.setNextShot(Speed.AMPSIDEOTF); }))
            .andThen(
                (new ShootCommand(s_Shooter, s_Index, false)
                .raceWith(Commands.waitSeconds(1.00))))
            .andThen(Commands.runOnce(() -> { DogLog.log("Auto/Status", "Amp-side OTF Shot complete");}))
        );
        NamedCommands.registerCommand("Source-side OTF Shot",
            Commands.runOnce(() -> { DogLog.log("Auto/Status", "Begin Source-side OTF Shot");})
            .andThen(Commands.runOnce(() -> { s_Shooter.setNextShot(Speed.SOURCESIDEOTF); }))
            .andThen(
                (new ShootCommand(s_Shooter, s_Index, false)
                .raceWith(Commands.waitSeconds(1.00))))
            .andThen(Commands.runOnce(() -> { DogLog.log("Auto/Status", "Source-side OTF Shot complete");}))
        );
        NamedCommands.registerCommand("Intake note",
            Commands.runOnce(() -> { DogLog.log("Auto/Status", "Beginning Intake");})
            .andThen(new IntakeCommand(s_Intake, s_Index, driver.getHID()))
            .andThen(Commands.runOnce(() -> { DogLog.log("Auto/Status", "Intake Complete");}))
            );

        NamedCommands.registerCommand("Amp Shot",
            Commands.runOnce(() -> { DogLog.log("Auto/Status", "Begin Amp Shot");})
            .andThen(Commands.runOnce(() -> { s_Shooter.setNextShot(Speed.AMP); }))
            .andThen(
                (new ShootCommand(s_Shooter, s_Index, false)
                .raceWith(Commands.waitSeconds(1.00))))
            .andThen(Commands.runOnce(() -> { DogLog.log("Auto/Status", "Amp Shot complete");}))
        );
        NamedCommands.registerCommand("Bloop Shot",
            Commands.runOnce(() -> { DogLog.log("Auto/Status", "Begin Bloop Shot");})
            .andThen(Commands.runOnce(() -> { s_Shooter.setNextShot(Speed.BLOOP); }))
            .andThen(
                (new ShootCommand(s_Shooter, s_Index, false)
                .raceWith(Commands.waitSeconds(1.00))))
            .andThen(Commands.runOnce(() -> { DogLog.log("Auto/Status", "Bloop Shot complete");}))
        );
        NamedCommands.registerCommand("Slide Shot",
            Commands.runOnce(() -> { DogLog.log("Auto/Status", "Begin Slide shot");})
            .andThen(Commands.runOnce(() -> { s_Shooter.setNextShot(Speed.SLIDE); }))
            .andThen(
                (new ShootCommand(s_Shooter, s_Index, false)
                .raceWith(Commands.waitSeconds(1.00))))
            .andThen(Commands.runOnce(() -> { DogLog.log("Auto/Status", "Slide shot complete");}))
        );
        NamedCommands.registerCommand("Short Slide Shot",
            Commands.runOnce(() -> { DogLog.log("Auto/Status", "Begin Short Slide shot");})
            .andThen(Commands.runOnce(() -> { s_Shooter.setNextShot(Speed.SHORTSLIDE); }))
            .andThen(
                (new ShootCommand(s_Shooter, s_Index, false)
                .raceWith(Commands.waitSeconds(1.00))))
            .andThen(Commands.runOnce(() -> { DogLog.log("Auto/Status", "Short Slide shot complete");}))
        );
        NamedCommands.registerCommand("Special Shot",
            Commands.runOnce(() -> { DogLog.log("Auto/Status", "Begin Special shot");})
            .andThen(Commands.runOnce(() -> { s_Shooter.setNextShot(Speed.SPECIAL); }))
            .andThen(
                (new ShootCommand(s_Shooter, s_Index, false)
                .raceWith(Commands.waitSeconds(1.00))))
            .andThen(Commands.runOnce(() -> { DogLog.log("Auto/Status", "Special shot complete");}))
        );
        NamedCommands.registerCommand("Override rotation", Commands.runOnce(s_Vision::enableRotationTargetOverride));
        NamedCommands.registerCommand("Restore rotation", Commands.runOnce(s_Vision::disableRotationTargetOverride));
        NamedCommands.registerCommand("Stop", Commands.runOnce(s_Swerve::stopSwerve));
        NamedCommands.registerCommand("Set Instant Pose", Commands.runOnce(() ->
            {
                if (s_Vision.haveSpeakerTarget()) {
                    Pose2d pose = s_Vision.lastPose();
                    s_Pose.setPose(pose);
                    DogLog.log("Auto/Status", "Pose updated from vision: " + PoseSubsystem.prettyPose(pose));
                } else {
                    DogLog.log("Auto/Status", "Refusing to update pose from vision without a current speaker target");
                }
            } ));
        NamedCommands.registerCommand("Coast after auto", new CoastAfterAuto(s_Swerve));
        NamedCommands.registerCommand("Coast drive motors", Commands.runOnce(s_Swerve::setDriveMotorsToCoast));

        // Build an autoChooser (defaults to none)
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("auto/Auto Chooser", autoChooser);
        buildAutos(autoChooser);

        // During calibration allow for direct control
        //SmartDashboard.putNumber("Shooter voltage direct", 0.0);
        //SmartDashboard.putData("Set shooter voltage", s_Shooter.runOnce(() -> { s_Shooter.setVoltage(SmartDashboard.getNumber("Shooter voltage direct", 0)); }));
        //SmartDashboard.putData("Stop shooter", s_Shooter.runOnce(() -> { s_Shooter.setVoltage(0); }));

        SmartDashboard.putNumber("TeleOp Speed Governor", 1.0);

        // Allow for direct RPM setting
        SmartDashboard.putNumber("Shooter top RPM", 1000.0);
        SmartDashboard.putNumber("Shooter bottom RPM", 1000.0);
        SmartDashboard.putData("Idle shooter", s_Shooter.runOnce(() -> { s_Shooter.setRPM(500); }));
        SmartDashboard.putData("Zero Gyro", Commands.print("Zeroing gyro").andThen(Commands.runOnce(s_Pose::zeroGyro, s_Swerve)).andThen(Commands.print("Gyro zeroed")).withName("Zero Gyro")); //TODO: Test
        SmartDashboard.putData("Zero heading", Commands.print("Zeroing heading").andThen(Commands.runOnce(s_Pose::zeroHeading, s_Swerve)).andThen(Commands.print("Heading zeroed")).withName("Zero heading")); //TODO: Test
        SmartDashboard.putData("Reset heading", Commands.print("Resetting heading").andThen(Commands.runOnce(s_Pose::resetHeading, s_Swerve)).andThen(Commands.print("Heading reset")).withName("Reset heading"));

        // Allow for direct climber control
        SmartDashboard.putData("Stop climbers", Commands.runOnce(() -> { s_LeftClimber.stop(); s_RightClimber.stop(); }, s_LeftClimber, s_RightClimber));
        SmartDashboard.putData("Left down slow", Commands.runOnce(() -> { s_LeftClimber.applyVoltage(Constants.Climber.slowVoltage); }, s_LeftClimber));
        SmartDashboard.putData("Right down slow", Commands.runOnce(() -> { s_RightClimber.applyVoltage(Constants.Climber.slowVoltage); }, s_RightClimber));

        SmartDashboard.putNumber("Left climber voltage", 0.0);
        SmartDashboard.putNumber("Right climber voltage", 0.0);
        SmartDashboard.putData("Set climber voltage", Commands.runOnce(() -> { s_LeftClimber.applyVoltage(SmartDashboard.getNumber("Left climber voltage", 0.0)); s_RightClimber.applyVoltage(SmartDashboard.getNumber("Right climber voltage", 0.0));}, s_LeftClimber, s_RightClimber));
        SmartDashboard.putData("Zero climbers", Commands.runOnce(() -> { s_LeftClimber.zero(); s_RightClimber.zero(); }, s_LeftClimber, s_RightClimber));

        SmartDashboard.putNumber("Left climber target position", 0.0);
        SmartDashboard.putData("Set left climber position", new ClimberPositionCommand(SmartDashboard.getNumber("Left climber target position", 0.0), LEDSubsystem.TempState.RETRACTING, s_LeftClimber));
        SmartDashboard.putNumber("Right climber target position", 0.0);
        SmartDashboard.putData("Set right climber position", new ClimberPositionCommand(SmartDashboard.getNumber("Right climber target position", 0.0), LEDSubsystem.TempState.RETRACTING, s_RightClimber));

        SmartDashboard.putData("autoSetup/SetSwerveCoast", Commands.runOnce(() -> { DogLog.log("Auto/Status", "Coasting Swerve Motors");}).andThen(Commands.runOnce(s_Swerve::setMotorsToCoast, s_Swerve)).andThen(Commands.runOnce(() -> { DogLog.log("Auto/Status", "Swerve Motors Coasted");})).ignoringDisable(true));
        SmartDashboard.putData("autoSetup/SetSwerveBrake", Commands.runOnce(() -> { DogLog.log("Auto/Status", "Braking Swerve Motors");}).andThen(Commands.runOnce(s_Swerve::setMotorsToBrake, s_Swerve)).andThen(Commands.runOnce(() -> { DogLog.log("Auto/Status", "Swerve Motors Braked");})).ignoringDisable(true));
        SmartDashboard.putData("autoSetup/SetSwerveAligned", Commands.runOnce(() -> { DogLog.log("Auto/Status", "Aligning Swerve Motors");}).andThen(Commands.run(s_Swerve::alignStraight, s_Swerve)).andThen(Commands.runOnce(() -> { DogLog.log("Auto/Status", "Swerve Motors Aligned");})).ignoringDisable(true));

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
            Commands.sequence(
                Commands.runOnce(s_Swerve::enableSpeedLimit),
                Commands.either(
                    new ShooterIntakeCommand(s_Shooter, s_Index, driver.getHID()),
                    new IntakeCommand(s_Intake, s_Index, driver.getHID()),
                    optShooterIntake),
                Commands.runOnce(s_Swerve::disableSpeedLimit))
            .handleInterrupt(s_Swerve::disableSpeedLimit)
            .withName("Intake"));
        shooterButton.whileTrue(
            Commands.either(new ShootCommand(s_Shooter, s_Index,
                () -> SmartDashboard.getNumber("Shooter top RPM", 0.0),
                () -> SmartDashboard.getNumber("Shooter bottom RPM", 0.0)),
            new ShootCommand(s_Shooter, s_Index, s_Swerve),
            optDirectRPM)
            .withName("Shoot"));
        climberExtendButton.onTrue(
            Commands.sequence(
                Commands.runOnce(s_Swerve::enableSpeedLimit),
                Commands.parallel(
                    new ClimberPositionCommand(Constants.Climber.extendedPosition, LEDSubsystem.TempState.EXTENDING, s_LeftClimber),
                    new ClimberPositionCommand(Constants.Climber.extendedPosition, LEDSubsystem.TempState.EXTENDING, s_RightClimber)))
            .withName("Extend Climbers"));
        SmartDashboard.putData("Disable speed limit", Commands.runOnce(s_Swerve::disableSpeedLimit));
        // climberExtendButton.onTrue(
        //     Commands.either(
        //         new ClimberPositionCommand(Constants.Climber.extendedPosition, LEDSubsystem.TempState.EXTENDING, s_LeftClimber)
        //         .alongWith(new ClimberPositionCommand(Constants.Climber.extendedPosition, LEDSubsystem.TempState.EXTENDING, s_RightClimber)),
        //         new ClimberPositionCommand(Constants.Climber.midPosition, LEDSubsystem.TempState.EXTENDING, s_LeftClimber)
        //         .alongWith(new ClimberPositionCommand(Constants.Climber.midPosition, LEDSubsystem.TempState.EXTENDING, s_RightClimber)),
        //         () -> s_LeftClimber.getPosition() < Constants.Climber.midPosition + 2 * Constants.Climber.positionError ||
        //               s_RightClimber.getPosition() < Constants.Climber.midPosition + 2 * Constants.Climber.positionError));

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