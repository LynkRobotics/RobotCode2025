package frc.robot;

import static frc.robot.Options.optAutoReefAiming;
import static frc.robot.Options.optBonusCoralStandoff;

import java.util.EnumMap;
import java.util.Set;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.util.Units;
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
import frc.robot.Constants.Pose.ReefFace;
import frc.robot.Constants.Elevator.Stop;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

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
    private final LEDSubsystem s_Led;
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
        s_Led = new LEDSubsystem();
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

        // Build an autoChooser (defaults to none)
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("auto/Auto Chooser", autoChooser);
        // buildAutos(autoChooser);

        SmartDashboard.putNumber("TeleOp Speed Governor", 1.0);

        SmartDashboard.putData(LoggedCommands.runOnce("Zero Gyro", s_Pose::zeroGyro, s_Swerve));
        SmartDashboard.putData(LoggedCommands.runOnce("Reset heading", s_Pose::resetHeading, s_Swerve));

        SmartDashboard.putData(LoggedCommands.runOnce("autoSetup/Set Swerve Coast", s_Swerve::setMotorsToCoast, s_Swerve).ignoringDisable(true));
        SmartDashboard.putData(LoggedCommands.runOnce("autoSetup/Set Swerve Brake", s_Swerve::setMotorsToBrake, s_Swerve).ignoringDisable(true));
        SmartDashboard.putData(LoggedCommands.run("autoSetup/Set Swerve Aligned", s_Swerve::alignStraight, s_Swerve).ignoringDisable(true));

        for (ReefFace face: ReefFace.values()) {
            setFaceCommands(face);
        }

        // Configure the button bindings
        configureButtonBindings();

        // Debug PathPlanner commands
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("Approach E");
            PathConstraints constraints = new PathConstraints(2.0, 1.0, Units.degreesToRadians(360.0), Units.degreesToRadians(720.0));
            SmartDashboard.putData(LoggedCommands.logWithName("Auto Align Left", AutoBuilder.pathfindThenFollowPath(path, constraints)));
        } catch (Exception exception) {
            LoggedAlert.Error("PathPlanner", "Left Approach Bind Failure", exception.getMessage());
        }
    }

    private Command ScoreCoral(ReefFace face, boolean left) {
        return LoggedCommands.sequence("Auto Align " + (left ? "Left " : "Right ") + face.toString() + " & Score",
            LoggedCommands.parallel("PID Align " + (left ? "Left " : "Right ") + face.toString(),
                Commands.sequence(
                    new PIDSwerve(s_Swerve, s_Pose, left ? face.approachLeft : face.approachRight, false),
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
            RobotState.ScoreGamePiece());
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

        zero.onTrue(s_Elevator.Zero());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    // private void addAutoCommand(SendableChooser<Command> chooser, Command command) {
        // chooser.addOption(command.getName(), command);
    // }

    // private void buildAutos(SendableChooser<Command> chooser) {
        // TODO add programatically defined Autos as needed
    // }

    public void teleopInit() {
    }

    public void teleopExit() {
    }
}