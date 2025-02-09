package frc.robot;

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
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ElevatorSubsystem.Stop;
import frc.robot.subsystems.RobotState.GamePiece;

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
            .withNtPublish(true));

        DogLog.log("Misc/RIO Serial Number", RobotController.getSerialNumber());
        DogLog.log("Misc/Is Rocky?", Constants.isRocky);

        // Initial Subsystems
        s_Swerve = new Swerve();
        s_Vision = new VisionSubsystem();
        s_Pose = new PoseSubsystem(s_Swerve, s_Vision);
        s_RobotState = new RobotState(s_Pose);
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
                s_Swerve::getSpeedLimitRot
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
        final Trigger moveElevator = driver.leftBumper();
        final Trigger score = driver.rightBumper();
        final Trigger L4 = driver.y();
        final Trigger L3 = driver.x();
        final Trigger L2 = driver.b();
        final Trigger L1 = driver.a();
        final Trigger alignLeft = driver.povLeft();
        final Trigger toggleGamePiece = driver.back();

        moveElevator.whileTrue(s_Elevator.GoToNext());
        score.onTrue(RobotState.ScoreGamePiece());
        L4.onTrue(LoggedCommands.runOnce("Set stop to L4", () -> s_Elevator.setNextStop(Stop.L4)));
        L3.onTrue(
            Commands.either(
                LoggedCommands.runOnce("Set stop to L3 Algae", () -> s_Elevator.setNextStop(Stop.L3_ALGAE)),
                LoggedCommands.runOnce("Set stop to L3", () -> s_Elevator.setNextStop(Stop.L3)),
                () -> RobotState.getActiveGamePiece() == GamePiece.ALGAE));
        L2.onTrue(
            Commands.either(
                LoggedCommands.runOnce("Set stop to L2 Algae", () -> s_Elevator.setNextStop(Stop.L2_ALGAE)),
                LoggedCommands.runOnce("Set stop to L2", () -> s_Elevator.setNextStop(Stop.L2)),
                () -> RobotState.getActiveGamePiece() == GamePiece.ALGAE));
        L1.onTrue(LoggedCommands.runOnce("Set stop to L1", () -> s_Elevator.setNextStop(Stop.L1)));

        toggleGamePiece.onTrue(RobotState.ToggleGamePiece());

        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("Approach E");
            PathConstraints constraints = new PathConstraints(2.0, 1.0, Units.degreesToRadians(360.0), Units.degreesToRadians(720.0));
            alignLeft.whileTrue(LoggedCommands.logWithName("Auto Align Left", AutoBuilder.pathfindThenFollowPath(path, constraints)));
        } catch (Exception exception) {
            LoggedAlert.Error("PathPlanner", "Left Approach Bind Failure", exception.getMessage());
        }

        SmartDashboard.putData("Disable speed limit", Commands.runOnce(s_Swerve::disableSpeedLimit));
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