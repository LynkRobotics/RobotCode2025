package frc.robot;

import java.util.Set;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.LoggedCommands;
import frc.robot.autos.Autos;
import frc.robot.subsystems.robotstate.RobotState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.controls.Controls;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pose.Pose;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;

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
    /* Subsystems */
    @SuppressWarnings ("unused")
    private final RobotState s_RobotState;
    private final Swerve s_Swerve;
    @SuppressWarnings ("unused")
    private final LED s_LED;
    private final Vision s_Vision;
    private final Pose s_Pose;
    @SuppressWarnings ("unused")
    private final Elevator s_Elevator;
    @SuppressWarnings ("unused")
    private final EndEffector s_EndEffector;
    @SuppressWarnings ("unused")
    private final Index s_Index;
    @SuppressWarnings ("unused")
    private final Climber s_Climber;

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
        s_Vision = new Vision();
        s_Pose = new Pose(s_Swerve, s_Vision);
        s_RobotState = new RobotState();
        s_Elevator = new Elevator();
        s_EndEffector = new EndEffector();
        s_LED = new LED();
        s_Index = new Index();
        s_Climber = new Climber();

        // Default named commands for PathPlanner
        SmartDashboard.putNumber("auto/Startup delay", 0.0);
        Autos.autoNamedCommand("Startup delay", Commands.defer(() -> Commands.waitSeconds(SmartDashboard.getNumber("auto/Startup delay", 0.0)), Set.of()));
        Autos.autoNamedCommand("Stop", Commands.runOnce(s_Swerve::stopSwerve));

        SmartDashboard.putNumber("TeleOp Speed Governor", 1.0);

        SmartDashboard.putData(LoggedCommands.runOnce("Zero Gyro", s_Pose::zeroGyro, s_Swerve));
        SmartDashboard.putData(LoggedCommands.runOnce("Reset heading", s_Pose::resetHeading, s_Swerve));

        SmartDashboard.putData(LoggedCommands.runOnce("autoSetup/Set Swerve Coast", s_Swerve::setMotorsToCoast, s_Swerve).ignoringDisable(true));
        SmartDashboard.putData(LoggedCommands.runOnce("autoSetup/Set Swerve Brake", s_Swerve::setMotorsToBrake, s_Swerve).ignoringDisable(true));
        SmartDashboard.putData(LoggedCommands.run("autoSetup/Set Swerve Aligned", s_Swerve::alignStraight, s_Swerve).ignoringDisable(true));

        Controls.instance.configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
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
        s_Swerve.setDefaultCommand(Controls.instance.TeleOpSwerve());
    }
}