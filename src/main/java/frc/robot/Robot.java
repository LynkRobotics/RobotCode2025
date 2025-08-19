// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.util.Elastic;
import frc.robot.autos.Autos;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.controls.Controls;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pose.Pose;
import frc.robot.subsystems.robotstate.RobotState;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;
import frc.robot.superstructure.Superstructure;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static final CTREConfigs ctreConfigs = new CTREConfigs();
    public static final Field2d field = new Field2d();
    public static final SendableChooser<String> fieldSelector = new SendableChooser<>();

    private Command autoCommand;
 
    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        // Serve up deployed files for Elastic dashboard
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

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

        // Ensure all subsystems get instantiated, and in order as necessary
        @SuppressWarnings("unused")
        Subsystem[] subsystems = new Subsystem[] {
            Climber.instance,
            EndEffector.instance,
            // Elevator.instance,
            Intake.instance,
            // LED.instance,
            // RobotState.instance,
            Swerve.instance,
            Vision.instance,
            Pose.instance,
            // Controls.instance,
            // Superstructure.instance,
            // Autos.instance
        };
        
        if (Constants.atHQ) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }
        Controls.instance.configureButtonBindings();

        DogLog.log("Misc/Robot Status", "Robot has Started");
        SmartDashboard.putData("Field Selector", fieldSelector);
        SmartDashboard.putData("Field", field);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        DogLog.log("Misc/FMS Match Time", DriverStation.getMatchTime());
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        DogLog.log("Misc/Robot Status", "Robot has been disabled");
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        DogLog.log("Misc/Robot Status", "Auto has begun");

        // Ensure the Swerve subsystem doesn't run a default command, in case we previously were in teleop mode
        Command oldDefault = Swerve.instance.getDefaultCommand();
        Swerve.instance.removeDefaultCommand();
        if (oldDefault != null && oldDefault.isScheduled()) {
            oldDefault.cancel();
        }

        // Schedule the autonomous command
        autoCommand = Autos.instance.getAutonomousCommand();
        if (autoCommand != null) {
            DogLog.log("Misc/Robot Status", "Running auto command " + autoCommand.getName());
            autoCommand.schedule();
        }

        if (!Constants.atHQ) {
            Elastic.selectTab("Primary");
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        DogLog.log("Misc/Robot Status", "TeleOp has begun");

        // Makes sure that the autonomous command stops running when teleop starts running
        if (autoCommand != null) {
            autoCommand.cancel();
        }

        // Run the TeleOp Swerve command by default
        Swerve swerve = Swerve.instance;
        swerve.stopSwerve();
        CommandScheduler.getInstance().schedule(swerve.BrakeDriveMotors());
        swerve.setDefaultCommand(Controls.instance.TeleOpSwerve());

        if (!Constants.atHQ) {
            Elastic.selectTab("Primary");
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }

    public static boolean isRed() {
        var alliance = DriverStation.getAlliance();

        assert alliance.isPresent() : "Cannot determine Alliance color";

        DogLog.log("DriverStation/Status", "Alliance recorded as " + alliance.toString());
        return alliance.get() == DriverStation.Alliance.Red;
    }
}