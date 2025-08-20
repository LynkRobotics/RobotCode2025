package frc.robot.subsystems.controls;

import static frc.robot.Options.optAutoReefAiming;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.LoggedCommands;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.Stop;
import frc.robot.subsystems.pose.Pose;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.superstructure.Superstructure;
import frc.robot.commands.TeleopSwerve;

public class Controls extends SubsystemBase{
    public static final Controls instance = new Controls();

    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);

    /* Drive Controls */
    private final Supplier<Double> translation = driver::getLeftY;
    private final Supplier<Double> strafe = driver::getLeftX;
    private final Supplier<Double> rotation = driver::getRightX;

    public Controls() {
        SmartDashboard.putNumber("TeleOp Speed Governor", 1.0);

        Pose pose = Pose.instance;
        SmartDashboard.putData(LoggedCommands.runOnce("Zero Gyro", pose::zeroGyro));
        SmartDashboard.putData(LoggedCommands.runOnce("Reset heading", pose::resetHeading));

        Swerve swerve = Swerve.instance;
        SmartDashboard.putData(LoggedCommands.runOnce("autoSetup/Set Swerve Coast", swerve::setMotorsToCoast, swerve).ignoringDisable(true));
        SmartDashboard.putData(LoggedCommands.runOnce("autoSetup/Set Swerve Brake", swerve::setMotorsToBrake, swerve).ignoringDisable(true));
        SmartDashboard.putData(LoggedCommands.run("autoSetup/Set Swerve Aligned", swerve::alignStraight, swerve).ignoringDisable(true));
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

    public Command TriggerRumble() {
        Command rumbleCommmand = Rumble();

        return Commands.runOnce(() -> rumbleCommmand.schedule());
    }

    public void configureButtonBindings() {
        /* Driver Buttons */
        final Trigger intakeCoral = driver.leftBumper();
        final Trigger intakeGroundAlgae = driver.rightBumper();
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
        intakeCoral.whileTrue(Superstructure.instance.SmartIntake());
        // zero.onTrue(Elevator.instance.Zero());
        // unjam.onTrue(Intake.instance.Expel());
        // score.whileTrue(Superstructure.ScoreGamePiece()); // Also useful to dump Algae or put it into Processor

        L4.onTrue(Superstructure.instance.SetStop(Stop.L4));
        L3.onTrue(Superstructure.instance.SetStop(Stop.L3));
        L2.onTrue(Superstructure.instance.SetStop(Stop.L2));
        L1.onTrue(Superstructure.instance.SetStop(Stop.L1));

        // goLeft.whileTrue(Superstructure.instance.SmartScore(true));
        // goRight.whileTrue(Superstructure.instance.SmartScore(false));

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

        // driver.povDown().onTrue(Climber.instance.DeployAndIntake());
        // driver.povUp().onTrue(Climber.instance.Retract());
    }

    private double speedLimitFactor() {
        return 1.0 - Elevator.instance.raisedPercentage() * (1.0 - ElevatorConstants.speedLimitAtMax);
    }

    public Command TeleOpSwerve() {
        return new TeleopSwerve(
                Swerve.instance,
                () -> -translation.get() * Constants.driveStickSensitivity,
                () -> -strafe.get() * Constants.driveStickSensitivity,
                () -> -rotation.get() * Constants.turnStickSensitivity,
                this::speedLimitFactor
            );
    }
}