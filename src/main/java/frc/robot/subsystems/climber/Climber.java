// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedCommands;
import frc.robot.Ports;
import frc.robot.subsystems.controls.Controls;
import frc.robot.subsystems.climber.ClimberConstants.ClimberPosition;

public class Climber extends SubsystemBase {
    public static final Climber instance = new Climber();

    public enum ClimbState {
        NONE,
        GRABBING,
        GRABBED,
        CLIMBING,
        CLIMBED
    }

    private ClimbState climbState = ClimbState.NONE;

    /* Devices */
    private final TalonFX deployMotor;
    private final TalonFX intakeMotor;

    /* Control Requests */
    private final ControlRequest intakeControl = new VoltageOut(ClimberConstants.intakeVoltage);
    private final ControlRequest resetControl = new VoltageOut(ClimberConstants.resetVoltage);

    private final Debouncer stallDebouncer = new Debouncer(ClimberConstants.stallPeriod.in(Units.Seconds));

    public Climber() {
        /* Devices */
        deployMotor = new TalonFX(Ports.CLIMBER_DEPLOY.id, Ports.CLIMBER_DEPLOY.bus.name);
        deployMotor.getConfigurator().apply(ClimberConstants.getDeployMotorConfig());
        intakeMotor = new TalonFX(Ports.CLIMBER_ROLLERS.id, Ports.CLIMBER_ROLLERS.bus.name);
        intakeMotor.getConfigurator().apply(ClimberConstants.getIntakeMotorConfig());

        deployMotor.setPosition(ClimberConstants.startsClear ? ClimberPosition.CLEAR.angle : ClimberPosition.STOWED.angle);
        deployMotor.setControl(ClimberPosition.CLEAR.control);

        SmartDashboard.putData("Climber/Start Reset", StartReset());
        SmartDashboard.putData("Climber/Stop Reset", StopReset());
    }

    private Command StartReset() {
        return LoggedCommands.runOnce("Start Climber Reset", () -> deployMotor.setControl(resetControl), this);
    }

    private Command StopReset() {
        return LoggedCommands.runOnce("Stop Climber Reset", () -> {
            deployMotor.stopMotor();
            deployMotor.setNeutralMode(NeutralModeValue.Coast);
        }, this);
    }

    private Command Deploy() {
        return LoggedCommands.runOnce("Deploy climber", () -> {
            deployMotor.setControl(ClimberPosition.DEPLOYED.control);
        }, this);
    }

    private Command FullyStow() {
        return LoggedCommands.runOnce("Fully stow climber", () -> {
            deployMotor.setControl(ClimberPosition.FULLY_STOWED.control);
        }, this);
    }

    private Command Intake() {
        return LoggedCommands.runOnce("Intake climber", () -> intakeMotor.setControl(intakeControl), this);
    }

    public Command GrabCage() {
        return LoggedCommands.sequence("Grab Cage",
            Commands.runOnce(() -> climbState = ClimbState.GRABBING),
            Deploy(),
            Intake(),
            LoggedCommands.waitSeconds("Pre-intake delay", 1.5),
            LoggedCommands.waitUntil("Wait until climber intake stalled", this::intakeStalled),
            LoggedCommands.waitSeconds("Post-intake delay", 0.5),
            Commands.runOnce(() -> climbState = ClimbState.GRABBED),
            Controls.instance.TriggerRumble());
    }

    public Command Retract() {
        return LoggedCommands.sequence("Retract climber",
            Commands.runOnce(() -> climbState = ClimbState.CLIMBING),
            LoggedCommands.runOnce("Stop climber intake", () -> intakeMotor.stopMotor(), this),
            FullyStow());
    }

    public Command RetractAndWait() {
        return LoggedCommands.sequence("Retract climber and wait",
            Retract(),
            LoggedCommands.waitUntil("Wait until stowed", this::isFullyStowed),
            Commands.runOnce(() -> climbState = ClimbState.CLIMBED));
    }

    public ClimbState getClimbState() {
        return climbState;
    }

    private boolean intakeStalled() {
        return stallDebouncer.calculate(intakeMotor.getTorqueCurrent().getValue().gt(ClimberConstants.currentStallThreshold));
    }

    private boolean isFullyStowed() {
        return deployMotor.getPosition().getValue().minus(ClimberPosition.FULLY_STOWED.angle).abs(Units.Rotations) <= ClimberPosition.EPISILON.angle.in(Units.Rotations);
    }

    @Override
    public void periodic() {
        Command currentCommand = getCurrentCommand();
        DogLog.log("Climber/Current Command", currentCommand == null ? "None" : currentCommand.getName());
        DogLog.log("Climber/Deploy Current", deployMotor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("Climber/Deploy Voltage", deployMotor.getMotorVoltage().getValueAsDouble());
        DogLog.log("Climber/Deploy Velocity", deployMotor.getVelocity().getValueAsDouble());
        DogLog.log("Climber/Deploy Position", deployMotor.getPosition().getValueAsDouble());
        DogLog.log("Climber/Intake Current", intakeMotor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("Climber/Intake Velocity RPM", intakeMotor.getVelocity().getValue().in(Units.RPM));
        DogLog.log("Climber/Intake Velocity RPS", intakeMotor.getVelocity().getValue().in(Units.RotationsPerSecond));
        DogLog.log("Climber/Intake Stalled", intakeStalled());
    }
}