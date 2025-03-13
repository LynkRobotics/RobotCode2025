// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedAlert;
import frc.lib.util.LoggedCommands;
import frc.lib.util.TunableOption;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    /* Devices */
    private final TalonFX motor;

    /* Control Requests */
    // TODO Use PositionVoltage
    private final VoltageOut fastDeployControl = new VoltageOut(Constants.Climber.fastDeployVoltage).withEnableFOC(true);
    private final VoltageOut slowDeployControl = new VoltageOut(Constants.Climber.slowDeployVoltage).withEnableFOC(true);
    private final VoltageOut engageRetractControl = new VoltageOut(Constants.Climber.engageRetractVoltage).withEnableFOC(true);
    private final VoltageOut fastRetractControl = new VoltageOut(Constants.Climber.fastRetractVoltage).withEnableFOC(true);
    private final VoltageOut slowRetractControl = new VoltageOut(Constants.Climber.slowRetractVoltage).withEnableFOC(true);
    private final VoltageOut holdControl = new VoltageOut(Constants.Climber.holdVoltage).withEnableFOC(true);

    private boolean deployed = false;

    private static final TunableOption optOverrideClimberTiming = new TunableOption("Override Climber Timing", false);

    public ClimberSubsystem() {
        /* Devices */
        motor = new TalonFX(Constants.Climber.motorID, Constants.Climber.canBus);

        SmartDashboard.putData(LoggedCommands.runOnce("Coast Climber", () -> motor.setNeutralMode(NeutralModeValue.Coast)).ignoringDisable(true));
        SmartDashboard.putData(LoggedCommands.runOnce("Brake Climber", () -> motor.setNeutralMode(NeutralModeValue.Brake)).ignoringDisable(true));
        applyConfigs();
    }

    public void applyConfigs() {
        /* Configure the motor */
        var motorConfig = new TalonFXConfiguration();
        /* Set motor to brake control */
        motorConfig.MotorOutput.NeutralMode = Constants.Climber.motorNeutralValue;
        /* Set the motor direction */
        motorConfig.MotorOutput.Inverted = Constants.Climber.motorOutputInverted;
        /* Config the peak outputs */
        motorConfig.Voltage.PeakForwardVoltage = Constants.Climber.peakForwardVoltage;
        motorConfig.Voltage.PeakReverseVoltage = Constants.Climber.peakReverseVoltage;
        /* Apply Index Motor Configs */
        motor.getConfigurator().apply(motorConfig);
    }

    private boolean climberPartiallyDeployed() {
        return motor.getPosition().getValueAsDouble() >= Constants.Climber.fastDeployedPosition;
    }

    private boolean climberFullyDeployed() {
        return motor.getPosition().getValueAsDouble() >= Constants.Climber.fullyDeployedPosition;
    }

    public Command Deploy() {
        return LoggedCommands.either("Deploy Climber",
            Commands.sequence(
                Commands.runOnce(() -> deployed = true),
                Commands.either(
                    LoggedCommands.log("Climber already partially deployed"),
                    Commands.sequence(
                        LoggedCommands.runOnce("Set fast deploy voltage", () -> motor.setControl(fastDeployControl), this),
                        LoggedCommands.waitUntil("Wait for climber partial deployment", this::climberPartiallyDeployed)),
                    this::climberPartiallyDeployed),
                Commands.either(
                    LoggedCommands.log("Climber already fully deployed"),
                    Commands.sequence(
                        LoggedCommands.runOnce("Set slow deploy voltage", () -> motor.setControl(slowDeployControl), this),
                        LoggedCommands.waitUntil("Wait for climber full deployment", this::climberFullyDeployed)),
                    this::climberFullyDeployed),
                LoggedCommands.runOnce("Stop climber (deploy)", motor::stopMotor, this))
                .handleInterrupt(motor::stopMotor),
            Commands.sequence(
                LoggedCommands.log("Cannot deploy before cutoff time (" + Constants.Climber.timeCutoff + ")"),
                Commands.runOnce(() -> LoggedAlert.Error("Climber", "Too Early", "Cannot deploy before cutoff time"))
            ),
            () -> optOverrideClimberTiming.get() || DriverStation.getMatchTime() <= Constants.Climber.timeCutoff);
    }

    private boolean climberEngaged() {
        return motor.getPosition().getValueAsDouble() <= Constants.Climber.engageRetractedPosition;
    }

    private boolean climberPartiallyRetracted() {
        return motor.getPosition().getValueAsDouble() <= Constants.Climber.fastRetractedPosition;
    }

    private boolean climberFullyRetracted() {
        return motor.getPosition().getValueAsDouble() <= Constants.Climber.fullyRetractedPosition;
    }

    public Command Retract() {
        return LoggedCommands.either("Retract Climber",
            Commands.sequence(
                Commands.runOnce(LEDSubsystem::notifyClimbStarted),
                Commands.either(
                    LoggedCommands.log("Climber already engaged"),
                    Commands.sequence(
                        LoggedCommands.runOnce("Set engagement retract voltage", () -> motor.setControl(engageRetractControl), this),
                        LoggedCommands.waitUntil("Wait for climber engagement", this::climberEngaged)),
                    this::climberEngaged),
                Commands.either(
                    LoggedCommands.log("Climber already partially retracted"),
                    Commands.sequence(
                        LoggedCommands.runOnce("Set fast retract voltage", () -> motor.setControl(fastRetractControl), this),
                        LoggedCommands.waitUntil("Wait for partial climber retraction", this::climberPartiallyRetracted)),
                    this::climberPartiallyRetracted),
                Commands.either(
                    LoggedCommands.log("Climber already fully retracted"),
                    Commands.sequence(
                        LoggedCommands.runOnce("Set slow retract voltage", () -> motor.setControl(slowRetractControl), this),
                        LoggedCommands.waitUntil("Wait for full climber retraction", this::climberFullyRetracted)),
                    this::climberFullyRetracted),
                LoggedCommands.runOnce("Set hold voltage", () -> motor.setControl(holdControl), this),
                Commands.runOnce(LEDSubsystem::notifyClimbCompleted),
                LoggedCommands.idle("Hold climb", this))
                .handleInterrupt(motor::stopMotor),
            Commands.sequence(
                LoggedCommands.log("Cannot retract prior to deployment"),
                Commands.runOnce(() -> LoggedAlert.Error("Climber", "Not Deployed", "Cannot retract before deploy"))),
            () -> deployed);
    }

    @Override
    public void periodic() {
        DogLog.log("Climber/TorqueCurrent", motor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("Climber/Velocity", motor.getVelocity().getValueAsDouble());
        DogLog.log("Climber/Position", motor.getPosition().getValueAsDouble());
    }
}