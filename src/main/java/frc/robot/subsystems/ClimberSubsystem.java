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
    private final VoltageOut deployControl = new VoltageOut(Constants.Climber.deployVoltage).withEnableFOC(true);
    private final VoltageOut retractControl = new VoltageOut(Constants.Climber.retractVoltage).withEnableFOC(true);

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

    public Command Deploy() {
        return LoggedCommands.either("Deploy Climber",
            Commands.sequence(
                Commands.runOnce(() -> deployed = true),
                LoggedCommands.runOnce("Set deploy voltage", () -> motor.setControl(deployControl)),
                LoggedCommands.waitUntil("Wait for climber deployment", () -> motor.getPosition().getValueAsDouble() >= Constants.Climber.deployedPosition),
                LoggedCommands.runOnce("Stop climber (deploy)", motor::stopMotor)),
            Commands.sequence(
                LoggedCommands.log("Cannot deploy before cutoff time (" + Constants.Climber.timeCutoff + ")"),
                Commands.runOnce(() -> LoggedAlert.Error("Climber", "Too Early", "Cannot deploy before cutoff time"))
            ),
            () -> optOverrideClimberTiming.get() || DriverStation.getMatchTime() <= Constants.Climber.timeCutoff);
    }

    public Command Retract() {
        return LoggedCommands.either("Retract Climber",
            Commands.sequence(
                LoggedCommands.runOnce("Set retract voltage", () -> motor.setControl(retractControl)),
                LoggedCommands.waitUntil("Wait for climber retraction", () -> motor.getPosition().getValueAsDouble() <= Constants.Climber.retractedPosition),
                // TODO Hold position
                LoggedCommands.runOnce("Stop climber (retract)", motor::stopMotor)),
            Commands.sequence(
                LoggedCommands.log("Cannot retract prior to deployment"),
                Commands.runOnce(() -> LoggedAlert.Error("Climber", "Not Deployed", "Cannot retract before deploy"))
            ),
            () -> deployed);
    }

    @Override
    public void periodic() {
        DogLog.log("Climber/TorqueCurrent", motor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("Climber/Velocity", motor.getVelocity().getValueAsDouble());
        DogLog.log("Climber/Position", motor.getPosition().getValueAsDouble());
    }
}