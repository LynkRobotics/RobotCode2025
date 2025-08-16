// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedCommands;
import frc.robot.Ports;
import frc.robot.subsystems.intake.IntakeConstants.IntakePosition;

public class Intake extends SubsystemBase {
    public static final Intake instance = new Intake();
    public static final Timer stallTimer = new Timer();

    private IntakePosition desiredState = IntakePosition.RETRACTED;
    private boolean atDesiredState = true;

    /* Devices */
    private final TalonFX deployMotor;
    private final TalonFX intakeMotor;
    private final TalonFX indexMotor;

    /* Control Requests */
    private final VoltageOut deployZeroingControl = new VoltageOut(IntakeConstants.deployZeroingVoltage).withEnableFOC(false);
    private final VoltageOut indexControl = new VoltageOut(IntakeConstants.indexVoltage).withEnableFOC(true);
    private final VoltageOut indexExpelControl = new VoltageOut(IntakeConstants.indexExpelVoltage).withEnableFOC(true);
    private final VoltageOut intakeControl = new VoltageOut(IntakeConstants.intakeVoltage).withEnableFOC(true);
    private final VoltageOut intakeExpelControl = new VoltageOut(IntakeConstants.intakeExpelVoltage).withEnableFOC(true);

    public Intake() {
        /* Devices */
        deployMotor = new TalonFX(Ports.CORAL_DEPLOY.id, Ports.CORAL_DEPLOY.bus.name);
        intakeMotor = new TalonFX(Ports.CORAL_ROLLERS.id, Ports.CORAL_ROLLERS.bus.name);
        indexMotor = new TalonFX(Ports.INDEXER.id, Ports.INDEXER.bus.name);

        /* Configs */
        deployMotor.getConfigurator().apply(IntakeConstants.getDeployConfig());
        intakeMotor.getConfigurator().apply(IntakeConstants.getIntakeConfig());
        indexMotor.getConfigurator().apply(IntakeConstants.getIndexConfig());

        SmartDashboard.putData("Intake/Zero Deploy", ZeroIntake());

        deployMotor.setPosition(IntakePosition.FULL_STOW.position);
        // deployMotor.setControl(desiredState.control);
    }

    public Command Deploy() {
        return LoggedCommands.runOnce("Deploy Intake", () -> {
            deployMotor.setControl(IntakePosition.DEPLOYED.control);
            intakeMotor.setControl(intakeControl);
            indexMotor.setControl(indexControl); 
        }, this);
    }

    public Command Expel() {
        return LoggedCommands.print("Expel Intake", "TODO Implement Intake expel");
    }

    public Command Retract() {
        return LoggedCommands.parallel("Retract Intake", 
            Commands.print("TODO Moving intake"), 
            Commands.runOnce(() -> { intakeMotor.stopMotor(); indexMotor.stopMotor(); }, this)
        );
    }

    public Command StopIntake() {
        return LoggedCommands.runOnce("Stopping Coral Intake", null);
    }

    // Gently deploy intake until it stalls to recalibrate the zero position
    public Command ZeroIntake() {
        return LoggedCommands.sequence("Zeroing Coral Intake",
            Commands.runOnce(() -> deployMotor.setControl(deployZeroingControl), this),
            Commands.waitUntil(() -> stallTimer.isRunning() && stallTimer.hasElapsed(IntakeConstants.deployStallTime.in(Units.Seconds))),
            LoggedCommands.runOnce("Stop intake deploy", () -> {
                deployMotor.stopMotor();
                deployMotor.setPosition(IntakePosition.DEPLOYED.position);
            }, this));
    }

    @Override
    public void periodic() {
        Command currentCommand = getCurrentCommand();
        DogLog.log("Intake/Current Command", currentCommand == null ? "None" : currentCommand.getName());
        DogLog.log("Intake/Desired State", desiredState.name());
        DogLog.log("Intake/At Desired State", atDesiredState);
        DogLog.log("Intake/Deploy Current", deployMotor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("Intake/Deploy Velocity", deployMotor.getVelocity().getValueAsDouble());
        DogLog.log("Intake/Deploy Position", deployMotor.getPosition().getValueAsDouble());
        DogLog.log("Intake/Intake Current", intakeMotor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("Intake/Intake Velocity", intakeMotor.getVelocity().getValueAsDouble());
        DogLog.log("Intake/Index Current", indexMotor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("Intake/Index Velocity", indexMotor.getVelocity().getValueAsDouble());

        // Detect intake deployment stalls by checking the current
        if (deployMotor.getTorqueCurrent().getValue().lt(IntakeConstants.deployStallCurrent)) {
            if (!stallTimer.isRunning()) {
                stallTimer.restart();
            }
        } else if (stallTimer.isRunning()) {
            stallTimer.stop();
        }
    }
}