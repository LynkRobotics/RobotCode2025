// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedCommands;
import frc.robot.subsystems.controls.Controls;

public class Climber extends SubsystemBase {
    public static final Climber instance = new Climber();

    /* Devices */
    private final TalonFX deployMotor;
    private final TalonFX intakeMotor;

    /* Control Requests */
    private final VoltageOut intakeControl = new VoltageOut(ClimberConstants.intakeVoltage);

    public Climber() {
        /* Devices */
        deployMotor = new TalonFX(ClimberConstants.deployMotorID, ClimberConstants.canBus);
        deployMotor.getConfigurator().apply(ClimberConstants.getDeployMotorConfig());
        intakeMotor = new TalonFX(ClimberConstants.intakeMotorID, ClimberConstants.canBus);
        intakeMotor.getConfigurator().apply(ClimberConstants.getIntakeMotorConfig());

        //SmartDashboard.putData(LoggedCommands.runOnce("Coast Climber", () -> motor.setNeutralMode(NeutralModeValue.Coast)).ignoringDisable(true));
        //SmartDashboard.putData(LoggedCommands.runOnce("Brake Climber", () -> motor.setNeutralMode(NeutralModeValue.Brake)).ignoringDisable(true));
    }

    private Command Deploy() {
        return LoggedCommands.print("Deploy climber", "TODO Implement climber deploy");
        // TODO move with magic motion
    }

    private Command Intake() {
        intakeMotor.setControl(intakeControl);
        return LoggedCommands.print("Intake cage", "TODO Implement climber cage intake");
    }

    public Command DeployAndIntake() {
        return LoggedCommands.deadline("Deploy and Intake climber", 
            WaitForIntake(),
            Deploy(),
            Intake()
        );
    }

    private Command WaitForIntake() {
        return LoggedCommands.sequence("Wait for Intake",
            LoggedCommands.waitUntil("Wait for intake motor stalled", () -> false /* TODO intakeMotor.isStalled() */),
            LoggedCommands.run("Stop climber intake due to stall", intakeMotor::stopMotor, this),
            LoggedCommands.waitSeconds("Post climber intake delay", 0.5),
            Controls.instance.TriggerRumble(),
            LoggedCommands.print("Flash LEDs", "TODO Flash LEDs"));
    }

    public Command Retract() {
        intakeMotor.stopMotor();
        return LoggedCommands.print("Retract climber", "TODO Implement climber retract");
        // TODO move with magic motion
        // Idle rollers and move to pull position
        // In superstructue, then move intake into full stow position
    }

    @Override
    public void periodic() {
        Command currentCommand = getCurrentCommand();
        DogLog.log("Climber/Current Command", currentCommand == null ? "None" : currentCommand.getName());
        DogLog.log("Climber/Deploy Current", deployMotor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("Climber/Deploy Velocity", deployMotor.getVelocity().getValueAsDouble());
        DogLog.log("Climber/Deploy Position", deployMotor.getPosition().getValueAsDouble());
        DogLog.log("Climber/Intake Current", intakeMotor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("Climber/Intake Velocity", intakeMotor.getVelocity().getValueAsDouble());
    }
}