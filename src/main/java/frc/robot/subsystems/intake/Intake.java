// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedCommands;

public class Intake extends SubsystemBase {
    public static final Intake instance = new Intake();

    private enum IntakeState {
        START(0.0),
        DEPLOYED(0.0),
        RETRACTED(0.0);

        public double position;

        IntakeState(double position) {
            this.position = position;
        }
    }
    
    private IntakeState desiredState = IntakeState.START;
    private boolean atDesiredState = true;

    /* Devices */
    private final TalonFX deployMotor;
    private final TalonFX intakeMotor;
    private final TalonFX indexMotor;

    /* Control Requests */
    // private final VoltageOut intakeControl = new VoltageOut(IntakeConstants.intakeVoltage).withEnableFOC(true);

    public Intake() {
        /* Devices */
        deployMotor = new TalonFX(IntakeConstants.deployMotorID, IntakeConstants.canBus);
        intakeMotor = new TalonFX(IntakeConstants.intakeMotorID, IntakeConstants.canBus);
        indexMotor = new TalonFX(IntakeConstants.indexMotorID, IntakeConstants.canBus);

        applyConfigs();
    }

    public void applyConfigs() {
        /* Configure the motor */
        var motorConfig = new TalonFXConfiguration();
        /* Set motor to brake control */
        motorConfig.MotorOutput.NeutralMode = IntakeConstants.motorNeutralValue;
        /* Set the motor direction */
        motorConfig.MotorOutput.Inverted = IntakeConstants.motorOutputInverted;
        /* Config the peak outputs */
        motorConfig.Voltage.PeakForwardVoltage = IntakeConstants.peakForwardVoltage;
        motorConfig.Voltage.PeakReverseVoltage = IntakeConstants.peakReverseVoltage;
        /* Apply motor Configs */
        deployMotor.getConfigurator().apply(motorConfig);
        intakeMotor.getConfigurator().apply(motorConfig);
        indexMotor.getConfigurator().apply(motorConfig);
    }

    public Command Deploy() {
        return LoggedCommands.print("Deploy Intake", "TODO Implement Intake deploy");
    }

    public Command Reverse() {
        return LoggedCommands.print("Reverse Intake", "TODO Implement Intake reverse");
    }

    public Command Retract() {
        return LoggedCommands.print("Retract Intake", "TODO Implement Intake retract");
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
    }
}