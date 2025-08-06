// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedCommands;

public class Climber extends SubsystemBase {
    public static final Climber instance = new Climber();

    /* Devices */
    private final TalonFX deployMotor;
    private final TalonFX intakeMotor;

    /* Control Requests */

    public Climber() {
        /* Devices */
        deployMotor = new TalonFX(ClimberConstants.deployMotorID, ClimberConstants.canBus);
        deployMotor.getConfigurator().apply(ClimberConstants.getDeployMotorConfig());
        intakeMotor = new TalonFX(ClimberConstants.intakeMotorID, ClimberConstants.canBus);
        intakeMotor.getConfigurator().apply(ClimberConstants.getIntakeMotorConfig());

        //SmartDashboard.putData(LoggedCommands.runOnce("Coast Climber", () -> motor.setNeutralMode(NeutralModeValue.Coast)).ignoringDisable(true));
        //SmartDashboard.putData(LoggedCommands.runOnce("Brake Climber", () -> motor.setNeutralMode(NeutralModeValue.Brake)).ignoringDisable(true));
    }

    public static Command Deploy() {
        return LoggedCommands.print("Deploy climber", "TODO Implement climber deploy");
    }

    public static Command Intake() {
        return LoggedCommands.print("Intake cage", "TODO Implement climber cage intake");
    }

    public static Command DeployAndIntake() {
        return LoggedCommands.print("Deploy and Intake", "TODO Implement climber deploy and cage intake");
    }

    public static Command Retract() {
        return LoggedCommands.print("Retract climber", "TODO Implement climber retract");
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