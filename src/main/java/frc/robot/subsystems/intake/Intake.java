// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedCommands;
import frc.robot.Ports;
import frc.robot.subsystems.intake.IntakeConstants.IntakePosition;

public class Intake extends SubsystemBase {
    public static final Intake instance = new Intake();
    
    private static final Timer expelTimer = new Timer();

    private static final Debouncer stallDebouncer = new Debouncer(IntakeConstants.deployStallTime.in(Units.Seconds), DebounceType.kRising);
    private static boolean zeroing = false;

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
        SmartDashboard.putData("Intake/Move to DEPLOYED",
            LoggedCommands.runOnce("Move Intake to DEPLOYED", () -> { deployMotor.setControl(IntakePosition.DEPLOYED.control); }, this));
        SmartDashboard.putData("Intake/Move to RETRACTED",
            LoggedCommands.runOnce("Move Intake to RETRACTED", () -> { deployMotor.setControl(IntakePosition.RETRACTED.control); }, this));
        SmartDashboard.putData("Intake/Move to FULL_STOW",
            LoggedCommands.runOnce("Move Intake to FULL_STOW", () -> { deployMotor.setControl(IntakePosition.FULL_STOW.control); }, this));

        // We *should* be fully stowed, but given all the testing we do, also zero to start
        deployMotor.setPosition(desiredState.position);
        startZero();

        // We could start by holding the state *if* we didn't start by zeroing
        // deployMotor.setControl(desiredState.control);
    }

    private void startZero() {
        zeroing = true;
        deployMotor.setControl(deployZeroingControl);
    }

    private void runDeploy() {
        DogLog.log("Intake/Status", "Deploying Intake");
        expelTimer.stop();
        deployMotor.setControl(IntakePosition.DEPLOYED.control);
        intakeMotor.setControl(intakeControl);
        indexMotor.setControl(indexControl);
    }

    private void runExpel() {
        DogLog.log("Intake/Status", "Expelling Intake");
        deployMotor.setControl(IntakePosition.DEPLOYED.control);
        intakeMotor.setControl(intakeExpelControl);
        indexMotor.setControl(indexExpelControl);
        expelTimer.restart();
    }

    private void stopIntake() {
        DogLog.log("Intake/Status", "Stopping Intake");
        intakeMotor.stopMotor();
        indexMotor.stopMotor();
        deployMotor.setControl(IntakePosition.RETRACTED.control);
    }

    public Command Deploy() {
        return LoggedCommands.runOnce("Deploy Intake", this::runDeploy, this);
    }

    public Command Expel() {
        return LoggedCommands.runOnce("Expel from Intake", this::runExpel, this);
    }

    public Command Stop() {
        return LoggedCommands.runOnce("Stop Intake", this::stopIntake, this);
    }

    // Gently deploy intake until it stalls to recalibrate the zero position
    public Command ZeroIntake() {
        return LoggedCommands.runOnce("Triggering zero of Coral Intake", this::startZero, this);
    }

    @Override
    public void periodic() {
        Command currentCommand = getCurrentCommand();
        DogLog.log("Intake/Current Command", currentCommand == null ? "None" : currentCommand.getName());
        DogLog.log("Intake/Desired State", desiredState.name());
        DogLog.log("Intake/At Desired State", atDesiredState);
        DogLog.log("Intake/Deploy Current", deployMotor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("Intake/Deploy Velocity", deployMotor.getVelocity().getValueAsDouble());
        DogLog.log("Intake/Deploy Position (Rotations)", deployMotor.getPosition().getValue().in(Units.Rotations));
        DogLog.log("Intake/Deploy Position (Degrees)", deployMotor.getPosition().getValue().in(Units.Degrees));
        DogLog.log("Intake/Intake Current", intakeMotor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("Intake/Intake Velocity", intakeMotor.getVelocity().getValueAsDouble());
        DogLog.log("Intake/Index Current", indexMotor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("Intake/Index Velocity", indexMotor.getVelocity().getValueAsDouble());

        // Detect intake deployment stalls by checking the current
        if (stallDebouncer.calculate(deployMotor.getTorqueCurrent().getValue().gt(IntakeConstants.deployStallCurrent))) {
            if (zeroing) {
                DogLog.log("Intake/Status", "Intake deploy zeroing complete");
                zeroing = false;
                deployMotor.stopMotor();
                deployMotor.setPosition(IntakePosition.DEPLOYED.position);
                deployMotor.setControl(desiredState.control); // Return to the desiredState
            } else {
                DogLog.log("Intake/Status", "Intake deploy stall detected");
                deployMotor.stopMotor();
            }
        }

        // If the expel timer has elapsed, end expel and reset deploy position
        if (expelTimer.isRunning() && expelTimer.hasElapsed(IntakeConstants.expelTime.in(Units.Seconds))) {
            expelTimer.stop();
            stopIntake();
        }
    }
}