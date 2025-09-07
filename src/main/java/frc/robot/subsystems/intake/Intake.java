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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedCommands;
import frc.robot.Ports;
import frc.robot.subsystems.intake.IntakeConstants.IntakePosition;

public class Intake extends SubsystemBase {
    public static final Intake instance = new Intake();
    
    private static final Timer expelTimer = new Timer();

    private static final Debouncer stallDebouncer = new Debouncer(IntakeConstants.deployStallTime.in(Units.Seconds), DebounceType.kRising);
    private boolean zeroing = false;

    private IntakePosition desiredState = IntakePosition.RETRACTED;
    private boolean atDesiredState = true;
    private boolean intaking = false;

    /* Devices */
    private final TalonFX deployMotor;
    private final TalonFX intakeMotor;
    private final TalonFX indexMotor;

    /* Control Requests */
    private final VoltageOut deployZeroingControl = new VoltageOut(IntakeConstants.deployZeroingVoltage).withEnableFOC(true);
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
        for (IntakePosition position : IntakePosition.values()) {
            SmartDashboard.putData("Intake/Move to " + position,
                LoggedCommands.runOnce("Move Intake to " + position, () -> { deployMotor.setControl(position.control); }, this));
        }

        // We *should* be fully stowed, but given all the testing we do, also zero to start
        deployMotor.setPosition(IntakePosition.FULL_STOW.position);
        startZero();
    }

    private void startZero() {
        zeroing = true;
        deployMotor.setControl(deployZeroingControl);
    }

    private void setDeploy(IntakePosition state) {
        desiredState = state;
        deployMotor.setControl(desiredState.control);
    }

    public boolean intaking() {
        return intaking;
    }

    private void runDeploy() {
        DogLog.log("Intake/Status", "Deploying Intake");
        expelTimer.stop();
        setDeploy(IntakePosition.DEPLOYED);
        intakeMotor.setControl(intakeControl);
        intaking = true;
        indexMotor.setControl(indexControl);
    }

    private void runExpel() {
        DogLog.log("Intake/Status", "Expelling Intake");
        setDeploy(IntakePosition.DEPLOYED);
        intakeMotor.setControl(intakeExpelControl);
        intaking = false;
        indexMotor.setControl(indexExpelControl);
        expelTimer.restart();
    }

    private void runExpelForever() {
        DogLog.log("Intake/Status", "Expelling Intake until stopped");
        setDeploy(IntakePosition.DEPLOYED);
        intakeMotor.setControl(intakeExpelControl);
        intaking = false;
        indexMotor.setControl(indexExpelControl);
    }

    private void stopIntake() {
        DogLog.log("Intake/Status", "Stopping Intake");
        intakeMotor.stopMotor();
        intaking = false;
        indexMotor.stopMotor();
        setDeploy(IntakePosition.RETRACTED);
    }

    private void fullStow() {
        DogLog.log("Intake/Status", "Fully stowing Intake");
        intakeMotor.stopMotor();
        intaking = false;
        indexMotor.stopMotor();
        setDeploy(IntakePosition.FULL_STOW);
    }

    public Command Deploy() {
        return LoggedCommands.runOnce("Deploy Intake", this::runDeploy, this);
    }

    public Command Jog() {
        return LoggedCommands.sequence("Jog Intake",
            LoggedCommands.runOnce("Jog intake up a bit", () -> { deployMotor.setControl(IntakePosition.JOGGED.control); }, this),
            Commands.waitSeconds(0.4),
            Deploy());
    }

    public Command Expel() {
        return LoggedCommands.runOnce("Expel from Intake", this::runExpel, this);
    }

    public Command ExpelForever() {
        return LoggedCommands.sequence("Expel from Intake",
            Commands.runOnce(this::runExpelForever, this),
            Commands.idle());
    }

    public Command Stop() {
        return LoggedCommands.runOnce("Stop Intake", this::stopIntake, this);
    }

    public Command FullStow() {
        return LoggedCommands.runOnce("Fully stow Intake", this::fullStow, this);
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
        DogLog.log("Intake/Zeroing?", zeroing);
        DogLog.log("Intake/Intaking?", intaking);
        DogLog.log("Intake/Deploy Current", deployMotor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("Intake/Deploy Voltage", deployMotor.getMotorVoltage().getValueAsDouble());
        DogLog.log("Intake/Deploy Velocity", deployMotor.getVelocity().getValueAsDouble());
        DogLog.log("Intake/Deploy Position (Rotations)", deployMotor.getPosition().getValue().in(Units.Rotations));
        DogLog.log("Intake/Deploy Position (Degrees)", deployMotor.getPosition().getValue().in(Units.Degrees));
        DogLog.log("Intake/Intake Current", intakeMotor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("Intake/Intake Velocity", intakeMotor.getVelocity().getValueAsDouble());
        DogLog.log("Intake/Index Current", indexMotor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("Intake/Index Velocity", indexMotor.getVelocity().getValueAsDouble());

        // Detect intake deployment stalls by checking the current
        if (stallDebouncer.calculate(deployMotor.getTorqueCurrent().getValue().lt(IntakeConstants.deployStallCurrent))) {
            if (zeroing) {
                DogLog.log("Intake/Status", "Intake deploy zeroing complete");
                zeroing = false;
                deployMotor.stopMotor();
                deployMotor.setPosition(IntakePosition.FULLY_DEPLOYED.position);
                deployMotor.setControl(desiredState.control); // Return to the desiredState
            } else {
                DogLog.log("Intake/Status", "Intake deploy stall detected");
                deployMotor.stopMotor();
            }
        }

        // If the expel timer has elapsed, end expel and reset deploy position
        if (expelTimer.isRunning()) { // Remove for now: && expelTimer.hasElapsed(IntakeConstants.expelTime.in(Units.Seconds))) {
            expelTimer.stop();
            stopIntake();
        }
    }
}