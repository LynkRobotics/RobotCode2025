package frc.robot.subsystems.algaeroller;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedCommands;
import frc.robot.Ports;
import frc.robot.subsystems.algaeroller.AlgaeRollerContants.AlgaeRollerPosition;
import frc.robot.subsystems.elevator.Elevator;

public class AlgaeRoller extends SubsystemBase {
    public static final AlgaeRoller instance = new AlgaeRoller();

    /* Devices */
    private final TalonFX deployMotor;
    private final TalonFX rollerMotor;
    
    /* Control Requests */    
    private final ControlRequest intakeControl = new VoltageOut(AlgaeRollerContants.intakeVoltage).withEnableFOC(true);
    private final ControlRequest expelControl = new VoltageOut(AlgaeRollerContants.expelVoltage).withEnableFOC(true);
    private final ControlRequest L1AssistControl = new VoltageOut(AlgaeRollerContants.L1AssistVoltage).withEnableFOC(true);

    boolean waitingForClear = false;
    boolean waitingForStop = false;

    AlgaeRollerPosition currentTarget;
    
    AlgaeRoller() {
        /* Devices */
        deployMotor = new TalonFX(Ports.ALGAE_DEPLOY.id, Ports.ALGAE_DEPLOY.bus.name);
        deployMotor.getConfigurator().apply(AlgaeRollerContants.getDeployMotorConfig());
        rollerMotor = new TalonFX(Ports.ALGAE_ROLLERS.id, Ports.ALGAE_ROLLERS.bus.name);
        rollerMotor.getConfigurator().apply(AlgaeRollerContants.getRollerMotorConfig());

        // Expect to begin in STOWED position and hold it
        deployMotor.setPosition(AlgaeRollerPosition.STOWED.position);
        setTarget(AlgaeRollerPosition.STOWED);

        // Debugging help
        for (AlgaeRollerPosition position : AlgaeRollerPosition.values()) {
            SmartDashboard.putData("Algae Roller/Move to " + position.name(), LoggedCommands.runOnce("Move to " + position.name(), () -> moveTo(position), this));
        }
    }

    // TODO Add Zero() method / Command
    private void setTarget(AlgaeRollerPosition position) {
        DogLog.log("Algae Roller/Status", "Setting target to " + position.name());
        currentTarget = position;
        deployMotor.setControl(position.control);
    }

    private boolean atTarget() {
        return isNear(currentTarget);
    }

    public Command WaitForTarget() {
        return LoggedCommands.waitUntil("Wait for algae bar target", this::atTarget);
    }

    private boolean isNear(AlgaeRollerPosition position) {
        return deployMotor.getPosition().getValue().minus(position.position).abs(Units.Rotations) <= AlgaeRollerContants.epsilon.in(Units.Rotations);
    }

    public boolean isClear() {
        return deployMotor.getPosition().getValue().lte(AlgaeRollerPosition.CLEAR.position.plus(AlgaeRollerContants.epsilon));
    }

    private void ensureClear() {
        if (!isClear()) {
            moveTo(AlgaeRollerPosition.CLEAR);
        }
    }

    private void stowWhenClear() {
        if (Elevator.instance.isClear(Elevator.ClearState.CLEAR_HIGH)) {
            moveTo(AlgaeRollerPosition.STOWED);
        } else {
            DogLog.log("Algae Roller/Status", "Delaying stow due to elevator position");
            waitingForClear = true;
        }
    }

    private void stowWhenStopped() {
        if (Elevator.instance.atTarget()) {
            moveTo(AlgaeRollerPosition.STOWED);
        } else {
            DogLog.log("Algae Roller/Status", "Delaying stow due to elevator movement");
            waitingForStop = true;
        }
    }

    public Command TriggerAtleastClear() {
        return LoggedCommands.runOnce("Ensure algae bar clear", this::ensureClear, this);
    }

    private void moveTo(AlgaeRollerPosition position) {
        DogLog.log("Algae Roller/Status", "Moving to " + position.name());
        waitingForClear = waitingForStop = false;
        deployMotor.setControl(position.control);
    }

    public Command TriggerStowWhenClear() {
        return LoggedCommands.runOnce("Stow algae roller when clear", this::stowWhenClear, this);
    }

    public Command TriggerStowWhenStopped() {
        return LoggedCommands.runOnce("Stow algae roller when stopped", this::stowWhenStopped, this);
    }

    public Command TriggerStow() {
        return LoggedCommands.runOnce("Stow algae roller", () -> moveTo(AlgaeRollerPosition.STOWED), this);
    }

    public Command Intake() {
        return LoggedCommands.runOnce("Intake algae", () -> rollerMotor.setControl(intakeControl), this);
    }

    public Command Expel() {
        return LoggedCommands.runOnce("Expel algae", () -> rollerMotor.setControl(expelControl), this);
    }

    public Command GuideL1Coral() {
        return LoggedCommands.runOnce("Guide L1 coral", () -> rollerMotor.setControl(L1AssistControl), this);
    }

    @Override
    public void periodic() {
        Command currentCommand = getCurrentCommand();
        DogLog.log("Algae Roller/Current Command", currentCommand == null ? "None" : currentCommand.getName());
        DogLog.log("Algae Roller/Clear?", isClear());
        DogLog.log("Algae Roller/Waiting for clear?", waitingForClear);
        DogLog.log("Algae Roller/Waiting for stop?", waitingForStop);
        DogLog.log("Algae Roller/Deploy Current", deployMotor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("Algae Roller/Deploy Velocity", deployMotor.getVelocity().getValueAsDouble());
        DogLog.log("Algae Roller/Deploy Position (rotations)", deployMotor.getPosition().getValue().in(Units.Rotations));
        DogLog.log("Algae Roller/Deploy Position (degress)", deployMotor.getPosition().getValue().in(Units.Degrees));
        DogLog.log("Algae Roller/Intake Current", rollerMotor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("Algae Roller/Intake Velocity", rollerMotor.getVelocity().getValueAsDouble());

        if (waitingForClear && Elevator.instance.isClear(Elevator.ClearState.CLEAR_HIGH)) {
            moveTo(AlgaeRollerPosition.STOWED);
        } else if (waitingForStop && Elevator.instance.atTarget()) {
            moveTo(AlgaeRollerPosition.STOWED);
        }
    }
}