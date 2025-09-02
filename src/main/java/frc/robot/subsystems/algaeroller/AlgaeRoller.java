package frc.robot.subsystems.algaeroller;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedCommands;
import frc.robot.Ports;
import frc.robot.subsystems.algaeroller.AlgaeRollerConstants.AlgaeRollerPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;

public class AlgaeRoller extends SubsystemBase {
    public static final AlgaeRoller instance = new AlgaeRoller();

    /* Devices */
    private final TalonFX deployMotor;
    private final TalonFX rollerMotor;
    
    /* Control Requests */
    private final ControlRequest deployZeroingControl = new VoltageOut(AlgaeRollerConstants.deployZeroingVoltage).withEnableFOC(true);
    private final ControlRequest intakeControl = new VoltageOut(AlgaeRollerConstants.intakeVoltage).withEnableFOC(true);
    private final ControlRequest expelControl = new VoltageOut(AlgaeRollerConstants.expelVoltage).withEnableFOC(true);
    private final ControlRequest L1AssistControl = new VoltageOut(AlgaeRollerConstants.L1AssistVoltage).withEnableFOC(true);

    private final Debouncer stallDebouncer = new Debouncer(AlgaeRollerConstants.deployStallTime.in(Units.Seconds), DebounceType.kRising);
    private boolean zeroing = false;

    boolean waitingForClear = false;
    boolean waitingForStop = false;
    boolean waitingForPivotClear = false;

    AlgaeRollerPosition currentTarget = AlgaeRollerPosition.STOWED;
    
    AlgaeRoller() {
        /* Devices */
        deployMotor = new TalonFX(Ports.ALGAE_DEPLOY.id, Ports.ALGAE_DEPLOY.bus.name);
        deployMotor.getConfigurator().apply(AlgaeRollerConstants.getDeployMotorConfig());
        rollerMotor = new TalonFX(Ports.ALGAE_ROLLERS.id, Ports.ALGAE_ROLLERS.bus.name);
        rollerMotor.getConfigurator().apply(AlgaeRollerConstants.getRollerMotorConfig());

        // Expect to begin in STOWED position and hold it
        deployMotor.setPosition(AlgaeRollerPosition.ZEROED.position);
        if (AlgaeRollerConstants.enabled) {
            startZero();
            // moveTo(currentTarget);
        } else {
            deployMotor.stopMotor();
        }

        // Debugging help
        for (AlgaeRollerPosition position : AlgaeRollerPosition.values()) {
            SmartDashboard.putData("Algae Roller/Move to " + position.name(), LoggedCommands.runOnce("Move to " + position.name(), () -> moveTo(position), this));
        }
        
        SmartDashboard.putData("Algae Roller/Zero Algae Roller", Zero());
    }

    public void startZero() {
        zeroing = true;
        stallDebouncer.calculate(false);
        DogLog.log("Algae Roller/Status", "Zeroing");
        if (AlgaeRollerConstants.enabled) {
            deployMotor.setControl(deployZeroingControl);
        }
    }

    public Command Zero() {
        return LoggedCommands.runOnce("Triggering zero of Algae Roller", this::startZero, this);
    }

    private void moveTo(AlgaeRollerPosition position) {
        DogLog.log("Algae Roller/Status", "Moving to " + position.name());
        waitingForClear = waitingForStop = false;
        currentTarget = position;
        if (AlgaeRollerConstants.enabled) {
            deployMotor.setControl(position.control);
        }
    }

    private boolean atTarget() {
        return isNear(currentTarget);
    }

    public Command WaitForTarget() {
        return LoggedCommands.waitUntil("Wait for algae bar target", this::atTarget);
    }

    private boolean isNear(AlgaeRollerPosition position) {
        if (!AlgaeRollerConstants.enabled) return true;
        return deployMotor.getPosition().getValue().minus(position.position).abs(Units.Rotations) <= AlgaeRollerConstants.epsilon.in(Units.Rotations);
    }

    public boolean isClear() {
        if (!AlgaeRollerConstants.enabled) return true;
        return deployMotor.getPosition().getValue().lte(AlgaeRollerPosition.CLEAR.position.plus(AlgaeRollerConstants.epsilon));
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
            waitingForStop = false;
            waitingForPivotClear = false;
        }
    }

    private void stowWhenStopped() {
        if (Elevator.instance.atFinalTarget()) {
            moveTo(AlgaeRollerPosition.STOWED);
        } else {
            DogLog.log("Algae Roller/Status", "Delaying stow due to elevator movement");
            waitingForStop = true;
            waitingForClear = false;
            waitingForPivotClear = false;
        }
    }

    private void stowWhenStoppedAndPivotClear() {
        waitingForStop = !Elevator.instance.atFinalTarget();
        waitingForPivotClear = !EndEffector.instance.inHighClearRange();
        waitingForClear = false;

        if (waitingForStop && waitingForPivotClear) {
            DogLog.log("Algae Roller/Status", "Delaying stow until elevator stopped and pivot clear");
            waitingForPivotClear = true;
        } else {
            moveTo(AlgaeRollerPosition.STOWED);
        }
    }

    public Command TriggerAtleastClear() {
        return LoggedCommands.runOnce("Ensure algae bar clear", this::ensureClear, this);
    }

    public Command TriggerStowWhenClear() {
        if (!AlgaeRollerConstants.enabled) return Commands.none();
        return LoggedCommands.runOnce("Stow algae roller when clear", this::stowWhenClear, this);
    }

    public Command TriggerStowWhenStopped() {
        if (!AlgaeRollerConstants.enabled) return Commands.none();
        return LoggedCommands.runOnce("Stow algae roller when stopped", this::stowWhenStopped, this);
    }

    public Command TriggerStowWhenStoppedAndPivotClear() {
        if (!AlgaeRollerConstants.enabled) return Commands.none();
        return LoggedCommands.runOnce("Stow algae roller when stopped", this::stowWhenStoppedAndPivotClear, this);
    }

    public Command TriggerStow() {
        if (!AlgaeRollerConstants.enabled) return Commands.none();
        return LoggedCommands.runOnce("Stow algae roller", () -> moveTo(AlgaeRollerPosition.STOWED), this);
    }

    public Command TriggerL1Assist() {
        if (!AlgaeRollerConstants.enabled) return Commands.none();
        return LoggedCommands.runOnce("Move algae roller to score L1", () -> moveTo(AlgaeRollerPosition.L1_SCORE), this);
    }

    public Command StartIntake() {
        if (!AlgaeRollerConstants.enabled) return Commands.none();
        return LoggedCommands.runOnce("Intake algae", () -> rollerMotor.setControl(intakeControl), this);
    }

    public Command StopIntake() {
        if (!AlgaeRollerConstants.enabled) return Commands.none();
        return LoggedCommands.runOnce("Stop algae intake", () -> rollerMotor.stopMotor(), this);
    }

    public Command StopAndClear() {
        return LoggedCommands.runOnce("Stop algae intake and move to clear", () -> {
            rollerMotor.stopMotor();
            if (AlgaeRollerConstants.enabled) {
                moveTo(AlgaeRollerPosition.CLEAR);
            }
        }, this);
    }

    public Command TriggerDeploy() {
        if (!AlgaeRollerConstants.enabled) return Commands.none();
        return LoggedCommands.runOnce("Deploy algae intake", () -> moveTo(AlgaeRollerPosition.DEPLOYED), this);
    }

    public Command Expel() {
        if (!AlgaeRollerConstants.enabled) return Commands.none();
        return LoggedCommands.runOnce("Expel algae", () -> rollerMotor.setControl(expelControl), this);
    }

    public Command GuideL1Coral() {
        if (!AlgaeRollerConstants.enabled) return Commands.none();
        return LoggedCommands.runOnce("Guide L1 coral", () -> rollerMotor.setControl(L1AssistControl), this);
    }

    @Override
    public void periodic() {
        Command currentCommand = getCurrentCommand();
        DogLog.log("Algae Roller/Current Command", currentCommand == null ? "None" : currentCommand.getName());
        DogLog.log("Algae Roller/Current target", currentTarget);
        DogLog.log("Algae Roller/Current target (rot)", currentTarget.position.in(Units.Rotations));
        DogLog.log("Algae Roller/Zeroing?", zeroing);
        DogLog.log("Algae Roller/Clear?", isClear());
        DogLog.log("Algae Roller/Waiting for clear?", waitingForClear);
        DogLog.log("Algae Roller/Waiting for stop?", waitingForStop);
        DogLog.log("Algae Roller/Waiting for pivot clear?", waitingForPivotClear);
        DogLog.log("Algae Roller/Deploy Current", deployMotor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("Algae Roller/Deploy Velocity", deployMotor.getVelocity().getValueAsDouble());
        DogLog.log("Algae Roller/Deploy Voltage", deployMotor.getMotorVoltage().getValueAsDouble());
        DogLog.log("Algae Roller/Deploy Position (rotations)", deployMotor.getPosition().getValue().in(Units.Rotations));
        DogLog.log("Algae Roller/Deploy Position (degrees)", deployMotor.getPosition().getValue().in(Units.Degrees));
        DogLog.log("Algae Roller/Intake Current", rollerMotor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("Algae Roller/Intake Velocity", rollerMotor.getVelocity().getValueAsDouble());

        // Detect deployment stalls by checking the current
        if (zeroing && DriverStation.isEnabled() && stallDebouncer.calculate(deployMotor.getVelocity().getValueAsDouble() == 0.0)) {
            DogLog.log("Algae Roller/Status", "Deploy zeroing complete");
            zeroing = false;
            if (AlgaeRollerConstants.enabled) {
                deployMotor.stopMotor();
                deployMotor.setPosition(AlgaeRollerPosition.ZEROED.position);
                deployMotor.setControl(currentTarget.control); // Return to the intended target
            }
        }

        if (waitingForClear && Elevator.instance.isClear(Elevator.ClearState.CLEAR_HIGH)) {
            if (waitingForStop || waitingForPivotClear) {
                waitingForClear = false;
            } else {
                moveTo(AlgaeRollerPosition.STOWED);
            }
        }
        if (waitingForStop && Elevator.instance.atFinalTarget()) {
            if (waitingForClear || waitingForPivotClear) {
                waitingForStop = false;
            } else {
                moveTo(AlgaeRollerPosition.STOWED);
            }
        }
        if (waitingForPivotClear && EndEffector.instance.inHighClearRange()) {
            if (waitingForClear || waitingForStop) {
                waitingForPivotClear = false;
            } else {
                moveTo(AlgaeRollerPosition.STOWED);
            }
        }
    }
}