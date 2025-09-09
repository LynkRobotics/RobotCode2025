package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedCommands;
import frc.robot.Ports;
import frc.robot.subsystems.algaeroller.AlgaeRoller;
import frc.robot.subsystems.elevator.ElevatorConstants.Stop;
import frc.robot.subsystems.endeffector.EndEffector;

public class Elevator extends SubsystemBase {
    public static final Elevator instance = new Elevator();

    public enum ClearState {
        NOT_CLEAR,
        CLEAR_LOW,
        CLEAR_HIGH;
    }
    
    private final TalonFX mainMotor;
    private final TalonFX followerMotor;
    private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);

    private Debouncer stallDebouncer = new Debouncer(ElevatorConstants.stallTimeout.in(Units.Seconds), Debouncer.DebounceType.kRising);
    private boolean stalled = false;
    private int stallCount = 0;
    private double lastPosition = 0.0;
    
    private ClearState clearState = ClearState.NOT_CLEAR;

    private Stop finalTarget = Stop.STOW;
    private Stop currentTarget = finalTarget;
    private boolean waitingOnAlgaeBarClear = false;

    private boolean speedLimited = false;

    public Elevator() {
        mainMotor = new TalonFX(Ports.ELEVATOR_MAIN.id, Ports.ELEVATOR_MAIN.bus.name);
        followerMotor = new TalonFX(Ports.ELEVATOR_FOLLOWER.id, Ports.ELEVATOR_FOLLOWER.bus.name);

        mainMotor.getConfigurator().apply(ElevatorConstants.getMotorConfig());
        mainMotor.stopMotor();
        followerMotor.setControl(new Follower(Ports.ELEVATOR_MAIN.id, false));

        SmartDashboard.putData("Elevator/Raise", Raise());
        SmartDashboard.putData("Elevator/Lower", Lower());
        SmartDashboard.putData("Elevator/Stop", Stop());
        SmartDashboard.putNumber("Elevator/Direct Voltage", ElevatorConstants.slowVoltage);
        SmartDashboard.putData("Elevator/Set Voltage", LoggedCommands.runOnce("Set Voltage", () -> { setVoltage(SmartDashboard.getNumber("Elevator/Direct Voltage", 0.0));}));
        SmartDashboard.putData("Elevator/Zero", Zero());
        SmartDashboard.putData("Elevator/SetZero", SetZero());

        SmartDashboard.putData("Elevator/Move to L1", LoggedCommands.runOnce("Move to L1", () -> moveTo(Stop.L1), this));
        SmartDashboard.putData("Elevator/Move to L2", LoggedCommands.runOnce("Move to L2", () -> moveTo(Stop.L2), this));
        SmartDashboard.putData("Elevator/Move to L3", LoggedCommands.runOnce("Move to L3", () -> moveTo(Stop.L3), this));
        SmartDashboard.putData("Elevator/Move to L4", LoggedCommands.runOnce("Move to L4", () -> moveTo(Stop.L4), this));

        setAsZero();
        moveTo(Stop.STOW);
    }

    public void setAsZero() {
        DogLog.log("Elevator/Status", "Set as Zero");
        mainMotor.setPosition(0);
        followerMotor.setPosition(0);
    }

    public Command SetZero() {
        return LoggedCommands.runOnce("Set Elevator Zero", this::setAsZero).ignoringDisable(true);
    }

    public Command Zero() {
        return LoggedCommands.sequence("Zero Elevator",
            Commands.deadline(
                LoggedCommands.waitUntil("Wait for stall", this::isStalled),
                Lower()),
            SetZero());
    }

    public Command Raise() {
        return LoggedCommands.runOnce("Raise Elevator", () -> setVoltage(ElevatorConstants.slowVoltage), this);
    }

    public Command Lower() {
        return LoggedCommands.runOnce("Lower Elevator", () -> setVoltage(-ElevatorConstants.slowVoltage), this);
    }

    public Command Stop() {
        return LoggedCommands.runOnce("Stop Elevator", this::stop, this);
    }

    private void stop() {
        DogLog.log("Elevator/Status", "Stopped");
        mainMotor.stopMotor();
    }

    private void setVoltage(double voltage) {
        DogLog.log("Elevator/Status", "Set voltage " + String.format("%1.2f", voltage));
        mainMotor.setControl(voltageOut.withOutput(voltage));
    }

    public Command TriggerMoveTo(Stop stop) {
        return LoggedCommands.runOnce("Move elevator to " + stop, () -> moveTo(stop), this);
    }

    public Command TriggerMoveToDirect(Stop stop) {
        return LoggedCommands.runOnce("Move elevator direcly to " + stop, () -> moveTo(stop, true), this);
    }

    private boolean isStalled() {
        return stalled;
    }

    public boolean atTarget() {
        return mainMotor.getPosition().getValue().minus(currentTarget.position).abs(Units.Rotations) <= ElevatorConstants.positionError;
    }

    public boolean atFinalTarget() {
        return mainMotor.getPosition().getValue().minus(finalTarget.position).abs(Units.Rotations) <= ElevatorConstants.positionError;
    }

    private double stopError(Stop stop) {
        return Math.abs(stop.height.minus(getHeight()).magnitude());
    }

    public boolean atStop(Stop stop) {
        return stopError(stop) <= ElevatorConstants.positionError;
    }

    private Distance getHeight(double position) {
        return Units.Inches.of(position / ElevatorConstants.rotPerInch);
    }

    private Distance getHeight() {
        return getHeight(mainMotor.getPosition().getValueAsDouble());
    }

    public boolean nearOrAbove(Stop stop) {
        // In theory -- but these have an old implementation
        // return nearStop(stop) || aboveStop(stop);
        return getHeight().gte(stop.height.minus(ElevatorConstants.epsilonThreshold));
    }

    public double raisedPercentage() {
        return MathUtil.clamp(getHeight().minus(Stop.CORAL_HOLD.height).div(Stop.BARGE.height).magnitude(), 0.0, 1.0);
    }
    
    public boolean isClear(ClearState desiredState) {
        if (desiredState == ClearState.CLEAR_HIGH && clearState == ClearState.CLEAR_HIGH) {
            return true;
        } else if (desiredState == ClearState.CLEAR_LOW && (clearState == ClearState.CLEAR_LOW || clearState == ClearState.CLEAR_HIGH)) {
            return true;
        }
        return false;
    }

    public boolean shouldLimitSpeed() {
        return speedLimited;
    }

    private void setCurrentTarget(Stop target, boolean requireAlgaeBarClear) {
        if (requireAlgaeBarClear && !AlgaeRoller.instance.isClear()) {
            // Set the target but not the set position (the current set position must be safe)
            waitingOnAlgaeBarClear = true;
            DogLog.log("Elevator/Status", "New target (once clear): " + target + " <- " + currentTarget);
        } else {
            // Either we don't need the algae bar to be clear, or it is clear
            waitingOnAlgaeBarClear = false;
            mainMotor.setControl(target.control);
            DogLog.log("Elevator/Status", "New target: " + target + " <- " + currentTarget);
        }
        currentTarget = target;
    }

    public void moveTo(Stop target) {
        moveTo(target, false);
    }

    public void moveTo(Stop target, boolean direct) {
        DogLog.log("Elevator/Status", "Final target: " + target + " <- " + finalTarget);
        finalTarget = target;

        Angle currentPosition = mainMotor.getPosition().getValue();
        Angle targetPosition = target.position;
        boolean goingUp = currentPosition.lt(targetPosition);

        if (direct) {
            // We are explicitly asking to move directly to a position
            setCurrentTarget(target, false);
        } else if (goingUp) {
            // We need to wait for the algae bar to be clear if we are below the CLEAR_HIGH mark
            setCurrentTarget(target, currentPosition.lt(Stop.CLEAR_HIGH.position));
        } else if (currentPosition.gt(Stop.CLEAR_HIGH.position)) {
            // We are moving down, towards the CLEAR_HIGH mark, so ensure that
            // we don't go past until we are sure we are clear (checked in periodic())
            setCurrentTarget(Stop.CLEAR_HIGH, false);
        } else if (currentPosition.gt(Stop.CLEAR_LOW.position)) {
            // We are moving down, towards the CLEAR_LOW mark, so ensure that
            // we don't go past until we are sure we are clear (checked in periodic())
            setCurrentTarget(Stop.CLEAR_LOW, true);
        } else {
            // Check if the End Effector needs to pivot
            if (EndEffector.instance.waitingToPivot()) {
                // If the End Effector is waiting to pivot, we need to move to CLEAR_LOW
                // to enable it to pivot before moving to our final target
                setCurrentTarget(Stop.CLEAR_LOW, true);
            } else {
                // The End Effector doesn't need to wait for the elevator, so we can move to our target
                setCurrentTarget(target, true);
            }
        }
    }

    @Override
    public void periodic() {
        Command currentCommand = getCurrentCommand();
        DogLog.log("Elevator/Current Command", currentCommand == null ? "None" : currentCommand.getName());

        double position = mainMotor.getPosition().getValueAsDouble();
        Distance height = getHeight(position);
        double followPosition = followerMotor.getPosition().getValueAsDouble();
        double voltage = mainMotor.getMotorVoltage().getValueAsDouble();

        // Determine clear state of Elevator
        if (height.gt(Stop.CLEAR_HIGH.height.minus(ElevatorConstants.epsilonThreshold))) {
            clearState = ClearState.CLEAR_HIGH;
        } else if (height.gt(Stop.CLEAR_LOW.height.minus(ElevatorConstants.epsilonThreshold))) {
            clearState = ClearState.CLEAR_LOW;
        } else {
            clearState = ClearState.NOT_CLEAR;
        }

        // Check if the speed should be limited
        speedLimited = height.gt(Stop.SLOW_DOWN.height);

        // Check if we aren't moving towards the current target because we need to wait for the algae bar to be clear
        if (waitingOnAlgaeBarClear && AlgaeRoller.instance.isClear()) {
            // We are unblocked, and now can move towards the intended target
            waitingOnAlgaeBarClear = false;
            mainMotor.setControl(currentTarget.control);
            DogLog.log("Elevator/Status", "Unblocked for target: " + currentTarget);
        }

        // Check if we are moving towards an interim target due to requirements that must
        // be met before moving to the final target
        if (finalTarget != currentTarget) {
            // TODO Handle the move up-and-down for withing CLEAR_LOW range
            if (currentTarget == Stop.CLEAR_HIGH) {
                // If we are headed for the CLEAR_HIGH stop, it's because we need to ensure that the algae roller is clear
                // prior to dropping below the CLEAR_HIGH stop
                if (AlgaeRoller.instance.isClear()) {
                    // Potentially stop at CLEAR_LOW to ensure that the end effector completes pivoting first
                    // Note that we don't need to check the Algae Roller because we checked it already
                    if (finalTarget.height.lt(Stop.CLEAR_LOW.height)) {
                        setCurrentTarget(Stop.CLEAR_LOW, false);
                    } else {
                        currentTarget = finalTarget;
                        setCurrentTarget(finalTarget, false);
                    }
                } // We'll hold at CLEAR_HIGH until the algae bar is clear
            }
            
            // If we are headed for the CLEAR_LOW stop, it's because we need to ensure that the pivot completes movement
            // prior to dropping below the CLEAR_LOW stop
            if (currentTarget == Stop.CLEAR_LOW) {
                if (EndEffector.instance.inPosition()) {
                    // We don't need to check the Algae Roller because we must have passed requirements
                    // already since we were already targeting CLEAR_LOW, which is below CLEAR_HIGH
                    setCurrentTarget(finalTarget, false);
                }
            }
        }
        DogLog.log("Elevator/Final Target", finalTarget.name());
        DogLog.log("Elevator/Current Target", currentTarget.name());
        DogLog.log("Elevator/Current Target (rot)", currentTarget.position.in(Units.Rotations));

        // Handle exceptions in cases other than elevator at rest
        stalled = stallDebouncer.calculate(voltage != 0.0 && position == lastPosition && !waitingOnAlgaeBarClear && !atTarget());
        if (stalled) {
            if (currentTarget.height.baseUnitMagnitude() == 0.0 && height.lt(ElevatorConstants.autoZeroHeight)) {
                DogLog.log("Elevator/Status", "Auto-zeroing due to stall");
                setAsZero();
            } else {
                DogLog.log("Elevator/Status", "Stall detected");                
            }
            stallCount++;
            DogLog.log("Elevator/Stall count", stallCount);                
            stop();
        }
        lastPosition = position;

        // double followDifference = position - followPosition;
        // if (Math.abs(followDifference) >= positionDiffMax) {
            // This seems to be a semi-normal experience, perhaps due to latency in reporting motor position at faster speeds
            // The motors are mechanically connected, so it really should be impossible to actually be out of sync
            // LoggedAlert.Warning("Elevator", "Elevator Unequal", "Elevator motor position difference of " + String.format("%01.2f", followDifference) + " exceeds limit");
            // stop();
        // }

        DogLog.log("Elevator/height", height.in(Units.Inches));
        DogLog.log("Elevator/mainPosition", position);
        DogLog.log("Elevator/mainPosition (degrees)", mainMotor.getPosition().getValue().in(Units.Degrees));
        DogLog.log("Elevator/mainVelocity", mainMotor.getVelocity().getValueAsDouble());
        DogLog.log("Elevator/mainVoltage", voltage);
        DogLog.log("Elevator/followPosition", followPosition);
        DogLog.log("Elevator/followVelocity", followerMotor.getVelocity().getValueAsDouble());
        DogLog.log("Elevator/followVoltage", followerMotor.getMotorVoltage().getValueAsDouble());
        DogLog.log("Elevator/clearState", clearState.name());
        DogLog.log("Elevator/atTarget", atTarget());
        DogLog.log("Elevator/stalled", isStalled());

        SmartDashboard.putNumber("Elevator/Stall Count", stallCount);
        SmartDashboard.putString("Elevator/Target", currentTarget.name());
        SmartDashboard.putBoolean("Elevator/At Final Target", atFinalTarget());
    }
}