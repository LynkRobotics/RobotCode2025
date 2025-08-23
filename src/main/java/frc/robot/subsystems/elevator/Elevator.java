package frc.robot.subsystems.elevator;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedAlert;
import frc.lib.util.LoggedCommands;
import frc.robot.Ports;
import frc.robot.subsystems.algaeroller.AlgaeRoller;
import frc.robot.subsystems.elevator.ElevatorConstants.Stop;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.pose.PoseConstants;
import frc.robot.subsystems.pose.Pose;

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
    private final MotionMagicExpoVoltage positionControl = new MotionMagicExpoVoltage(0.0).withEnableFOC(true);

    private Stop nextStop = Stop.STOW;

    private Debouncer stallDebouncer = new Debouncer(ElevatorConstants.stallTimeout.in(Units.Seconds), Debouncer.DebounceType.kRising);
    private boolean stalled = false;
    private int stallCount = 0;
    private double lastPosition = 0.0;
    private boolean autoUp = false;
    
    private ClearState clearState = ClearState.NOT_CLEAR;

    private Stop finalTarget = Stop.STOW;
    private Stop currentTarget = finalTarget;
    private boolean waitingOnAlgaeBarClear = false;

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
        SmartDashboard.putNumber("Elevator/Direct Position", 0.0);
        SmartDashboard.putData("Elevator/Set Position", LoggedCommands.runOnce("Set Position", () -> { setPosition(SmartDashboard.getNumber("Elevator/Direct Position", 0.0));}));
        SmartDashboard.putNumber("Elevator/Direct Height", 0.0);
        SmartDashboard.putData("Elevator/Set Height", LoggedCommands.runOnce("Set Height", () -> { setHeight(Units.Inches.of(SmartDashboard.getNumber("Elevator/Direct Height", 12.0)));}));
        SmartDashboard.putData("Elevator/Zero", Zero());
        SmartDashboard.putData("Elevator/SetZero", SetZero());
        // SmartDashboard.putData("Elevator/FastZero", FastZero());

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

    public Command GoToNext() {
        return LoggedCommands.sequence("Move Elevator to stop",
            LoggedCommands.log(() -> "Next stop: " + nextStop),
            Commands.runOnce(() -> {
                // RobotState.updateActiveStop(nextStop);
                setHeight(nextStop.height);
            }, this),
            LoggedCommands.idle("Idle to hold elevator", this));
    }

    public void setNextStop(Stop stop) {
        DogLog.log("Elevator/Status", "Next stop = " + stop);
        // RobotState.updateNextStop(stop);
        nextStop = stop;
    }

    public Command AutoElevatorUp(Translation2d target) {
        return AutoElevatorUp(target, () -> nextStop).withName("Auto Elevator Up to Next");
    }

    public Command AutoElevatorUp(Translation2d target, Stop stop) {
        return AutoElevatorUp(target, () -> stop).withName("Auto Elevator Up to " + stop);
    };

    public Command AutoElevatorUp(Translation2d target, Supplier<Stop> stopSupplier) {
        return LoggedCommands.startRun("Auto Elevator Up",
            () -> autoUp = false,
            () -> {
                // TODO Always flip?
                if (!autoUp && Pose.distanceTo(Pose.flipIfRed(target)) <= PoseConstants.autoUpDistance.in(Units.Meter)) {
                    Stop stop = stopSupplier.get();
                    // RobotState.updateActiveStop(stop);
                    setHeight(stop.height);
                    autoUp = true;
                }
            },
            this);
    };

    public Command WaitForStop(Stop stop) {
        return LoggedCommands.waitUntil("Wait for Elevator to reach " + stop, () -> atStop(stop));
    }

    public Command WaitForNext() {
        return LoggedCommands.waitUntil("Wait for Elevator to reach next stop", () -> atNextStop());
    }

    public Command WaitForNearNext() {
        return LoggedCommands.waitUntil("Wait for Elevator to near next stop", () -> nearNextStop());
    }

    // Very particular use cases
    public Command WaitForTowardsNext() {
        return LoggedCommands.waitUntil("Wait for Elevator towards next stop", () -> towardsNextStop());
    }

    private boolean isStalled() {
        return stalled;
    }

    // Set Elevator height to given position
    private void setHeight(Distance height) {
        if (height.gt(ElevatorConstants.maxHeight)) {
            LoggedAlert.Warning("Elevator", "Elevator Range", "Requested elevator height too high");
            height = ElevatorConstants.maxHeight;
        }
        if (height.gt(ElevatorConstants.baseHeight)) {
            LoggedAlert.Warning("Elevator", "Elevator Range", "Requested elevator height too low");
            height = ElevatorConstants.baseHeight;
        }
        DogLog.log("Elevator/Status", "Move to height " + String.format("%1.1f", height));

        double position = height.minus(ElevatorConstants.baseHeight).in(Units.Inches) * ElevatorConstants.rotPerInch;
        setPosition(position);
    }

    private void setPosition(double position) {
        DogLog.log("Elevator/Status", "Move to position " + String.format("%1.2f", position));
        DogLog.log("Elevator/Set Position", position);
        mainMotor.setControl(positionControl.withPosition(position));
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

    public boolean nearStop(Stop stop) {
        return stopError(stop) <= ElevatorConstants.positionCloseError;
    }

    public boolean aboveStop(Stop stop) {
        return getHeight().gt(stop.height);
    }

    // Are potentially towards a stop? (limited use cases)
    public boolean towardsStop(Stop stop) {
        return getHeight().plus(ElevatorConstants.towardsMargin).gte(stop.height);
    }

    public boolean atNextStop() {
        return atStop(nextStop);
    }

    public boolean nearNextStop() {
        return nearStop(nextStop);
    }

    public boolean towardsNextStop() {
        return towardsStop(nextStop);
    }

    private Distance getHeight(double position) {
        // TODO Reevaluate baseHeight, not used by 1678
        return Units.Inches.of(position / ElevatorConstants.rotPerInch); //.plus(ElevatorConstants.baseHeight);
    }

    private Distance getHeight() {
        return getHeight(mainMotor.getPosition().getValueAsDouble());
    }

    public double raisedPercentage() {
        return MathUtil.clamp(getHeight().minus(Stop.CORAL_HOLD.height).div(Stop.L4.height).magnitude(), 0.0, 1.0);
    }
    
    public boolean isClear(ClearState desiredState) {
        if (desiredState == ClearState.CLEAR_HIGH && clearState == ClearState.CLEAR_HIGH) {
            return true;
        } else if (desiredState == ClearState.CLEAR_LOW && (clearState == ClearState.CLEAR_LOW || clearState == ClearState.CLEAR_HIGH)) {
            return true;
        }
        return false;
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
        } else if(goingUp) {
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
            if (!EndEffector.instance.inPosition()) {
                // If the End Effector is waiting to pivot, we need to move to CLEAR_LOW
                // to enable it to pivot before moving to our final target
                setCurrentTarget(Stop.CLEAR_LOW, true);
            } else {
                // The End Effector is already in position, so we can move to our target
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
        // double followDifference = position - followPosition;
        double voltage = mainMotor.getMotorVoltage().getValueAsDouble();

        // Determine clear state of Elevator
        if (height.gt(Stop.CLEAR_HIGH.height.minus(ElevatorConstants.epsilonThreshold))) {
            clearState = ClearState.CLEAR_HIGH;
        } else if (height.gt(Stop.CLEAR_LOW.height.minus(ElevatorConstants.epsilonThreshold))) {
            clearState = ClearState.CLEAR_LOW;
        } else {
            clearState = ClearState.NOT_CLEAR;
        }

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
        DogLog.log("Elevator/followoltage", followerMotor.getMotorVoltage().getValueAsDouble());
        DogLog.log("Elevator/clearState", clearState.name());
        DogLog.log("Elevator/atTarget", atTarget());
        DogLog.log("Elevator/stalled", isStalled());

        SmartDashboard.putBoolean("Elevator/At Target", atTarget());
        SmartDashboard.putBoolean("Elevator/L1", atStop(Stop.L1));
        SmartDashboard.putBoolean("Elevator/L2", atStop(Stop.L2));
        SmartDashboard.putBoolean("Elevator/L3", atStop(Stop.L3));
        SmartDashboard.putBoolean("Elevator/L4", atStop(Stop.L4));
        SmartDashboard.putString("Elevator/Next Stop", nextStop.toString());
    }
}