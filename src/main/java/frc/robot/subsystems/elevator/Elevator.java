package frc.robot.subsystems.elevator;

import static frc.robot.Options.optServiceMode;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.subsystems.robotstate.RobotState;

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
    private final PositionVoltage positionVoltage = new PositionVoltage(0.0).withEnableFOC(true);
    private final MotionMagicExpoVoltage positionControl = new MotionMagicExpoVoltage(0.0).withEnableFOC(true);
    // private final MechanismLigament2d mechanism;

    private Stop nextStop = Stop.SAFE;

    private int stallCount = 0;
    private final int stallMax = 3;
    private double lastPosition = 0.0;
    private boolean zeroing = false;
    private boolean autoUp = false;
    private Timer scoreTimer = new Timer();
    
    private final double positionDiffMax = 0.5;

    private ClearState clearState = ClearState.NOT_CLEAR;

    private Stop finalTarget = Stop.STOW;
    private Stop currentTarget = finalTarget;

    public Elevator() {
        mainMotor = new TalonFX(Ports.ELEVATOR_MAIN.id, Ports.ELEVATOR_MAIN.bus.name);
        followerMotor = new TalonFX(Ports.ELEVATOR_FOLLOWER.id, Ports.ELEVATOR_FOLLOWER.bus.name);

        mainMotor.getConfigurator().apply(ElevatorConstants.getMotorConfig());
        mainMotor.stopMotor();
        followerMotor.setControl(new Follower(Ports.ELEVATOR_MAIN.id, true));

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
        SmartDashboard.putData("Elevator/FastZero", FastZero());

        // TODO
        // initDefaultCommand();
    }

    public void setAsZero() {
        DogLog.log("Elevator/Status", "Set as Zero");
        // RobotState.setElevatorAtZero(true);
        mainMotor.setPosition(0);
        followerMotor.setPosition(0);
    }

    public Command SetZero() {
        return LoggedCommands.runOnce("Set Elevator Zero", 
            () -> {
                setAsZero();
            }).ignoringDisable(true);
    }

    public Command Zero() {
        return IfNotBlocked(LoggedCommands.sequence("Zero Elevator",
            Commands.runOnce(() -> zeroing = true),
            Commands.deadline(
                LoggedCommands.waitUntil("Wait for stall", this::isStalled),
                Lower()).handleInterrupt(() -> zeroing = false),
            SetZero()));
    }

    public Command FastZero() {
        return IfNotBlocked(LoggedCommands.sequence("Fast Zero",
            Commands.deadline(
                LoggedCommands.waitUntil("Wait for elevator in safe zone", this::isSafe),
                Move(Stop.SAFE)),
            Zero()));
    }

    public Command Raise() {
        return LoggedCommands.runOnce("Raise Elevator", 
            () -> {
                setVoltage(ElevatorConstants.slowVoltage);
            },
            this);
    }

    public Command Lower() {
        return LoggedCommands.runOnce("Lower Elevator",
        () -> {
            setVoltage(-ElevatorConstants.slowVoltage);
        },
        this);
    }

    public Command Stop() {
        return LoggedCommands.runOnce("Stop Elevator", this::stop, this);
    }

    private void stop() {
        DogLog.log("Elevator/Status", "Stopped");
        mainMotor.stopMotor();
    }

    private void setVoltage(double voltage) {
        stallCount = 0;
        // TODO Handle blocking
        DogLog.log("Elevator/Status", "Set voltage " + String.format("%1.2f", voltage));
        // RobotState.setElevatorAtZero(false);
        mainMotor.setControl(voltageOut.withOutput(voltage));
    }

    // TODO Use enum directly
    private Distance stopHeight(Stop stop) {
        return stop.height;
    }

    public Command Move(Stop stop) {
        return IfNotBlocked(LoggedCommands.sequence("Move Elevator to " + stop,
            Commands.runOnce(() -> {
                // RobotState.updateActiveStop(stop);
                setHeight(stopHeight(stop));
                scoreTimer.stop();
            }, this),
            LoggedCommands.idle("Idle to hold elevator", this)));
    }

    public Command GoToNext() {
        return IfNotBlocked(LoggedCommands.sequence("Move Elevator to stop",
            LoggedCommands.log(() -> "Next stop: " + nextStop),
            Commands.runOnce(() -> {
                // RobotState.updateActiveStop(nextStop);
                setHeight(stopHeight(nextStop));
            }, this),
            LoggedCommands.idle("Idle to hold elevator", this)));
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
        return IfNotBlocked(LoggedCommands.startRun("Auto Elevator Up",
            () -> autoUp = false,
            () -> {
                // TODO Always flip?
                if (!autoUp && Pose.distanceTo(Pose.flipIfRed(target)) <= PoseConstants.autoUpDistance) {
                    Stop stop = stopSupplier.get();
                    // RobotState.updateActiveStop(stop);
                    setHeight(stopHeight(stop));
                    autoUp = true;
                }
            },
            this));
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
        return stallCount >= stallMax;
    }

    // Set Elevator height to given position, provided in inches
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
        stallCount = 0;
        // TODO Handle blocking
        DogLog.log("Elevator/Status", "Move to position " + String.format("%1.2f", position));
        DogLog.log("Elevator/Set Position", position);
        // RobotState.setElevatorAtZero(false);
        //mainMotor.setControl(positionVoltage.withPosition(position));
        mainMotor.setControl(positionControl.withPosition(position));
    }
    
    public boolean atTarget() {
        return Math.abs(mainMotor.getPosition().getValueAsDouble() - currentTarget.position) <= ElevatorConstants.positionError;
    }

    private Stop safetyStop() {
        // return !RobotState.getFinalSensor() ? Stop.HOLD : Stop.SAFE;
        return Stop.SAFE;
    }

    private boolean isSafe(Distance height) {
        return height.lt(safetyStop().height.plus(ElevatorConstants.safetyMargin));
    }

    private boolean isSafe() {
        return isSafe(getHeight());
    }

    private double stopError(Stop stop) {
        return Math.abs(stopHeight(stop).minus(getHeight()).magnitude());
    }

    public boolean atStop(Stop stop) {
        double stopError = stopError(stop);
        // The safe stop is just a guideline, and has a wider margin for error
        double allowableError = stop == Stop.SAFE ? 3 * ElevatorConstants.positionError : ElevatorConstants.positionError;

        return stopError <= allowableError;
    }

    public boolean aboveStop(Stop stop) {
        return getHeight().gt(stopHeight(stop));
    }

    public boolean nearStop(Stop stop) {
        double stopError = stopError(stop);
        // The safe stop is just a guideline, and has a wider margin for error
        double allowableError = stop == Stop.SAFE ? 3 * ElevatorConstants.positionError : ElevatorConstants.positionCloseError;

        return stopError <= allowableError;
    }

    // Are potentially towards a stop? (limited use cases)
    public boolean towardsStop(Stop stop) {
        return getHeight().plus(ElevatorConstants.towardsMargin).gte(stopHeight(stop));
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
        return Units.Inches.of(position / ElevatorConstants.rotPerInch).plus(ElevatorConstants.baseHeight);
    }

    private Distance getHeight() {
        return getHeight(mainMotor.getPosition().getValueAsDouble());
    }

    public double raisedPercentage() {
        return MathUtil.clamp(getHeight().minus(Stop.CORAL_HOLD.height).div(Stop.L4.height).magnitude(), 0.0, 1.0);
    }

    public Command IfNotBlocked(Command command) {
        return command;
        // return LoggedCommands.either("Block check then run " + command.getName(),
        //     command,
        //     LoggedCommands.runOnce("Blocked Elevator Warning",
        //         () -> LoggedAlert.Warning("Elevator", "Blocked", "Block Elevator prevents running " + command.getName())),
        //     () -> !RobotState.elevatorPathBlocked());
    }


    public Command MoveToSafety() {
        return Commands.either(
            Commands.sequence(
                LoggedCommands.log("Not moving by default in service mode"),
                Stop(),
                Commands.idle(this)
            ),
            LoggedCommands.sequence("Move Elevator to Safety",
                // Commands.runOnce(() -> {
                //     movingToSafety = true;
                //     safetyDeferred = false;
                // }),
                Commands.either(
                    LoggedCommands.deadline("Move to Hold position with Coral",
                        LoggedCommands.waitUntil("Wait for no Coral", () -> !RobotState.haveCoral()),
                        Move(Stop.CORAL_HOLD)),
                    LoggedCommands.sequence("Zero and Idle",
                        Commands.either(
                            Zero(),
                            FastZero(),
                            RobotState::haveAlgae),
                        LoggedCommands.idle("Elevator holding at zero", this)),
                    RobotState::coralReady))
                    // .handleInterrupt(() -> movingToSafety = false)
                    ,
            optServiceMode::get);
    }
    
    public void initDefaultCommand() {
        setDefaultCommand(MoveToSafety());
    }

    public boolean isClear(ClearState desiredState) {
        if (desiredState == ClearState.CLEAR_HIGH && clearState == ClearState.CLEAR_HIGH) {
            return true;
        } else if (desiredState == ClearState.CLEAR_LOW && (clearState == ClearState.CLEAR_LOW || clearState == ClearState.CLEAR_HIGH)) {
            return true;
        }
        return false;
    }

    private void setCurrentTarget(Stop target) {
        DogLog.log("Elevator/Status", "Current target: " + target + " <- " + currentTarget);
        currentTarget = target;
        setPosition(currentTarget.position);
    }

    public void moveTo(Stop target) {
        DogLog.log("Elevator/Status", "Final target: " + target + " <- " + finalTarget);
        finalTarget = target;

        if (target == currentTarget) {
            return;
        }
        setCurrentTarget(target);
    }

    @Override
    public void periodic() {
        Command currentCommand = getCurrentCommand();
        DogLog.log("Elevator/Current Command", currentCommand == null ? "None" : currentCommand.getName());

        double position = mainMotor.getPosition().getValueAsDouble();
        Distance height = getHeight(position);
        double followPosition = followerMotor.getPosition().getValueAsDouble();
        double followDifference = position - followPosition;
        double voltage = mainMotor.getMotorVoltage().getValueAsDouble();

        // Determine clear state of Elevator
        if (height.gt(Stop.CLEAR_HIGH.height)) {
            clearState = ClearState.CLEAR_HIGH;
        } else if (height.gt(Stop.CLEAR_LOW.height)) {
            clearState = ClearState.CLEAR_LOW;
        } else {
            clearState = ClearState.NOT_CLEAR;
        }

        if (finalTarget != currentTarget) {
            // Must be moving down and need to make sure elements are clear

            // TODO Handle the move up-and-down for withing CLEAR_LOW range
            if (currentTarget == Stop.CLEAR_HIGH) {
                // If we are headed for the CLEAR_HIGH stop, it's because we need to ensure that the algae roller is clear
                // prior to dropping below the CLEAR_HIGH stop
                if (AlgaeRoller.isClear()) {
                    if (finalTarget.height.gt(Stop.CLEAR_LOW.height)) {
                        currentTarget = finalTarget;
                    } else {
                        currentTarget = Stop.CLEAR_LOW;
                    }
                }
            } else if (currentTarget == Stop.CLEAR_LOW) {
                // If we are headed for the CLEAR_LOW stop, it's because we need to ensure that the pivot completes movement
                // prior to dropping below the CLEAR_LOW stop
                if (EndEffector.instance.inPosition()) {
                    currentTarget = finalTarget;
                }
            } else {
                LoggedAlert.Warning("Elevator", "Target", "Unexpected interim target: " + currentTarget);
            }
        }
        DogLog.log("Elevator/Current Target", currentTarget.name());
        DogLog.log("Elevator/Final Target", finalTarget.name());

        // Handle exceptions in cases other than elevator at rest
        if (voltage != 0.0) {
            // if (RobotState.elevatorPathBlocked()) {
            //     // Stop elevator when moving and blockage detected
            //     LoggedAlert.Error("Elevator", "Blocked", "Elevator stopped due to blockage");
            //     stop();
            // } else if (!isSafe() && !movingToSafety && !RobotState.raisedElevatorAllowable()) {
            //     // Elevator is unsafe, not allowed to raised, and not already moving to safety
            //     LoggedAlert.Warning("Elevator", "Safety", "Cancelling current command to return to safe position");
                
            //     if (currentCommand != null) {
            //         currentCommand.cancel();
            //     }
            // } else
            if (!atTarget() && position == lastPosition) {
                // Motor not moving -- detect stalls
                ++stallCount;
                if (isStalled()) {
                    DogLog.log("Elevator/Status", "Stall detected");
                    if (!zeroing) {
                        LoggedAlert.Warning("Elevator", "Elevator Stalled", "Elevator stopped due to stall");
                    }
                    stop();
                }
            } else {
                stallCount = 0;
            }
        }
        lastPosition = position;

        // If we have Coral ready, and the Elevator is still at zero, cancel the current default command so that it runs again with the L1 default
        // if (RobotState.coralReady() && RobotState.getElevatorAtZero()) {
        //     if (currentCommand != null) {
        //         currentCommand.cancel();
        //         RobotState.setElevatorAtZero(false);
        //     }
        // }
        // TODO Lower Elevator if we don't have Coral?

        if (Math.abs(followDifference) >= positionDiffMax) {
            // This seems to be a semi-normal experience, perhaps due to latency in reporting motor position at faster speeds
            // The motors are mechanically connected, so it really should be impossible to actually be out of sync
            // LoggedAlert.Warning("Elevator", "Elevator Unequal", "Elevator motor position difference of " + String.format("%01.2f", followDifference) + " exceeds limit");
            // stop();
        }

        DogLog.log("Elevator/height", height.in(Units.Inches));
        DogLog.log("Elevator/leftPosition", position);
        DogLog.log("Elevator/leftVelocity", mainMotor.getVelocity().getValueAsDouble());
        DogLog.log("Elevator/leftVoltage", voltage);
        DogLog.log("Elevator/rightPosition", followPosition);
        DogLog.log("Elevator/rightVelocity", followerMotor.getVelocity().getValueAsDouble());
        DogLog.log("Elevator/rightVoltage", followerMotor.getMotorVoltage().getValueAsDouble());
        DogLog.log("Elevator/stallCount", stallCount);
        DogLog.log("Elevator/clearState", clearState.name());

        SmartDashboard.putBoolean("Elevator/Stalled", isStalled());
        SmartDashboard.putBoolean("Elevator/Moving", voltage != 0.0);
        SmartDashboard.putBoolean("Elevator/At Target", atTarget());

        SmartDashboard.putBoolean("Elevator/Safe", isSafe(height));
        // SmartDashboard.putBoolean("Elevator/HOLD", atStop(Stop.HOLD));
        SmartDashboard.putBoolean("Elevator/L1", atStop(Stop.L1));
        SmartDashboard.putBoolean("Elevator/L2", atStop(Stop.L2));
        SmartDashboard.putBoolean("Elevator/L3", atStop(Stop.L3));
        SmartDashboard.putBoolean("Elevator/L4", atStop(Stop.L4));
        SmartDashboard.putString("Elevator/Next Stop", nextStop.toString());

        // mechanism.setLength(Units.inchesToMeters(height));
    }
}