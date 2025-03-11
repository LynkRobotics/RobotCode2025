package frc.robot.subsystems;

import static frc.robot.Options.optBonusCoralStandoff;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedAlert;
import frc.lib.util.LoggedCommands;
import frc.robot.Constants;
import frc.robot.Constants.Elevator.Stop;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
    // TODO Use MotionMagicVoltage?
    private final PositionVoltage positionVoltage = new PositionVoltage(0.0).withEnableFOC(true);
    // private final MechanismLigament2d mechanism;

    private Stop nextStop = Stop.SAFE;

    private int stallCount = 0;
    private final int stallMax = 3;
    private double lastPosition = 0.0;
    private double desiredPosition = -1.0;
    private boolean zeroing = false;
    private boolean movingToSafety = false;
    private boolean safetyDeferred = false;
    private boolean autoUp = false;
    private Timer scoreTimer = new Timer();
    
    private final double positionDiffMax = 0.5;

    public ElevatorSubsystem() {
        leftMotor = new TalonFX(Constants.Elevator.leftID, Constants.Elevator.canBus);
        rightMotor = new TalonFX(Constants.Elevator.rightID, Constants.Elevator.canBus);
        applyConfigs();

        SmartDashboard.putData("Elevator/Raise", Raise());
        SmartDashboard.putData("Elevator/Lower", Lower());
        SmartDashboard.putData("Elevator/Stop", Stop());
        SmartDashboard.putNumber("Elevator/Direct Voltage", Constants.Elevator.slowVoltage);
        SmartDashboard.putData("Elevator/Set Voltage", LoggedCommands.runOnce("Set Voltage", () -> { setVoltage(SmartDashboard.getNumber("Elevator/Direct Voltage", 0.0));}));
        SmartDashboard.putNumber("Elevator/Direct Position", 0.0);
        SmartDashboard.putData("Elevator/Set Position", LoggedCommands.runOnce("Set Position", () -> { setPosition(SmartDashboard.getNumber("Elevator/Direct Position", 0.0));}));
        SmartDashboard.putNumber("Elevator/Direct Height", 0.0);
        SmartDashboard.putData("Elevator/Set Height", LoggedCommands.runOnce("Set Height", () -> { setHeight(SmartDashboard.getNumber("Elevator/Direct Height", 12.0));}));
        SmartDashboard.putData("Elevator/Zero", Zero());
        SmartDashboard.putData("Elevator/SetZero", SetZero());
        SmartDashboard.putData("Elevator/FastZero", FastZero());

        // double canvasWidth = Constants.Swerve.wheelBase * 1.5;
        // double canvasHeight = Units.inchesToMeters(Constants.Elevator.maxHeight) * 1.25;
        // Mechanism2d canvas = new Mechanism2d(canvasWidth, canvasHeight, new Color8Bit(Color.kLightGray));
        // MechanismRoot2d origin = canvas.getRoot("elevator-root", canvasWidth / 2.0, 0);
        // MechanismLigament2d offset = origin.append(new MechanismLigament2d("elevator-offset", canvasWidth / 2.0  - Units.inchesToMeters(Constants.Elevator.setback), 0.0, 1.0, new Color8Bit()));
        // mechanism = offset.append(new MechanismLigament2d("elevator", Units.inchesToMeters(Constants.Elevator.baseHeight), 90.0, Units.inchesToMeters(Constants.Elevator.thickness), new Color8Bit(0xBF, 0x57, 0x00)));

        // TODO Get mechanism working
        // SmartDashboard.putData("Elevator/mechanism", canvas);
        initDefaultCommand();
    }

    public void setAsZero() {
        DogLog.log("Elevator/Status", "Set as Zero");
        RobotState.setElevatorAtZero(true);
        leftMotor.setPosition(0);
        rightMotor.setPosition(0);
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
                setVoltage(Constants.Elevator.slowVoltage);
            },
            this);
    }

    public Command Lower() {
        return LoggedCommands.runOnce("Lower Elevator",
        () -> {
            setVoltage(-Constants.Elevator.slowVoltage);
        },
        this);
    }

    public Command Stop() {
        return LoggedCommands.runOnce("Stop Elevator", this::stop, this);
    }

    private void stop() {
        DogLog.log("Elevator/Status", "Stopped");
        leftMotor.stopMotor();
    }

    private void setVoltage(double voltage) {
        stallCount = 0;
        desiredPosition = -1.0;
        if (RobotState.elevatorPathBlocked()) {
            LoggedAlert.Error("Elevator", "Blocked", "Cannot set voltage on elevator while blocked");
            return;
        }
        DogLog.log("Elevator/Status", "Set voltage " + String.format("%1.2f", voltage));
        RobotState.setElevatorAtZero(false);
        leftMotor.setControl(voltageOut.withOutput(voltage));
    }

    private double stopHeight(Stop stop) {
        double height = stop.height;

        if (optBonusCoralStandoff.get() && (stop == Stop.L1 || stop == Stop.L2 || stop == Stop.L3 || stop == Stop.L4)) {
            height += Constants.Elevator.standoffBoost;
        }

        return height;
    }

    public Command Move(Stop stop) {
        return IfNotBlocked(LoggedCommands.sequence("Move Elevator to " + stop,
            Commands.runOnce(() -> {
                RobotState.updateActiveStop(stop);
                setHeight(stopHeight(stop));
                scoreTimer.stop();
            }, this),
            LoggedCommands.idle("Idle to hold elevator", this)));
    }

    public Command GoToNext() {
        return IfNotBlocked(LoggedCommands.sequence("Move Elevator to stop",
            LoggedCommands.log(() -> "Next stop: " + nextStop),
            Commands.runOnce(() -> {
                RobotState.updateActiveStop(nextStop);
                setHeight(stopHeight(nextStop));
            }, this),
            LoggedCommands.idle("Idle to hold elevator", this)));
    }

    public void setNextStop(Stop stop) {
        DogLog.log("Elevator/Status", "Next stop = " + stop);
        RobotState.updateNextStop(stop);
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
                if (!autoUp && PoseSubsystem.distanceTo(PoseSubsystem.flipIfRed(target)) <= Constants.Pose.autoUpDistance) {
                    Stop stop = stopSupplier.get();
                    RobotState.updateActiveStop(stop);
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

    private boolean isStalled() {
        return stallCount >= stallMax;
    }

    // Set Elevator height to given position, provided in inches
    private void setHeight(double height) {
        if (height > Constants.Elevator.maxHeight) {
            LoggedAlert.Warning("Elevator", "Elevator Range", "Requested elevator height too high");
            height = Constants.Elevator.maxHeight;
        }
        if (height < Constants.Elevator.baseHeight) {
            LoggedAlert.Warning("Elevator", "Elevator Range", "Requested elevator height too low");
            height = Constants.Elevator.baseHeight;
        }
        DogLog.log("Elevator/Status", "Move to height " + String.format("%1.1f", height));

        double position = (height - Constants.Elevator.baseHeight) * Constants.Elevator.rotPerInch;
        setPosition(position);
    }

    private void setPosition(double position) {
        stallCount = 0;
        desiredPosition = position;
        if (RobotState.elevatorPathBlocked()) {
            LoggedAlert.Error("Elevator", "Blocked", "Cannot move elevator while blocked");
            return;
        }
        DogLog.log("Elevator/Status", "Move to position " + String.format("%1.2f", position));
        DogLog.log("Elevator/Set Position", position);
        RobotState.setElevatorAtZero(false);
        leftMotor.setControl(positionVoltage.withPosition(position));
    }
    
    private boolean inRange(double position) {
        return desiredPosition >= 0.0 && Math.abs(desiredPosition - position) <= Constants.Elevator.positionError;
    }

    private Stop safetyStop() {
        return !RobotState.getFinalSensor() ? Stop.L1 : Stop.SAFE;
    }

    private boolean isSafe(double height) {
        return height < (safetyStop().height + Constants.Elevator.safetyMargin);
    }

    private boolean isSafe() {
        return isSafe(getHeight());
    }

    private double stopError(Stop stop) {
        return Math.abs(stopHeight(stop) - getHeight());
    }

    public boolean atStop(Stop stop) {
        double stopError = stopError(stop);
        // The safe stop is just a guideline, and has a wider margin for error
        double allowableError = stop == Stop.SAFE ? 3 * Constants.Elevator.positionError : Constants.Elevator.positionError;

        return stopError <= allowableError;
    }

    public boolean aboveStop(Stop stop) {
        return getHeight() > stopHeight(stop);
    }

    public boolean nearStop(Stop stop) {
        double stopError = stopError(stop);
        // The safe stop is just a guideline, and has a wider margin for error
        double allowableError = stop == Stop.SAFE ? 3 * Constants.Elevator.positionError : Constants.Elevator.positionCloseError;

        return stopError <= allowableError;
    }

    public boolean atNextStop() {
        return atStop(nextStop);
    }

    public boolean nearNextStop() {
        return nearStop(nextStop);
    }

    private double getHeight(double position) {
        return position / Constants.Elevator.rotPerInch + Constants.Elevator.baseHeight;
    }

    private double getHeight() {
        return getHeight(leftMotor.getPosition().getValueAsDouble());
    }

    public double raisedPercentage() {
        return MathUtil.clamp((getHeight() - Stop.L1.height) / Stop.L4_SCORE.height, 0.0, 1.0);
    }

    public Command IfNotBlocked(Command command) {
        return LoggedCommands.either("Block check then run " + command.getName(),
            command,
            LoggedCommands.runOnce("Blocked Elevator Warning",
                () -> LoggedAlert.Warning("Elevator", "Blocked", "Block Elevator prevents running " + command.getName())),
            () -> !RobotState.elevatorPathBlocked());
    }

    private void applyConfigs() {
        // Configure the primary motor
        var motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.NeutralMode = Constants.Elevator.motorNeutralValue;
        motorConfig.MotorOutput.Inverted = Constants.Elevator.motorOutputInverted;
        motorConfig.Voltage.PeakForwardVoltage = Constants.Elevator.peakForwardVoltage;
        motorConfig.Voltage.PeakReverseVoltage = Constants.Elevator.peakReverseVoltage;
        motorConfig.CurrentLimits.StatorCurrentLimit = 80; //TODO: Test 
        motorConfig.CurrentLimits.SupplyCurrentLimit = 120; //TODO: Make these constants
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    
        // PID & FF configuration
        motorConfig.Slot0.kP = Constants.Elevator.kP;
        motorConfig.Slot0.kI = Constants.Elevator.kI;
        motorConfig.Slot0.kD = Constants.Elevator.kD;
        motorConfig.Slot0.kS = Constants.Elevator.kS;
        motorConfig.Slot0.kV = Constants.Elevator.kV;
        motorConfig.Slot0.kA = Constants.Elevator.kA;
        motorConfig.Slot0.kG = Constants.Elevator.kG;
    
        // Set Motion Magic settings
        // var motionMagicConfigs = motorConfig.MotionMagic;
        // motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Elevator.cruiseVelocity;
        // motionMagicConfigs.MotionMagicAcceleration = Constants.Elevator.acceleration;
        // motionMagicConfigs.MotionMagicJerk = Constants.Elevator.jerk;
    
        // Apply motor config
        leftMotor.getConfigurator().apply(motorConfig);

        // Ensure that left motor begins stopped
        leftMotor.stopMotor();

        // Set right motor to follow left, but opposite direction
        rightMotor.setControl(new Follower(Constants.Elevator.leftID, true));
    }

    public Command MoveToSafety() {
        return LoggedCommands.sequence("Move Elevator to Safety",
            Commands.runOnce(() -> {
                movingToSafety = true;
                safetyDeferred = false;
            }),
            Commands.either(
                LoggedCommands.deadline("Move to L1 with Coral",
                    LoggedCommands.waitUntil("Wait for no Coral", () -> !RobotState.haveCoral()),
                    Move(Stop.L1)),
                LoggedCommands.sequence("Zero and Idle",
                    Commands.either(
                        Zero(),
                        FastZero(),
                        RobotState::haveAlgae),
                    LoggedCommands.idle("Elevator holding at zero", this)),
                RobotState::coralReady))
                .handleInterrupt(() -> movingToSafety = false);
    }
    
    public void initDefaultCommand() {
        setDefaultCommand(LoggedCommands.either("Elevator Default Command",
            MoveToSafety(),
            LoggedCommands.sequence("Defer Elevator Safety",
                Commands.runOnce(() -> safetyDeferred = true),
                LoggedCommands.idle("Idle to hold elevator", this)),
            RobotState::elevatorDownAllowed));
    }

    @Override
    public void periodic() {
        double position = leftMotor.getPosition().getValueAsDouble();
        double height = getHeight(position);
        double followPosition = rightMotor.getPosition().getValueAsDouble();
        double followDifference = position - followPosition;
        double voltage = leftMotor.getMotorVoltage().getValueAsDouble();

        // Handle exceptions in cases other than elevator at rest
        if (voltage != 0.0) {
            if (RobotState.elevatorPathBlocked()) {
                // Stop elevator when moving and blockage detected
                LoggedAlert.Error("Elevator", "Blocked", "Elevator stopped due to blockage");
                stop();
            } else if (safetyDeferred && RobotState.elevatorDownAllowed()) {
                // Moving elevator to safety was deferred, but is available now
                Command currentCommand = getCurrentCommand();
                
                if (currentCommand != null) {
                    currentCommand.cancel();
                }
            } else if (!isSafe() && !movingToSafety && !RobotState.raisedElevatorAllowable()) {
                // Elevator is unsafe, not allowed to raised, and not already moving to safety
                LoggedAlert.Warning("Elevator", "Safety", "Cancelling current command to return to safe position");
                Command currentCommand = getCurrentCommand();
                
                if (currentCommand != null) {
                    currentCommand.cancel();
                }
            } else if (!inRange(position) && position == lastPosition) {
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

        // TODO Can we move this into ScoreGamePiece command?
        if (RobotState.coralScoring() && atStop(Stop.L4)) {
            setHeight(Stop.L4_SCORE.height); // TODO Avoid repeatedly calling ... even though it might not be an issue?
        }

        // If we have Coral ready, and the Elevator is still at zero, cancel the current default command so that it runs again with the L1 default
        if (RobotState.coralReady() && RobotState.getElevatorAtZero()) {
            Command currentCommand = getCurrentCommand();
            if (currentCommand != null) {
                currentCommand.cancel();
                RobotState.setElevatorAtZero(false);
            }
        }
        // TODO Lower Elevator if we don't have Coral?

        if (Math.abs(followDifference) >= positionDiffMax) {
            // This seems to be a semi-normal experience, perhaps due to latency in reporting motor position at faster speeds
            // The motors are mechanically connected, so it really should be impossible to actually be out of sync
            // LoggedAlert.Warning("Elevator", "Elevator Unequal", "Elevator motor position difference of " + String.format("%01.2f", followDifference) + " exceeds limit");
            // stop();
        }

        DogLog.log("Elevator/height", height);
        DogLog.log("Elevator/leftPosition", position);
        DogLog.log("Elevator/leftVelocity", leftMotor.getVelocity().getValueAsDouble());
        DogLog.log("Elevator/leftVoltage", voltage);
        DogLog.log("Elevator/rightPosition", followPosition);
        DogLog.log("Elevator/rightVelocity", rightMotor.getVelocity().getValueAsDouble());
        DogLog.log("Elevator/rightVoltage", rightMotor.getMotorVoltage().getValueAsDouble());
        DogLog.log("Elevator/stallCount", stallCount);

        SmartDashboard.putBoolean("Elevator/Stalled", isStalled());
        SmartDashboard.putBoolean("Elevator/Moving", voltage != 0.0);
        SmartDashboard.putBoolean("Elevator/In Range", inRange(position));

        SmartDashboard.putBoolean("Elevator/Safe", isSafe(height));
        SmartDashboard.putBoolean("Elevator/L1", atStop(Stop.L1));
        SmartDashboard.putBoolean("Elevator/L2", atStop(Stop.L2));
        SmartDashboard.putBoolean("Elevator/L3", atStop(Stop.L3));
        SmartDashboard.putBoolean("Elevator/L4", atStop(Stop.L4));
        SmartDashboard.putString("Elevator/Next Stop", nextStop.toString());

        // mechanism.setLength(Units.inchesToMeters(height));
    }
}