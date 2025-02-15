package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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
import frc.robot.Constants;
import frc.robot.commands.LoggedCommands;
import frc.robot.subsystems.RobotState.CoralState;
import frc.robot.subsystems.RobotState.GamePiece;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
    private final PositionVoltage positionVoltage = new PositionVoltage(0.0).withEnableFOC(true);
    private final MechanismLigament2d mechanism;

    private Stop nextStop = Stop.SAFE;

    private int stallCount = 0;
    private final int stallMax = 3;
    private double lastPosition = 0.0;
    private double desiredPosition = -1.0;
    private boolean zeroing = false;
    private boolean movingToSafety = false;
    private boolean autoUp = false;
    
    private final double positionDiffMax = 0.5;

    public enum Stop {
        // Intake occurs at zero
        SAFE     (Constants.Elevator.baseHeight + 5.0),
        L1       (26.0  - Constants.Elevator.endEffectorHeight),
        L2       (34.5  - Constants.Elevator.endEffectorHeight),
        L2_ALGAE (38.0  - Constants.Elevator.endEffectorHeight),
        L3       (50.0  - Constants.Elevator.endEffectorHeight),
        L3_ALGAE (55.0  - Constants.Elevator.endEffectorHeight),
        L4       (75.0  - Constants.Elevator.endEffectorHeight),
        L4_SCORE (77.75 - Constants.Elevator.endEffectorHeight);

        Stop(double height) {
            this.height = height;
        }

        public final double height;
    }

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

        double canvasWidth = Constants.Swerve.wheelBase * 1.5;
        double canvasHeight = Units.inchesToMeters(Constants.Elevator.maxHeight) * 1.25;
        Mechanism2d canvas = new Mechanism2d(canvasWidth, canvasHeight, new Color8Bit(Color.kLightGray));
        MechanismRoot2d origin = canvas.getRoot("elevator-root", canvasWidth / 2.0, 0);
        MechanismLigament2d offset = origin.append(new MechanismLigament2d("elevator-offset", canvasWidth / 2.0  - Units.inchesToMeters(Constants.Elevator.setback), 0.0, 1.0, new Color8Bit()));
        mechanism = offset.append(new MechanismLigament2d("elevator", Units.inchesToMeters(Constants.Elevator.baseHeight), 90.0, Units.inchesToMeters(Constants.Elevator.thickness), new Color8Bit(0xBF, 0x57, 0x00)));

        SmartDashboard.putData("Elevator/mechanism", canvas);
        // initDefaultCommand();
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
        return LoggedCommands.sequence("Zero Elevator",
            Commands.runOnce(() -> zeroing = true),
            Commands.deadline(
                LoggedCommands.waitUntil("Wait for stall", this::isStalled),
                Lower()).handleInterrupt(() -> zeroing = false),
            SetZero());
    }

    public Command FastZero() {
        return LoggedCommands.sequence("Fast Zero",
            Commands.deadline(
                LoggedCommands.waitUntil("Wait for elevator in safe zone", this::isSafe),
                Move(Stop.SAFE)),
            Zero());
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

    public Command Move(Stop stop) {
        return LoggedCommands.sequence("Move Elevator to " + stop,
            Commands.runOnce(() -> setHeight(stop.height), this),
            LoggedCommands.idle("Idle to hold elevator", this));
    }

    public Command GoToNext() {
        return LoggedCommands.sequence("Move Elevator to stop",
            LoggedCommands.log(() -> "Next stop: " + nextStop),
            Commands.runOnce(() -> setHeight(nextStop.height), this),
            LoggedCommands.idle("Idle to hold elevator", this));
    }

    public void setNextStop(Stop stop) {
        DogLog.log("Elevator/Status", "Next stop = " + stop);
        nextStop = stop;
    }

    public Command AutoElevatorUp(Translation2d target) {
        return LoggedCommands.startRun(
            "Auto Elevator Up",
            () -> autoUp = false,
            () -> {
                if (!autoUp && PoseSubsystem.distanceTo(target) <= Constants.Pose.autoUpDistance) {
                    setHeight(nextStop.height);
                    autoUp = true;
                }
            },
            this);
    };

    public Command WaitForNext() {
        return LoggedCommands.waitUntil("Wait for Elevator at next stop", () -> atStop(nextStop));
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

    private boolean atStop(Stop stop) {
        double stopError = Math.abs(stop.height - getHeight());
        // The safe stop is just a guideline, and has a wider margin for error
        double allowableError = stop == Stop.SAFE ? 3 * Constants.Elevator.positionError : Constants.Elevator.positionError;

        return stopError <= allowableError;
    }

    private double getHeight(double position) {
        return position / Constants.Elevator.rotPerInch + Constants.Elevator.baseHeight;
    }

    private double getHeight() {
        return getHeight(leftMotor.getPosition().getValueAsDouble());
    }

    private void applyConfigs() {
        // Configure the primary motor
        var motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.NeutralMode = Constants.Elevator.motorNeutralValue;
        motorConfig.MotorOutput.Inverted = Constants.Elevator.motorOutputInverted;
        motorConfig.Voltage.PeakForwardVoltage = Constants.Elevator.peakForwardVoltage;
        motorConfig.Voltage.PeakReverseVoltage = Constants.Elevator.peakReverseVoltage;
    
        // PID & FF configuration
        motorConfig.Slot0.kP = Constants.Elevator.kP;
        motorConfig.Slot0.kI = Constants.Elevator.kI;
        motorConfig.Slot0.kD = Constants.Elevator.kD;
        motorConfig.Slot0.kS = Constants.Elevator.kS;
        motorConfig.Slot0.kV = Constants.Elevator.kV;
        motorConfig.Slot0.kA = Constants.Elevator.kA;
        motorConfig.Slot0.kG = Constants.Elevator.kG;
    
        // Set Motion Magic settings
        var motionMagicConfigs = motorConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Elevator.cruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = Constants.Elevator.acceleration;
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
            Commands.runOnce(() -> movingToSafety = true),
            Commands.either(
                Move(Stop.L1),
                LoggedCommands.sequence("Zero and Idle",
                    FastZero(),
                    LoggedCommands.idle("Elevator holding at zero", this)),
                () -> RobotState.getActiveGamePiece() == GamePiece.CORAL && RobotState.getCoralState() == CoralState.READY))
            .handleInterrupt(() -> movingToSafety = false);
    }
    
    public void initDefaultCommand() {
        setDefaultCommand(MoveToSafety());
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

        if (RobotState.getActiveGamePiece() == GamePiece.CORAL && RobotState.getCoralState() == CoralState.SCORING && atStop(Stop.L4)) {
            Move(Stop.L4_SCORE).schedule();
        }

        // If we have Coral ready, and the Elevator is still at zero, cancel the current default command so that it runs again with the L1 default
        if (RobotState.getActiveGamePiece() == GamePiece.CORAL && RobotState.getCoralState() == CoralState.READY && RobotState.getElevatorAtZero()) {
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

        mechanism.setLength(Units.inchesToMeters(height));
    }
}