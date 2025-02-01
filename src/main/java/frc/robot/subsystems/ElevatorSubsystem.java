package frc.robot.subsystems;

import java.util.EnumMap;
import java.util.Map;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
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

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
    private final PositionVoltage positionVoltage = new PositionVoltage(0.0).withEnableFOC(true);
    private final MechanismLigament2d mechanism;

    private int stallCount = 0;
    private final int stallMax = 3;
    private double lastPosition = 0.0;
    private double desiredPosition = -1.0;
    
    private final double positionDiffMax = 0.5;

    public enum Stop {
        INTAKE,
        L1,
        L2,
        L3,
        L4
    };

    // TODO Refine heights
    // Elevator heights are defined in terms off inches that the elevator is off the ground
    private final EnumMap<Stop, Double> elevatorHeights = new EnumMap<>(Map.ofEntries(
      Map.entry(Stop.INTAKE, 24.0),
      Map.entry(Stop.L1, 20.0 - Constants.Elevator.endEffectorHeight),
      Map.entry(Stop.L2, 30.5 - Constants.Elevator.endEffectorHeight),
      Map.entry(Stop.L3, 45.5 - Constants.Elevator.endEffectorHeight),
      Map.entry(Stop.L4, 71.5 - Constants.Elevator.endEffectorHeight)
    ));

    public ElevatorSubsystem() {
        leftMotor = new TalonFX(Constants.Elevator.leftID, Constants.Elevator.CanBus);
        rightMotor = new TalonFX(Constants.Elevator.rightID, Constants.Elevator.CanBus);
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

        double canvasWidth = Constants.Swerve.wheelBase * 1.5;
        double canvasHeight = Units.inchesToMeters(Constants.Elevator.maxHeight) * 1.25;
        Mechanism2d canvas = new Mechanism2d(canvasWidth, canvasHeight, new Color8Bit(Color.kLightGray));
        MechanismRoot2d origin = canvas.getRoot("elevator-root", canvasWidth / 2.0, 0);
        MechanismLigament2d offset = origin.append(new MechanismLigament2d("elevator-offset", canvasWidth / 2.0  - Units.inchesToMeters(Constants.Elevator.setback), 0.0, 1.0, new Color8Bit()));
        mechanism = offset.append(new MechanismLigament2d("elevator", Units.inchesToMeters(Constants.Elevator.baseHeight), 90.0, Units.inchesToMeters(Constants.Elevator.thickness), new Color8Bit(0xBF, 0x57, 0x00)));

        SmartDashboard.putData("Elevator/mechanism", canvas);
    }

    public void setAsZero() {
        leftMotor.setPosition(0);
        rightMotor.setPosition(0);
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
        leftMotor.stopMotor();
        // rightMotor.stopMotor();
    }

    private void setVoltage(double voltage) {
        stallCount = 0;
        desiredPosition = -1.0;
        leftMotor.setControl(voltageOut.withOutput(voltage));
    }

    public Command Move(Stop stop) {
        return LoggedCommands.runOnce("Move Elevator to " + stop,
            () -> {
                setHeight(elevatorHeights.get(stop));
            },
            this);
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
        leftMotor.setControl(positionVoltage.withPosition(position));
    }
    
    private boolean inRange(double position) {
        return desiredPosition >= 0.0 && Math.abs(desiredPosition - position) <= Constants.Elevator.positionError;
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

    @Override
    public void periodic() {
        double position = leftMotor.getPosition().getValueAsDouble();
        double height = position / Constants.Elevator.rotPerInch + Constants.Elevator.baseHeight;
        double followPosition = rightMotor.getPosition().getValueAsDouble();
        double followDifference = position - followPosition;
        double voltage = leftMotor.getMotorVoltage().getValueAsDouble();

        if (voltage != 0.0) {
            if (!inRange(position) && position == lastPosition) {
                ++stallCount;
                if (isStalled()) {
                    LoggedAlert.Warning("Elevator", "Elevator Stalled", "Elevator stopped due to stall");
                    stop();
                }
            } else {
                stallCount = 0;
            }
        }
        lastPosition = position;

        if (Math.abs(followDifference) >= positionDiffMax) {
            LoggedAlert.Error("Elevator", "Elevator Unequal", "Elevator motor position difference of " + String.format("%01.2f", followDifference) + " exceeds limit");
            stop();
        }

        DogLog.log("Elevator/height", height);
        DogLog.log("Elevator/leftPosition", position);
        DogLog.log("Elevator/leftVelocity", leftMotor.getVelocity().getValueAsDouble());
        DogLog.log("Elevator/leftVoltage", voltage);
        DogLog.log("Elevator/rightPosition", followPosition);
        DogLog.log("Elevator/rightVelocity", rightMotor.getVelocity().getValueAsDouble());
        DogLog.log("Elevator/rightVoltage", rightMotor.getMotorVoltage().getValueAsDouble());
        DogLog.log("Elevator/stallCount", stallCount);

        SmartDashboard.putBoolean("elevator/stalled", isStalled());
        SmartDashboard.putBoolean("elevator/moving", voltage != 0.0);

        mechanism.setLength(Units.inchesToMeters(height));
    }
}