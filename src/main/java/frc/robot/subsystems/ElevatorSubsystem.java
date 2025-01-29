package frc.robot.subsystems;

import java.util.EnumMap;
import java.util.Map;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Elastic;
import frc.lib.util.Elastic.Notification;
import frc.lib.util.Elastic.Notification.NotificationLevel;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommands;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
    private final PositionVoltage positionVoltage = new PositionVoltage(0.0).withEnableFOC(true);
    private final MechanismLigament2d mech2d;

    private int stallCount = 0;
    private final int stallMax = 3;
    private double lastPosition = 0.0;
    
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
        SmartDashboard.putData("Elevator/Zero", Zero());
        SmartDashboard.putData("Elevator/SetZero", SetZero());

        Mechanism2d mechanism = new Mechanism2d(Constants.Elevator.maxHeight, Constants.Elevator.maxHeight);
        MechanismRoot2d root = mechanism.getRoot("elevator-root", Constants.Elevator.maxHeight / 2, 0);
        mech2d = root.append(new MechanismLigament2d("elevator", Constants.Elevator.baseHeight, 90));

        SmartDashboard.putData("Elevator/mechanism", mechanism);
    }

    public void setAsZero() {
        leftMotor.setPosition(0);
        rightMotor.setPosition(0);
    }

    public Command SetZero() {
        return LoggedCommands.runOnce("Set Elevator Zero", this::setAsZero);
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
                stallCount = 0;
                leftMotor.setControl(voltageOut.withOutput(Constants.Elevator.slowVoltage));
            },
            this).ignoringDisable(true);
    }

    public Command Lower() {
        return LoggedCommands.runOnce("Lower Elevator",
            () -> {
                stallCount = 0;
                leftMotor.setControl(voltageOut.withOutput(-Constants.Elevator.slowVoltage));
            },
            this);
    }

    public Command Stop() {
        return LoggedCommands.runOnce("Stop Elevator", this::stop, this);
    }

    private void stop() {
        leftMotor.stopMotor();
        //rightMotor.stopMotor();
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
            Elastic.sendNotification(new Notification(NotificationLevel.WARNING, "Elevator Range", "Requested elevator height too high"));
            height = Constants.Elevator.maxHeight;
        }
        if (height < Constants.Elevator.baseHeight) {
            Elastic.sendNotification(new Notification(NotificationLevel.WARNING, "Elevator Range", "Requested elevator height too low"));
            height = Constants.Elevator.baseHeight;
        }

        double position = (height - Constants.Elevator.baseHeight) * Constants.Elevator.rotPerInch;
        stallCount = 0;
        leftMotor.setControl(positionVoltage.withPosition(position));
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
            if (position == lastPosition) {
                ++stallCount;
                if (isStalled()) {
                    Elastic.sendNotification(new Notification(NotificationLevel.WARNING, "Elevator Stalled", "Elevator stopped due to stall"));
                    stop();
                }
            } else {
                stallCount = 0;
            }
        }
        lastPosition = position;

        if (Math.abs(followDifference) >= positionDiffMax) {
            Elastic.sendNotification(new Notification(NotificationLevel.ERROR, "Elevator Unequal", "Elevator motor position difference of " + String.format("%01.2f", followDifference) + " exceeds limit"));
            stop();
        }

        DogLog.log("Elevator/leftPosition", position);
        DogLog.log("Elevator/leftVelocity", leftMotor.getVelocity().getValueAsDouble());
        DogLog.log("Elevator/leftVoltage", voltage);
        DogLog.log("Elevator/rightPosition", followPosition);
        DogLog.log("Elevator/rightVelocity", rightMotor.getVelocity().getValueAsDouble());
        DogLog.log("Elevator/rightVoltage", rightMotor.getMotorVoltage().getValueAsDouble());
        DogLog.log("Elevator/stallCount", stallCount);

        SmartDashboard.putBoolean("elevator/stalled", isStalled());
        SmartDashboard.putBoolean("elevator/moving", voltage != 0.0);

        mech2d.setLength(height);
    }
}