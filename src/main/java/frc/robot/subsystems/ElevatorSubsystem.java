package frc.robot.subsystems;

import java.util.EnumMap;
import java.util.Map;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    public enum Stop {
        ZERO,
        INTAKE,
        L1,
        L2,
        L3,
        L4
    };

    // TODO Define positions
    private final EnumMap<Stop, Double> elevatorPositions = new EnumMap<>(Map.ofEntries(
      Map.entry(Stop.ZERO, 0.0),
      Map.entry(Stop.INTAKE, 10.0),
      Map.entry(Stop.L1, 20.0),
      Map.entry(Stop.L2, 30.0),
      Map.entry(Stop.L3, 40.0),
      Map.entry(Stop.L4, 50.0)
    ));

    public ElevatorSubsystem() {
        leftMotor = new TalonFX(Constants.Elevator.leftID, Constants.Elevator.canBus);
        rightMotor = new TalonFX(Constants.Elevator.rightID, Constants.Elevator.canBus);
        applyConfigs();

        SmartDashboard.putData("Elevator/Raise", Raise());
        SmartDashboard.putData("Elevator/Lower", Lower());
        SmartDashboard.putData("Elevator/Zero", Zero());
    }

    public void setAsZero() {

    }

    public Command Zero() {
        return LoggedCommands.runOnce("Zero Elevator",
            () -> {
                Elastic.sendNotification(new Notification(NotificationLevel.ERROR, "Not implemented", "Zeroing the elevator has not yet been implemented."));
            },
            this);
    }

    public Command Raise() {
        return LoggedCommands.runOnce("Raise Elevator",
            () -> {
                leftMotor.setControl(voltageOut.withOutput(Constants.Elevator.slowVoltage));
            },
            this);
    }

    public Command Lower() {
        return LoggedCommands.runOnce("Lower Elevator",
            () -> {
                leftMotor.setControl(voltageOut.withOutput(-Constants.Elevator.slowVoltage));
            },
            this);
    }

    public Command Move(Stop stop) {
        return LoggedCommands.runOnce("Move Elevator to " + stop,
            () -> {
                setPosition(elevatorPositions.get(stop));
            },
            this);
    }

    private void setPosition(double position) {
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
        DogLog.log("Elevator/leftPosition", leftMotor.getPosition().getValueAsDouble());
        DogLog.log("Elevator/leftVelocity", leftMotor.getVelocity().getValueAsDouble());
        DogLog.log("Elevator/leftVoltage", leftMotor.getMotorVoltage().getValueAsDouble());
    }
}