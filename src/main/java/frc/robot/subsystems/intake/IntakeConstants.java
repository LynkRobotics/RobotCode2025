package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;

public class IntakeConstants {
    /* IDs */
    public static final int deployMotorID = 10;
    public static final int intakeMotorID = 11;
    public static final int indexMotorID = 12;

    /* CANbus */
    public static final String canBus = "rio";

    /* Motor Config Values */
    public static final double peakForwardVoltage = 12.0; 
    public static final double peakReverseVoltage = -12.0; 
    public static final InvertedValue motorOutputInverted = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue motorNeutralValue = NeutralModeValue.Brake;

    /* Motor Control Values */
    public static final double deployGearing = 40.0;
    public static final double indexGearing = 2.5;
    public static final double intakeGearing = (24.0 / 12.0);
	public static final Voltage indexVoltage = Units.Volts.of(10.0);
	public static final Voltage indexExpelVoltage = Units.Volts.of(-8.0);
    public static final Voltage intakeStartVoltage = Units.Volts.of(3.0);
	public static final Voltage intakeVoltage = Units.Volts.of(-12.0);
	public static final Voltage intakeExpelVoltage = Units.Volts.of(-12.0);

    public static TalonFXConfiguration getDeployConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = 180.0;
		config.Slot0.kD = 0.0;
		config.Slot0.kS = 0.0;
		config.Slot0.kG = 0.0;

		config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
		config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

		config.MotionMagic.MotionMagicCruiseVelocity = 7.0;
		config.MotionMagic.MotionMagicAcceleration = 15.0;

		config.Voltage.PeakForwardVoltage = 12.0;
		config.Voltage.PeakReverseVoltage = -12.0;

		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.CurrentLimits.SupplyCurrentLimit = 40.0;

		config.Feedback.SensorToMechanismRatio = deployGearing;

		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		// config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		// config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kFullStowPosition.in(Units.Rotations);

		// config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		// config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kDeployPosition.in(Units.Rotations);

        return config;
    }

    public static TalonFXConfiguration getIndexConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();

		config.CurrentLimits.StatorCurrentLimitEnable = true;
		config.CurrentLimits.StatorCurrentLimit = 120.0;

		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.CurrentLimits.SupplyCurrentLimit = 60.0;
		config.CurrentLimits.SupplyCurrentLowerLimit = 60.0;
		config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		config.Voltage.PeakForwardVoltage = 12.0;
		config.Voltage.PeakReverseVoltage = -12.0;

		config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		config.Feedback.SensorToMechanismRatio = indexGearing;

		return config;
	}

    public static TalonFXConfiguration getIntakeConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimitEnable = true;
		config.CurrentLimits.StatorCurrentLimit = 120.0;

		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.CurrentLimits.SupplyCurrentLimit = 60.0;
		config.CurrentLimits.SupplyCurrentLowerLimit = 60.0;
		config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		config.Voltage.PeakForwardVoltage = 12.0;
		config.Voltage.PeakReverseVoltage = -12.0;

		config.Feedback.SensorToMechanismRatio = intakeGearing;

		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        return config;
    }
}