package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public class IntakeConstants {
    public enum IntakePosition {
        FULLY_DEPLOYED(3.0),
        DEPLOYED(15.0),
        RETRACTED(55.0),
        FULL_STOW(83.0);

        public final Angle position;
        public final ControlRequest control;

        IntakePosition(double degrees) {
            position = Units.Degrees.of(degrees);
            control = new MotionMagicExpoVoltage(position).withEnableFOC(true);
        }
    }

	public static final double intakeSlowMode = 0.8; // How much to slow speed when intaking

    public static final double deployGearing = 40.0;
    public static final double indexGearing = 2.5;
    public static final double intakeGearing = (24.0 / 12.0);

	public static final Voltage deployZeroingVoltage = Units.Volts.of(-1.0);
	public static final Current deployStallCurrent = Units.Amps.of(-35.0);
	public static final Time deployStallTime = Units.Seconds.of(0.2);

	public static final Voltage indexVoltage = Units.Volts.of(10.0);
	public static final Voltage indexExpelVoltage = Units.Volts.of(-8.0);
	public static final Time expelTime = Units.Seconds.of(0.5);

	public static final Voltage intakeVoltage = Units.Volts.of(-12.0);
	public static final Voltage intakeExpelVoltage = Units.Volts.of(12.0);

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

		config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakePosition.FULL_STOW.position.in(Units.Rotations);

		config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakePosition.DEPLOYED.position.in(Units.Rotations);

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