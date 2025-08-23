package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public class ClimberConstants {
    /* Motor Config Values */
    public static final double intakeGearing = (18.0 / 12.0);
    public static final double deployGearing = (36.0 / 1.0);
    public static final Voltage intakeVoltage = Units.Volts.of(8.0);
    public static final Voltage resetVoltage = Units.Volts.of(-2.0);
	public static final Current currentStallThreshold = Units.Amps.of(20.0);
	public static final Time stallPeriod = Units.Seconds.of(0.7);
	private static final Angle maxExtension = Units.Degrees.of(1600);

    public static final Distance drumDiameter = Units.Inches.of(0.675);

	public static enum ClimberPosition {
		FULLY_STOWED(9.00),
		STOWED(10.948),
		DEPLOYED(19.8),
		CLEAR(STOWED.distance.plus(Units.Inches.of(0.5)).in(Units.Inches)),
		EPISILON(1.5);

		public final Distance distance;
		public final Angle angle;
		public final ControlRequest control;

		private ClimberPosition(double inches) {
			distance = Units.Inches.of(inches);
			angle = Units.Radians.of(distance.div(drumDiameter.div(2.0)).baseUnitMagnitude());
			control = new MotionMagicExpoVoltage(angle).withEnableFOC(true);
		}
	}

	public static TalonFXConfiguration getDeployMotorConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = 300.0;
		config.Slot0.kD = 0.0;
		config.Slot0.kS = 0.0;
		config.Slot0.kG = 0.0;
		config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

		config.MotionMagic.MotionMagicAcceleration = 1000;
		config.MotionMagic.MotionMagicCruiseVelocity = 500;
		config.MotionMagic.MotionMagicJerk = 100;

		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.CurrentLimits.SupplyCurrentLimit = 80.0;
		config.CurrentLimits.SupplyCurrentLowerLimit = -80.0;
		config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		config.Voltage.PeakForwardVoltage = 12.0;
		config.Voltage.PeakReverseVoltage = -12.0;

		config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
			ClimberPosition.STOWED.angle.plus(maxExtension).in(Units.Rotations);

		// Cannot reset climber with this
		// config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		// config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
		// 	ClimberPosition.FULLY_STOWED.angle.in(Units.Rotations);

		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		config.Feedback.SensorToMechanismRatio = deployGearing;
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        return config;
    }

    public static TalonFXConfiguration getIntakeMotorConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();

		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.CurrentLimits.SupplyCurrentLimit = 40.0;
		config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
		config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		config.CurrentLimits.StatorCurrentLimitEnable = true;
		config.CurrentLimits.StatorCurrentLimit = 80.0;

		config.Voltage.PeakForwardVoltage = 12.0;
		config.Voltage.PeakReverseVoltage = -12.0;

		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		config.Feedback.SensorToMechanismRatio = intakeGearing;
        
        return config;
    }
}