package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Robot;

public class EndEffectorConstants {
    /* Motor Control Values */
    private static final double coralRollerGearing = (7.5 / 1.0);
	// UNUSED private static final double algaeRollerGearing = (6.0 / 1.0);
    private static final double positionGearing = (48.0 / 10.0) * (64.0 / 18.0) * (48.0 / 18.0);
	public static final double gearedCancoderGearing = 41.0 / 40.0; // Not 39.0 like 1678, due to CAD...

	public static final Current algaeStatorCurrentThreshold = Units.Amps.of(70.0);
	public static final Current coralStatorCurrentThreshold = Units.Amps.of(60.0);
	public static final Angle pivotEpsilon = Units.Degrees.of(2.0);

	public static final Time coralSensorDebounce = Units.Seconds.of(0.04);
	public static final Time algaeSensorDebounce = Units.Seconds.of(0.09);

	public static enum EEControl {
		CORAL_INTAKE(6.0),
		CORAL_HOLD(1.0),
		CORAL_L1(-1.0),
		CORAL_L2(-3.0),
		CORAL_L3(-3.0),
		CORAL_L4(-10.0),
		ALGAE_INTAKE(12.0),
		ALGAE_HOLD(1.0),
		ALGAE_BARGE_SCORE(-9.0),
		ALGAE_PROCESSOR_SCORE(-3.0),
		SPIT(-3.0);  // What is this?

		private Voltage voltage;
		public ControlRequest control;

		EEControl(double voltage) {
			this.voltage = Units.Volts.of(voltage);
			this.control = new VoltageOut(this.voltage).withEnableFOC(false);
		}
	}

    public static enum EEPosition {
        L1(102.0),
        L23(100.75),
        L4(140.0),
        BARGE(-218.0),
        START(90.0),
        GROUND_INTAKE(-90.0),
        REEF_INTAKE(-105.0),
        REEF_PREP(-60.0),
        IDLE_AFTER_SCORING(-45.0),
        ALGAE_HOLD(-120.0),
        CORAL_HOLD(60.0),
        CLIMB(130.0),
		HIGH_CLEAR_START(50.0),
		HIGH_CLEAR_END(105.0);

        public final Angle position;
		public final ControlRequest control;

        EEPosition(double degrees) {
            position = Units.Degrees.of(degrees);
			control = new MotionMagicExpoVoltage(position).withEnableFOC(true);
        }
    }

    public static final TalonFXConfiguration getPieceConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();

		config.Voltage.PeakForwardVoltage = 12.0;
		config.Voltage.PeakReverseVoltage = -12.0;

		config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.StatorCurrentLimit = 80.0;

		config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.SupplyCurrentLimit = 60.0;
		config.CurrentLimits.SupplyCurrentLowerLimit = 60.0;
		config.CurrentLimits.SupplyCurrentLowerTime = 1.0;

		config.Feedback.SensorToMechanismRatio = coralRollerGearing;

		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		return config;
	}

    public static final TalonFXConfiguration getPositionConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = 75.0;
		config.Slot0.kD = 2.5;
		config.Slot0.kS = 0.05;

		config.Slot0.kG = 0.2;

		config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

		config.MotionMagic.MotionMagicCruiseVelocity = 2.0;
		config.MotionMagic.MotionMagicAcceleration = 1.0;

		config.Voltage.PeakForwardVoltage = 12.0;
		config.Voltage.PeakReverseVoltage = -12.0;

		config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.StatorCurrentLimit = 60.0;

		config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.SupplyCurrentLimit = 30.0;
		config.CurrentLimits.SupplyCurrentLowerLimit = 30.0;
		config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
				Units.Rotations.of(999.0).in(Units.Rotations);

		config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
				Units.Rotations.of(-999.0).in(Units.Rotations);
		
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		config.Feedback.SensorToMechanismRatio = positionGearing;

        return config;
    }

	public static final CANdiConfiguration getCANdiConfig() {
		CANdiConfiguration config = new CANdiConfiguration();

        config.DigitalInputs.S1CloseState = S1CloseStateValue.CloseWhenNotHigh;
        config.DigitalInputs.S2CloseState = S2CloseStateValue.CloseWhenNotHigh;

		return config;
	}

	public static CANcoderConfiguration getGeared40TCancoderConfig() {
		CANcoderConfiguration config = new CANcoderConfiguration();
		config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
		config.MagnetSensor.MagnetOffset = 0.22412109375;
		config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
		return config;
	}

	public static CANcoderConfiguration getDirect41TCancoderConfig() {
		CANcoderConfiguration config = new CANcoderConfiguration();
		config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
		config.MagnetSensor.MagnetOffset = 0.1650390625;
		config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
		return config;
	}
}