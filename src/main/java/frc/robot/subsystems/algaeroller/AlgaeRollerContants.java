package frc.robot.subsystems.algaeroller;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;

public class AlgaeRollerContants {
    /* IDs */
    public static final int deployMotorID = 8;
    public static final int rollerMotorID = 9;

    /* CANbus */
    public static final String canBus = "rio";

    /* Motor Control Values */
    public static final double deployGearing = (62.0 / 8.0) * (68.0 / 18.0) * (15.0 / 9.0);
	public static final double rollerGearing = (18.0 / 12.0);
	public static final Voltage intakeVoltage = Units.Volts.of(8.0);
	public static final Voltage expelVoltage = Units.Volts.of(-5.0);
	public static final Voltage L1AssistVoltage = Units.Volts.of(3.5);

	// TODO Determine values
    public static double retractedPosition = 0.0;
	public static double clearPosition = 0.0;
    public static double deployedPosition = 0.0;

    public static TalonFXConfiguration getDeployMotorConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();

		config.Slot0.kP = 115.0;
		config.Slot0.kS = 0.0;
		config.Slot0.kG = 0.2;

		config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

		// TODO config.MotionMagic.MotionMagicCruiseVelocity = 100.0;
		config.MotionMagic.MotionMagicCruiseVelocity = 5.0;
		config.MotionMagic.MotionMagicAcceleration = 80.0;

		// TODO
		// config.Voltage.PeakForwardVoltage = 12.0;
		// config.Voltage.PeakReverseVoltage = -12.0;
		config.Voltage.PeakForwardVoltage = 2.0;
		config.Voltage.PeakReverseVoltage = -2.0;

		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.CurrentLimits.SupplyCurrentLimit = 40.0;
		config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
		config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		config.Feedback.SensorToMechanismRatio = deployGearing;

		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		// config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		// config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kStowPosition.in(Units.Rotations);

		// config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		// config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -1000;

		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        return config;
    }

    public static TalonFXConfiguration getRollerMotorConfig() {
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

		config.Feedback.SensorToMechanismRatio = rollerGearing;

		return config;
	}
}
