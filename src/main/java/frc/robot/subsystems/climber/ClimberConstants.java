package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class ClimberConstants {
    /* Motor Config Values */
    public static final double intakeGearing = (18.0 / 12.0);
    public static final double deployGearing = (36.0 / 1.0);
    public static final Voltage intakeVoltage = Units.Volts.of(8.0);
	public static final AngularVelocity velocityThreshold = Units.RPM.of(2900.0);
    // TODO Experiment with current and/or velocity ourselves

    public static final double deployedPosition = 0.0;
    public static final double retractedPosition = 0.0;

    public static TalonFXConfiguration getDeployMotorConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();

        // TODO How much of this is used?

        config.Slot0.kP = 300.0;
		config.Slot0.kD = 0.0;
		config.Slot0.kS = 0.0;
		config.Slot0.kG = 0.0;
		config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

		// TODO
		// config.MotionMagic.MotionMagicAcceleration = 1000;
		// config.MotionMagic.MotionMagicCruiseVelocity = 500;
		config.MotionMagic.MotionMagicAcceleration = 50;
		config.MotionMagic.MotionMagicCruiseVelocity = 25;
		config.MotionMagic.MotionMagicJerk = 100;

		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.CurrentLimits.SupplyCurrentLimit = 80.0;
		config.CurrentLimits.SupplyCurrentLowerLimit = -80.0;
		config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		// TODO
		// config.Voltage.PeakForwardVoltage = 12.0;
		// config.Voltage.PeakReverseVoltage = -12.0;
		config.Voltage.PeakForwardVoltage = 2.0;
		config.Voltage.PeakReverseVoltage = -2.0;

		// config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		// config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
		// 		converter.toAngle(kStowPosition).plus(kMaxExtension).in(Units.Rotations);

		// config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		// config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
		// 		converter.toAngle(kPullPosition).in(Units.Rotations);

		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		config.Feedback.SensorToMechanismRatio = deployGearing;
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        return config;
    }

    public static TalonFXConfiguration getIntakeMotorConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();

        // TODO Is this even used? 

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

		// config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		// config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
		// 		converter.toAngle(kStowPosition).plus(kMaxExtension).in(Units.Rotations);

		// config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		// config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
		// 		converter.toAngle(kPullPosition).in(Units.Rotations);

		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		config.Feedback.SensorToMechanismRatio = intakeGearing;
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        return config;
    }
}