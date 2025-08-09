package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Robot;


public class ElevatorConstants {
    /* IDs */
    public static final int mainID = 14;
    public static final int followerID = 15;
    /* CANBus */
    public static final String canBus = "rio";
    /* Motor Config Values */
    public static final double peakForwardVoltage = 14;
    public static final double peakReverseVoltage = -14;
    public static final InvertedValue motorOutputInverted = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue motorNeutralValue = NeutralModeValue.Brake;

    // NOTE Elevator height is measured from the ground to top of the carriage
    public static final double thickness = 2.0; // Thickness of the elevator (only for Mechanism2d visualization)
    public static final double setback = 9.5; // Distance from front edge of robot (only for Mechanism2d visualization)
    public static final double bellyHeight = 0.755; // Height of the top surface of the belly pan from the ground
    public static final Distance baseHeight = Units.Inches.of(12.0 + bellyHeight); // TODO // Height of elevator in inches when it is at zero position
    public static final Distance maxHeight = Units.Inches.of(63.0); // TODO 72.0 + bellyHeight; // Height that elevator should never exceed
    public static final double endEffectorHeight = 6.0; // Height of end effector "target" above elevator height
    public static final double rotPerInch = 0.704; // Rotations to drive elevator one inch
    // public static final PerUnit<AngleUnit, DistanceUnit> rotperInch = PerUnit.combine(Units.Rotations.of(0.704), Units.Inches.of(1.0));
    // TODO Reevaluate these values

    public static final Distance safetyMargin = Units.Inches.of(1.5);   // How many inches away from safe mark to still be considered safe
    public static final double positionError = rotPerInch * 0.5; // Allowable rotation error to be considered in position
    public static final double positionCloseError = rotPerInch * 6.0; // Allowable rotation error to be considered in position
    public static final double stopError = 0.25;      // Allowable inches of error to be considered at a stop
    public static final double slowVoltage = 2.0;    // Volts to move slowly to zero
    public static final Distance towardsMargin = Units.Inches.of(32.0);

    public static final double speedLimitAtMax = 0.30;

    private static final Distance algaeLiftDistance = Units.Inches.of(2.0);

    public enum Stop {
        SAFE(Units.Inches.of(2.0)), // TODO Re-evaluate this position
        L1(Units.Inches.of(0.0)),
        L2(Units.Inches.of(6.3)),
        L3(L2.height.plus(Units.Inches.of(16.0))),
        L4(Units.Inches.of(60.25)),
        CORAL_HOLD(Units.Inches.of(13.21)),
        L2_ALGAE(Units.Inches.of(21.91)),
        L3_ALGAE(L2_ALGAE.height.plus(Units.Inches.of(16.0))),
        L2_ALGAELIFT(L2_ALGAE.height.plus(algaeLiftDistance)),
        L3_ALGAELIFT(L3_ALGAE.height.plus(algaeLiftDistance)),
        ALGAE_HOLD(Units.Inches.of(19.41)),
        BARGE(Units.Inches.of(62.17));

        Stop(Distance height) {
            this.height = height;
            this.position = 0.0; // TODO Conversion
        }

        public final Distance height;
        public final double position;
    }

    // public static final double L1RaiseDelay = 0.3;
    // public static final double standoffBoost = 1.5; // In inches

    public static final double gearing = (3.0 / 1.0);

	public static final TalonFXConfiguration getMotorConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();

		config.Slot0.kP = 16.3;
		config.Slot0.kD = 0.5;
		config.Slot0.kS = 0.45;
		config.Slot0.kG = 0.35;

		config.MotionMagic.MotionMagicCruiseVelocity = 20.0;

		config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.SupplyCurrentLimit = 80.0;
		config.CurrentLimits.SupplyCurrentLowerLimit = 80.0;
		config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		config.CurrentLimits.StatorCurrentLimitEnable = true;
		config.CurrentLimits.StatorCurrentLimit = 120.0;

		config.Voltage.PeakForwardVoltage = 12.0;
		config.Voltage.PeakReverseVoltage = -12.0;

		// FXConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		// FXConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
		// 		converter.toAngle(kNetHeight).in(Units.Rotations);

		// FXConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		// FXConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
		// 		converter.toAngle(kStowPosition).minus(Units.Degrees.of(10.0)).in(Units.Rotations);

		config.Feedback.SensorToMechanismRatio = gearing;

		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		return config;
	}
}