package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class ElevatorConstants {
    /* IDs */
    public static final int leftID = 4;
    public static final int rightID = 17;
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
    public static final double baseHeight = 12.0 + bellyHeight; // Height of elevator in inches when it is at zero position
    public static final double maxHeight = 72.0 + bellyHeight; // Height that elevator should never exceed
    public static final double endEffectorHeight = 6.0; // Height of end effector "target" above elevator height
    public static final double rotPerInch = 0.704; // Rotations to drive elevator one inch

    public static final double safetyMargin = 1.5;   // How many inches away from safe mark to still be considered safe
    public static final double positionError = rotPerInch * 0.5; // Allowable rotation error to be considered in position
    public static final double positionCloseError = rotPerInch * 6.0; // Allowable rotation error to be considered in position
    public static final double stopError = 0.25;      // Allowable inches of error to be considered at a stop
    public static final double slowVoltage = 2.0;    // Volts to move slowly to zero
    public static final double towardsMargin = 32.0; // inches

    public static final double RPSperVolt = 7.9; // RPS increase with every volt
    public static final double kP = 2.2; // output per unit of error in position (output/rotation)
    public static final double kI = 0.0; // output per unit of integrated error in position (output/(rotation*s))
    public static final double kD = 0.0; // output per unit of error in velocity (output/rps)
    public static final double kS = 0.0; // output to overcome static friction (output)
    public static final double kV = 1.0 / RPSperVolt; // output per unit of target velocity (output/rps)
    public static final double kA = 0.0; // output per unit of target acceleration (output/(rps/s))
    public static final double kG = 0.4; // output to overcome gravity
    public static final double cruiseVelocity = 100.0; // RPS
    public static final double acceleration = cruiseVelocity / 0.3; // Accelerate in 0.3 seconds

    public static final double speedLimitAtMax = 0.30;

    public enum Stop {
        // Intake occurs at zero
        SAFE     (ElevatorConstants.baseHeight + 5.0),
        L1       (21.3 - ElevatorConstants.endEffectorHeight),
        L1_SCORE (33.0 - ElevatorConstants.endEffectorHeight),
        HOLD     (30.0 - ElevatorConstants.endEffectorHeight),
        L2       (34.5 - ElevatorConstants.endEffectorHeight),
        L2_ALGAE (38.0 - ElevatorConstants.endEffectorHeight), 
        L3       (49.5 - ElevatorConstants.endEffectorHeight),
        L3_ALGAE (53.5 - ElevatorConstants.endEffectorHeight),
        ALGAE_RELEASE(63.5 - ElevatorConstants.endEffectorHeight),
        L4       (74.5 - ElevatorConstants.endEffectorHeight),
        L4_SCORE (77.0 - ElevatorConstants.endEffectorHeight);

        Stop(double height) {
            this.height = height;
        }

        public final double height;
    }

    public static final double L1RaiseDelay = 0.3;
    // public static final double standoffBoost = 1.5; // In inches
}
