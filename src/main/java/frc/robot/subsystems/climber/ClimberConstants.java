package frc.robot.subsystems.climber;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberConstants {
    /* IDs */
    public static final int deployMotorID = 17;
    public static final int intakeMotorID = 18;
    /* CANbus */
    public static final String canBus = "rio";
    /* Motor Config Values */
    public static final double peakForwardVoltage = 12.0; 
    public static final double peakReverseVoltage = -12.0; 
    public static final InvertedValue motorOutputInverted = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue motorNeutralValue = NeutralModeValue.Brake;
    /* Motor Control Values */
}
