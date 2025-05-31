package frc.robot.subsystems.index;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IndexConstants {
    /* IDs */
    public static final int motorID = 13;
    /* CANbus */
    public static final String canBus = "rio";
    /* Motor Config Values */
    public static final double peakForwardVoltage = 12.0; 
    public static final double peakReverseVoltage = -12.0; 
    public static final InvertedValue motorOutputInverted = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue motorNeutralValue = NeutralModeValue.Brake;
    /* Motor Control Values */
    public static final double intakeVoltage = -2.00;
    public static final double unjamVoltage = 1.00;
    public static final double rejectVoltage = 0.50;
        
    public static final double minIntakeVelocity = 12.0;
    public static final int maxStallCount = 15;
    public static final double unjamTime = 0.4;
}
