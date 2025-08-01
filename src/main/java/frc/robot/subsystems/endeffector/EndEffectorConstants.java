package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class EndEffectorConstants {
    /* IDs */
    public static final int positionMotorID = 20;
    public static final int pieceMotorID = 21;

    /* CANbus */
    public static final String canBus = "rio";

    /* Motor Config Values */
    public static final double peakForwardVoltage = 12.0; 
    public static final double peakReverseVoltage = -12.0; 
    public static final InvertedValue motorOutputInverted = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue motorNeutralValue = NeutralModeValue.Brake;

    /* Motor Control Values */
    // public static final double algaeOutVoltage = -4.0;
}
