package frc.robot.subsystems.algaeroller;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class AlgaeRollerContants {
    /* IDs */
    public static final int deployMotorID = 8;
    public static final int rollerMotorID = 9;

    /* CANbus */
    public static final String canBus = "rio";

    /* Motor Config Values */
    public static final double peakForwardVoltage = 12.0; 
    public static final double peakReverseVoltage = -12.0; 
    public static final InvertedValue motorOutputInverted = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue motorNeutralValue = NeutralModeValue.Brake;

    /* Motor Control Values */
    // public static final double intakeVoltage = 0.0;
    // public static final double coralAssistVoltage = 0.0;

    public static double retractedPosition = 0.0;
    public static double deployedPosition = 0.0;
}
