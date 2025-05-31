package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class EndEffectorConstants {
    /* IDs */
    public static final int motorID = 20;
    /* CANbus */
    public static final String canBus = "rio";
    /* Motor Config Values */
    public static final double peakForwardVoltage = 12.0; 
    public static final double peakReverseVoltage = -12.0; 
    public static final InvertedValue motorOutputInverted = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue motorNeutralValue = NeutralModeValue.Brake;
    /* Motor Control Values */
    public static final double feedVoltage = -6.5;
    public static final double unjamVoltage = 3.0;
    public static final double advanceVoltage = -1.9;
    public static final double scoreL1Voltage = -3.5;
    public static final double scoreL2L3Voltage = -6.0; // was -9.0 for standoff
    public static final double scoreL4Voltage = -6.0;
    public static final double algaeVoltage = 3.3;
    public static final double algaeHoldVoltage = 1.2;
    public static final double algaeBargeVoltage = -12.0;
    public static final double algaeOutVoltage = -4.0;
    public static final double minimumAlgaeAcquireCurrent = 80.0;
    public static final double minimumAlgaeHoldCurrent = 60.0;

    public static final double L1RunTime = 1.5;
    public static final double algaeRunTime = 0.5;
}
