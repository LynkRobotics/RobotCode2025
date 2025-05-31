package frc.robot.subsystems.climber;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberConstants {
    /* IDs */
    public static final int motorID = 46;
    /* CANbus */
    public static final String canBus = "rio";
    /* Motor Config Values */
    public static final double peakForwardVoltage = 12.0; 
    public static final double peakReverseVoltage = -12.0; 
    public static final InvertedValue motorOutputInverted = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue motorNeutralValue = NeutralModeValue.Brake;
    /* Motor Control Values */
    public static final double fastDeployVoltage = 12.0;
    public static final double slowDeployVoltage = 3.0;
    public static final double engageRetractVoltage = -4.0;
    public static final double fastRetractVoltage = -12.0;
    public static final double slowRetractVoltage = -3.0;
    public static final double holdVoltage = -0.5;
    public static final double fastDeployedPosition = 110.0;
    public static final double fullyDeployedPosition = 135.5;
    public static final double engageRetractedPosition = 120.0;
    public static final double pausePosition = 17.5;
    public static final double pauseDuration = 0.25;
    public static final double fastRetractedPosition = -15.0;
    public static final double fullyRetractedPosition = -30.5;
    public static final int timeCutoff = 30;
}
