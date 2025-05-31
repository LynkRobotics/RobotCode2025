package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import frc.robot.Constants;

import com.ctre.phoenix.led.RainbowAnimation;

public class LEDConstants {
    /* IDs */
    public static final int leftCandle = 0;
    public static final int rightCandle = 1;
    /* CANbus */
    public static final String canBus = "rio";
    /* LED arrangement */
    public static final int startIdx = 8;
    public static final int numLEDs = 86;
    public static final int totalLEDs = startIdx + numLEDs;
    public static final double brightness = Constants.atHQ ? 0.60 : 1.00;
    /* Animations */
    public static final Animation readyAnimation = new FireAnimation(1.0, 0.38, numLEDs, 0.8, 0.2, false, startIdx);
    public static final Animation climbedAnimation = new RainbowAnimation(1.0, 0.7, numLEDs, false, startIdx);
    public static final Animation climbingAnimation = new LarsonAnimation(255, 64, 0, 0, 0.85, numLEDs, BounceMode.Front, 7, startIdx);
    public static final Animation endGameAnimation = new ColorFlowAnimation(255, 64, 0, 0, 0.85, numLEDs, ColorFlowAnimation.Direction.Forward, startIdx);
    public static final Animation serviceModeAnimation = new ColorFlowAnimation(0, 25, 25, 0, 0.3, numLEDs, ColorFlowAnimation.Direction.Backward, startIdx);
    /* Misc */
    public static final double blinkRate = 0.2; // Regular blink rate
    public static final double errorBlinkRate = 0.1; // Blink rate for errors and warnings
    public static final double tempStateTime = 0.70; // How long for warnings and errors
    public static final double endGameNotifyStart = 20.0;
    public static final double endGameNotifyDuration = 4.0;    
}
