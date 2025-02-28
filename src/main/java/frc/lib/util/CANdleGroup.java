package frc.lib.util;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import frc.robot.Constants;

import com.ctre.phoenix.led.CANdleConfiguration;

import dev.doglog.DogLog;

public class CANdleGroup {
    private CANdle[] candles;
    
    public CANdleGroup(CANdle... candles) {
        this.candles = candles;
        applyConfigs();
    }

    private void applyConfigs() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.brightnessScalar = Constants.LEDs.brightness;
        config.stripType = LEDStripType.GRB;
        config.v5Enabled = true;
        config.disableWhenLOS = false; //TODO: verify // TODO: true -- why is this triggering?

        for (CANdle candle : candles) {
            candle.configAllSettings(config);
        }
    }

    public void animate(Animation animation) {
        DogLog.log("LED/Status", "Setting animation");
        for (CANdle candle : candles) {
            candle.animate(animation);
        }
    }

    public void clearAnimation() {
        DogLog.log("LED/Status", "Clearing animation");
        for (CANdle candle : candles) {
            candle.clearAnimation(0);
        }
    }

    public void setLEDs(int r, int g, int b) {
        DogLog.log("LED/Status", "Set LEDs to (" + r + "," + g + "," + b + ")");
        for (CANdle candle : candles) {
            candle.setLEDs(r, g, b);
        }
    }

    public void setLEDs(int r, int g, int b, int w, int startIdx, int count) {
        DogLog.log("LED/Status", "Set " + count + " LEDs at " + startIdx + " to (" + r + "," + g + "," + b + ")");
        for (CANdle candle : candles) {
            candle.setLEDs(r, g, b, w, startIdx, count);
        }
    }
}