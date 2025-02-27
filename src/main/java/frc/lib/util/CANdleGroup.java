package frc.lib.util;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;

public class CANdleGroup {
    private CANdle[] candles;
    
    public CANdleGroup(CANdle... candles) {
        this.candles = candles;
        applyConfigs();
    }

    private void applyConfigs() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.brightnessScalar = 1.0;
        config.stripType = LEDStripType.GRB;
        config.v5Enabled = true;
        config.disableWhenLOS = false; //TODO: verify // TODO: true -- why is this triggering?

        for (CANdle candle : candles) {
            candle.configAllSettings(config);
        }
    }

    public void animate(Animation animation) {
        clearAnimation();
        for (CANdle candle : candles) {
            candle.animate(animation);
        }
    }

    public void clearAnimation() {
        for (CANdle candle : candles) {
            candle.clearAnimation(0);
        }
    }

    public void setLEDs(int r, int g, int b) {
        clearAnimation();
        for (CANdle candle : candles) {
            candle.setLEDs(r, g, b);
        }
    }

    public void setLEDs(int r, int g, int b, int w, int startIdx, int count) {
        clearAnimation();
        for (CANdle candle : candles) {
            candle.setLEDs(r, g, b, w, startIdx, count);
        }
    }
}