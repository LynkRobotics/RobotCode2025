package frc.lib.util;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;

public class LynkCandle {
    public int candleNumber;

    private CANdle mCandle;
    
    public LynkCandle(int candleNumber) {
        this.candleNumber = candleNumber;
        
        /* CANdle Config */
        mCandle = new CANdle(candleNumber);
        applyConfigs();
    }

    private void applyConfigs() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.brightnessScalar = 1.0;
        config.stripType = LEDStripType.GRB;
        config.v5Enabled = true;
        config.disableWhenLOS = false; //TODO: verify // TODO: true -- why is this triggering?

        mCandle.configAllSettings(config);
    }

    public void setLynkAnimation(Animation animation) {
        mCandle.animate(animation);
    }

    public void clearLynkAnimation() {
        mCandle.clearAnimation(0);
    }

    public void setLynkLEDs(int r, int g, int b) {
        mCandle.setLEDs(r, g, b);
    }

}
