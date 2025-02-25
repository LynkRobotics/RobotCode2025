package frc.lib.util;

import java.net.http.WebSocket;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;

public class LED {
    public int candleNumber;

    private CANdle mCandle;
    
    public LED(int candleNumber) {
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

    public void setAnimation(Animation animation) {
        mCandle.animate(animation);
    }

    public void clearAnimation() {
        mCandle.clearAnimation(0);
    }

    public void setLEDs(int r, int g, int b) {
        mCandle.setLEDs(r, g, b);
    }

    public void setLEDs(int r, int g, int b, int w, int startIdx, int count) {
        mCandle.setLEDs(r, g, b, w, startIdx, count);
    }

}
