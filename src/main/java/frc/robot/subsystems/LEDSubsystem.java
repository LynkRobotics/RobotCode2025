// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
// import com.ctre.phoenix.led.LarsonAnimation;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANdleGroup;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
    /** Creates a new LEDSubsystem. */
    private static CANdleGroup leds;
    private static final Timer tempStateTimer = new Timer();
    private static final Timer blinkTimer = new Timer();

    private static LEDState state = LEDState.STARTUP;
    private static LEDState lastState = null;
    private static boolean blinkOff = false;

    private enum Color {
        off(0, 0, 0),
        red(255, 0, 0),
        green(0, 255, 0),
        hightideTeal(44, 162, 165),
        blue(0, 0, 255),
        cheesyBlue(0, 112, 255),
        cyan(0, 255, 255),
        magenta(255, 0, 255),
        yellow(255, 255, 0),
        white(255, 255, 255),
        dim(50, 50, 50),
        lynk(255, 64, 0),
        disabled(200, 0, 0);

        public final int r, g, b;

        Color(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }

    private static class LEDConfig {
        Animation animation = null;
        Color color = null;
        boolean blink = false;
        double pct = 1.0;

        LEDConfig(Color color) {
            this.color = color;
        }

        LEDConfig(Color color, boolean blink) {
            this.color = color;
            this.blink = true;
        }

        LEDConfig(Color color, double pct) {
            this.color = color;
            this.pct = pct;
        }

        LEDConfig(Animation animation) {
            this.animation = animation;
        }
    }

    public enum LEDState {
        STARTUP(new LEDConfig(Constants.LEDs.fireAnimation)),
        DISABLED(new LEDConfig(Color.disabled)),
        NORMAL(new LEDConfig(Color.lynk)),
        CORAL_L1(new LEDConfig(Color.white, 0.25)),
        CORAL_L2(new LEDConfig(Color.white, 0.50)),
        CORAL_L3(new LEDConfig(Color.white, 0.75)),
        CORAL_L4(new LEDConfig(Color.white, 1.00)),
        CORAL_UNKNOWN(new LEDConfig(Color.dim)),
        ALGAE_INTAKING(new LEDConfig(Color.cheesyBlue, true)),
        ALGAE(new LEDConfig(Color.cheesyBlue)),
        ERROR(new LEDConfig(Color.red, true)),
        WARNING(new LEDConfig(Color.yellow, true));

        public final LEDConfig config;

        LEDState(LEDConfig config) {
            this.config = config;
        } 
    }

    public LEDSubsystem() {
        // Control both CANdles in the same way
        leds = new CANdleGroup(
                new CANdle(Constants.LEDs.leftCandle, Constants.LEDs.canBus),
                new CANdle(Constants.LEDs.rightCandle, Constants.LEDs.canBus));

        // Set color on the CANdles themselves
        setColor(Color.lynk, 0, Constants.LEDs.startIdx);
    }

    private static void setColor(Color color, int startIdx, int count) {
        leds.setLEDs(color.r, color.g, color.b, 255, startIdx, count);
    }

    private static void setColor(Color color, int count) {
        setColor(color, Constants.LEDs.startIdx, count);
    }

    private static void setColor(Color color) {
        setColor(color, Constants.LEDs.numLEDs);
    }

    private static void setColor(Color color, double pct) {
        int ledCount = (int)Math.round(Constants.LEDs.numLEDs * pct);

        setColor(Color.off, Constants.LEDs.startIdx + ledCount, Constants.LEDs.numLEDs - ledCount);
        setColor(color, ledCount);
    }

    // private static void setLarson(Color color, int count) {
    //     leds.animate(new LarsonAnimation(color.r, color.g, color.b, 255, 0.5, count, LarsonAnimation.BounceMode.Front,
    //             7, Constants.LEDs.startIdx));
    // }

    public static void triggerError() {
        state = LEDState.ERROR;
        tempStateTimer.restart();
    }

    public static void triggerWarning() {
        state = LEDState.WARNING;
        tempStateTimer.restart();
    }

    @Override
    public void periodic() {
        // Determine the proper LED state
        if (DriverStation.isDisabled()) {
            if (state != LEDState.STARTUP && state != LEDState.DISABLED) {
                state = LEDState.DISABLED;
            }
        } else {
            // Check if the temporary state should be cleared
            if (tempStateTimer.isRunning() && tempStateTimer.hasElapsed(Constants.LEDs.tempStateTime)) {
                tempStateTimer.stop();
            }
            // Compute the proper state if's not temporarily overridden
            if (!tempStateTimer.isRunning()) {
                if (RobotState.haveAlgae()) {
                    state = LEDState.ALGAE;
                } else if (RobotState.intakingAlgae()) {
                    state = LEDState.ALGAE_INTAKING;
                } else if (RobotState.haveCoral()) {
                    switch (RobotState.getNextStop()) {
                        case L1:
                            state = LEDState.CORAL_L1;
                            break;
                        case L2:
                            state = LEDState.CORAL_L2;
                            break;
                        case L3:
                            state = LEDState.CORAL_L3;
                            break;
                        case L4:
                        case L4_SCORE:
                            state = LEDState.CORAL_L4;
                            break;
                        default:
                            state = LEDState.CORAL_UNKNOWN;
                            break;
                    }
                } else {
                    state = LEDState.NORMAL;
                }        
            }
        }

        // Change the LEDs if the state has changed
        if (state != lastState) {
            if (lastState != null && lastState.config.animation != null) {
                leds.clearAnimation();
            }
            if (state.config.animation != null) {
                leds.animate(state.config.animation);
            } else {
                if (state.config.pct == 1.0) {
                    setColor(state.config.color);
                } else {
                    setColor(state.config.color, state.config.pct);
                }
            }

            if (state.config.blink) {
                blinkOff = false;
                blinkTimer.restart();
            } else {
                blinkTimer.stop();
            }
        }

        // Blink the LEDs if the blink interval has elapsed
        if (blinkTimer.isRunning() && blinkTimer.advanceIfElapsed((state == LEDState.ERROR || state == LEDState.WARNING) ? Constants.LEDs.errorBlinkRate : Constants.LEDs.blinkRate)) {
            blinkOff = !blinkOff;
            setColor(blinkOff ? Color.off : state.config.color);
        }

        DogLog.log("LED/State", state.toString());
        lastState = state;
    }
}