// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import static frc.robot.Options.optAutoReefAiming;
import static frc.robot.Options.optL1Outside;
import static frc.robot.Options.optServiceMode;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
// import com.ctre.phoenix.led.LarsonAnimation;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANdleGroup;
import frc.robot.subsystems.robotstate.RobotState;
import frc.robot.subsystems.robotstate.RobotState.ClimbState;

public class LEDSubsystem extends SubsystemBase {
    /** Creates a new LEDSubsystem. */
    private static CANdleGroup leds;
    private static final Timer tempStateTimer = new Timer();
    private static final Timer blinkTimer = new Timer();
    private static final Timer endGameTimer = new Timer();

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
        dimYellow(150, 150, 0),
        white(250, 250, 250),
        brightWhite(255, 255, 255),
        dim(10, 10, 10),
        lynk(255, 64, 0),
        service(0, 10, 10),
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
        STARTUP(new LEDConfig(LEDConstants.readyAnimation)),
        DISABLED(new LEDConfig(Color.disabled)),
        NORMAL(new LEDConfig(Color.lynk)),
        MANUAL(new LEDConfig(Color.hightideTeal)),
        SLOWMODE(new LEDConfig(Color.dimYellow)),
        CORAL_L1(new LEDConfig(Color.magenta, 0.25)),
        CORAL_L1OUTSIDE(new LEDConfig(Color.cheesyBlue, 0.25)),
        CORAL_L2(new LEDConfig(Color.green, 0.50)),
        CORAL_L3(new LEDConfig(Color.magenta, 0.75)),
        CORAL_L4(new LEDConfig(Color.green, 1.00)),
        CORAL_UNKNOWN(new LEDConfig(Color.dim)),
        ALGAE_INTAKING(new LEDConfig(Color.cheesyBlue, true)),
        ALGAE(new LEDConfig(Color.cheesyBlue)),
        ENDGAME(new LEDConfig(LEDConstants.endGameAnimation)),
        CLIMBING(new LEDConfig(LEDConstants.climbingAnimation)),
        CLIMBED(new LEDConfig(LEDConstants.climbedAnimation)),
        SERVICE_MODE(new LEDConfig(LEDConstants.serviceModeAnimation)),
        SERVICE_DISABLED(new LEDConfig(Color.service)),
        ERROR(new LEDConfig(Color.red, true)),
        WARNING(new LEDConfig(Color.yellow, true)),
        INFO(new LEDConfig(Color.white, true));

        public final LEDConfig config;

        LEDState(LEDConfig config) {
            this.config = config;
        } 
    }

    public LEDSubsystem() {
        // Control both CANdles in the same way
        leds = new CANdleGroup(
                new CANdle(LEDConstants.leftCandle, LEDConstants.canBus),
                new CANdle(LEDConstants.rightCandle, LEDConstants.canBus));

        // Set color on the CANdles themselves
        setColor(Color.lynk, 0, LEDConstants.startIdx);
    }

    private static void setColor(Color color, int startIdx, int count) {
        leds.setLEDs(color.r, color.g, color.b, 255, startIdx, count);
    }

    private static void setColor(Color color, int count) {
        setColor(color, LEDConstants.startIdx, count);
    }

    private static void setColor(Color color) {
        setColor(color, LEDConstants.numLEDs);
    }

    private static void setColor(Color color, double pct) {
        int ledCount = (int)Math.round(LEDConstants.numLEDs * pct);

        setColor(Color.off, LEDConstants.startIdx + ledCount, LEDConstants.numLEDs - ledCount);
        setColor(color, ledCount);
    }

    // private static void setLarson(Color color, int count) {
    //     leds.animate(new LarsonAnimation(color.r, color.g, color.b, 255, 0.5, count, LarsonAnimation.BounceMode.Front,
    //             7, LEDConstants.startIdx));
    // }

    public static void triggerError() {
        state = LEDState.ERROR;
        tempStateTimer.restart();
    }

    public static void triggerWarning() {
        state = LEDState.WARNING;
        tempStateTimer.restart();
    }

    public static void triggerInfo() {
        state = LEDState.INFO;
        tempStateTimer.restart();
    }

    @Override
    public void periodic() {
        // Determine the proper LED state
        if (optServiceMode.get()) {
            state = DriverStation.isDisabled() ? LEDState.SERVICE_DISABLED : LEDState.SERVICE_MODE;
        } else if (DriverStation.isDisabled()) {
            if (state != LEDState.STARTUP && state != LEDState.DISABLED) {
                DogLog.log("LED/Status", "Transitioned to DISABLED from " + state);
                state = LEDState.DISABLED;
            }
        } else {
            // Check if the temporary state should be cleared
            if (tempStateTimer.isRunning() && tempStateTimer.hasElapsed(LEDConstants.tempStateTime)) {
                tempStateTimer.stop();
            }
            // Compute the proper state if's not temporarily overridden
            if (!tempStateTimer.isRunning()) {
                ClimbState climbState = RobotState.getClimbState();
                double timeLeft = DriverStation.getMatchTime();

                if (DriverStation.isTeleopEnabled() && timeLeft < LEDConstants.endGameNotifyStart && endGameTimer.get() == 0) {
                    DogLog.log("LED/Status", "Begin end game");
                    endGameTimer.start();
                    state = LEDState.ENDGAME;
                } else if (endGameTimer.isRunning() && endGameTimer.hasElapsed(LEDConstants.endGameNotifyDuration)) {
                    DogLog.log("LED/Status", "End end game");
                    endGameTimer.stop();
                } else {
                    DogLog.log("LED/Status", "State: " + climbState + "; timeLeft = " + String.format("%1.1f", timeLeft) + "; timer = " + String.format("%1.1f", endGameTimer.get()));
                }
                if (!endGameTimer.isRunning()) {
                    if (climbState == ClimbState.CLIMBING) {
                        state = LEDState.CLIMBING;
                    } else if (climbState == ClimbState.CLIMBED) {
                        state = LEDState.CLIMBED;
                    } else if (climbState == ClimbState.STARTED) {
                        state = LEDState.SLOWMODE;
                    } else if (RobotState.haveAlgae()) {
                        state = LEDState.ALGAE;
                    } else if (RobotState.intakingAlgae()) {
                        state = LEDState.ALGAE_INTAKING;
                    } else if (RobotState.haveCoral()) {
                        switch (RobotState.getNextStop()) {
                            case L1:
                                state = optL1Outside.get() ? LEDState.CORAL_L1OUTSIDE : LEDState.CORAL_L1;
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
                        }
                    } else {
                        state = optAutoReefAiming.get() ? LEDState.NORMAL : LEDState.MANUAL;
                    }        
                }
            }
        }

        // Change the LEDs if the state has changed
        if (state != lastState) {
            if (lastState == null || lastState.config.animation != null) {
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
        if (blinkTimer.isRunning() && blinkTimer.advanceIfElapsed((state == LEDState.ERROR || state == LEDState.WARNING || state == LEDState.INFO) ? LEDConstants.errorBlinkRate : LEDConstants.blinkRate)) {
            blinkOff = !blinkOff;
            setColor(blinkOff ? Color.off : state.config.color);
        }

        DogLog.log("LED/State", state.toString());
        lastState = state;
    }
}