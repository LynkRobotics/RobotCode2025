// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedCommands;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private final CANdle candle;
  private final CandleSelection which;
  private static BaseState baseState = BaseState.DISABLED;
  private static TempState tempState = null;
  private static BaseState lastBaseState = null;
  private static TempState lastTempState = null;
  private static double tempStateExpiry = 0.0;
  private static Timer tempStateTimer = new Timer();
  private static double blinkInterval = 0.25;
  private static Timer blinkTimer = new Timer();
  private static boolean blinkOff = false;

  public static class Color {
    private final int R, G, B;
    public Color(int r,  int g, int b) {
      R = r;
      G = g;
      B = b;
    }
  }

  public enum BaseState {
    DISABLED,
    READY,
  }

  public enum TempState {
    ERROR
  }

  public enum CandleSelection {
    LEFT,
    RIGHT
  }

  public static final class Colors {
    public static final Color off = new LEDSubsystem.Color(0, 0, 0);
    public static final Color red = new LEDSubsystem.Color(255, 0, 0);
    public static final Color green = new LEDSubsystem.Color(0, 255, 0);
    public static final Color blue = new LEDSubsystem.Color(0, 0, 255);
    public static final Color cyan = new LEDSubsystem.Color(0, 255, 255);
    public static final Color magenta = new LEDSubsystem.Color(255, 0, 255);
    public static final Color yellow = new LEDSubsystem.Color(255, 255, 0);
    public static final Color white = new LEDSubsystem.Color(255, 255, 255);
    public static final Color lynk = new LEDSubsystem.Color(255, 64, 0);
    public static final Color disabled = new LEDSubsystem.Color(200, 0, 0);
  }

  public LEDSubsystem(CandleSelection which) {
    this.which = which;
    candle = new CANdle(which == CandleSelection.LEFT ? 0 : 1, "rio");
    applyConfigs();

    // candle.animate(new RainbowAnimation(1,0.4, 94));
    animate(new FireAnimation(1.0, 0.38, 94, 0.8, 0.2, false, 8));
    // setBaseState(BaseState.READY);
  }

  private void applyConfigs() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.brightnessScalar = 1.0;
    config.stripType = LEDStripType.GRB;
    config.v5Enabled = true;
    config.disableWhenLOS = false; //TODO: verify // TODO: true -- why is this triggering?

    candle.configAllSettings(config);
  }

  public static void setBaseState(BaseState newState) {
    baseState = newState;
  }
  
  public static void setTempState(TempState newState) {
    tempState = newState;
  }

  public void animate(Animation animation) {
    candle.animate(animation);
  }
  
  public static void clearTempState() {
    tempState = null;
  }
  
  public void setColor(Color color) {
    //System.out.printf("Setting color (%d, %d, %d)%n", color.R, color.G, color.B);
    candle.setLEDs(color.R, color.G, color.B);
  }

  public void setRainbow(){
    candle.clearAnimation(0);
    candle.animate(new RainbowAnimation(0.50, 0.5, 68, false, 8));
  }

  public void clearAnimation(){
    candle.clearAnimation(0);
  }

  private Color tempStateColor(TempState state) {
    if (state == TempState.ERROR) {
      return Colors.red;
    } else {
      DogLog.log("LED/Status", "tempStateColor: Unknown state: " + state);
      return Colors.off;
    }
  }

  private Color baseStateColor(BaseState state) {
    if (state == BaseState.DISABLED) {
      return Colors.disabled;
    } else if (state == BaseState.READY) {
      return Colors.lynk;
    } else {
      DogLog.log("LED/Status", "baseStateColor: Unknown state: " + state);
      return Colors.off;
    }
  }



  @Override
  public void periodic() {
    SmartDashboard.putString("LED/Base state", baseState == null ? "NULL" : baseState.toString());
    SmartDashboard.putString("LED/Temp state", tempState == null ? "NULL" : tempState.toString());

    /*
     * if (enabled){
     *  if (LoggedAlert.ERROR == True){
     *    Flash.RED
     *  }
     *  if (LoggedAlert.WARNING == True){
     *    Flash.YELLOW
     *  }
     *  if (isIntaking){
     *    Flash.WHITE
     *  }
     *  if (goingForAlgae){
     *    Flash.ElevatorLevel.HighTideTeal
     *  }
     *  if (movingElevatorForScoringCoral){
     *    DripUp.ElevatorLevel.WHITE
     *    DeadLEDs.BLACK
     *  }
     *  if (hasCoral.inIndex){
     *    Solid.WHITE
     *  }
     *  if (){
     *  }
     * }
     */
    // If the temporary state is active...
    if (tempState != null) {
      if (tempState == lastTempState) {
        // Temporary state unchanged
        if (tempStateExpiry > 0.0 && tempStateTimer.hasElapsed(tempStateExpiry)) {
          // Temporary state has expired, and base state should be shown
          tempState = null;
        } else {
          // Temporary state is active, but might need to be blinked
          if (blinkTimer.hasElapsed(blinkInterval)) {
            blinkOff = !blinkOff;
            if (blinkOff) {
              setColor(Colors.off);
            } else {
              setColor(tempStateColor(tempState));
            }
            blinkTimer.restart();
          }
        }
      } else {
        // Start new temporary state
        setColor(tempStateColor(tempState));
        blinkOff = false;
        blinkTimer.restart();
        if (tempState == TempState.ERROR) {
          blinkInterval = 0.10;
          tempStateExpiry = 0.80;
          tempStateTimer.restart();
        } else {
          blinkInterval = 0.20;
          tempStateExpiry = 0.0;
        }
      }
    }

    // Check for a changed base state, or a dropped temporary state
    if (tempState == null) {
      // Check for possible temporary startup condition, and skip it
      if (baseState == null) {
        DogLog.log("LED/Status", "LEDSubsystem::periodic: Base State NULL");
      } else {
        if (baseState != lastBaseState || lastTempState != null) {
          setColor(baseStateColor(baseState));
          lastBaseState = baseState;
        }
      }
    }

    // Update the last states processed for reference in the next iteration
    lastTempState = tempState;
    lastBaseState = baseState;
  }
}