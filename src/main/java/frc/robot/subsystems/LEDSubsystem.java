// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.LarsonAnimation;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LED;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  public LED[] mLeds;
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
    ERROR,
    WARNING,
    INTAKING,
    ALGAE
  }


  public static final class Colors {
    public static final Color off = new LEDSubsystem.Color(0, 0, 0);
    public static final Color red = new LEDSubsystem.Color(255, 0, 0);
    public static final Color green = new LEDSubsystem.Color(0, 255, 0);
    public static final Color teal = new LEDSubsystem.Color(44,162,165); //i wonder who that could be
    public static final Color blue = new LEDSubsystem.Color(0, 0, 255);
    public static final Color cheesyBlue = new LEDSubsystem.Color(0, 112, 255);
    public static final Color cyan = new LEDSubsystem.Color(0, 255, 255);
    public static final Color magenta = new LEDSubsystem.Color(255, 0, 255);
    public static final Color yellow = new LEDSubsystem.Color(255, 255, 0);
    public static final Color white = new LEDSubsystem.Color(255, 255, 255);
    public static final Color lynk = new LEDSubsystem.Color(255, 64, 0);
    public static final Color disabled = new LEDSubsystem.Color(200, 0, 0);
  }

  public LEDSubsystem() {
    mLeds = new LED[] {
      new LED(Constants.LEDs.leftCandle, Constants.LEDs.canBus),
      new LED(Constants.LEDs.rightCandle, Constants.LEDs.canBus)
    };

    animate(Constants.LEDs.fireAnimation);
  }

  public static void setBaseState(BaseState newState) {
    baseState = newState;
  }
  
  public static void setTempState(TempState newState) {
    tempState = newState;
  }

  public void animate(Animation animation) {
    for (LED led : mLeds) {
      led.setAnimation(animation);
    }
  }
  
  public static void clearTempState() {
    tempState = null;
  }
  
  public void setColor(Color color) {
    for (LED led : mLeds) {
      led.setLEDs(color.R, color.G, color.B);
    }
  }

  public void setRainbow(){
    for (LED led : mLeds) {
      led.clearAnimation();
      led.setAnimation(Constants.LEDs.rainbowAnimation); 
    }
  }

  public void clearAnimation(){
    for (LED led : mLeds) {
      led.clearAnimation();
    }
  }

  public void setLEDs(Color color, int count) {
    for (LED led : mLeds) {
      led.setLEDs(color.R, color.G, color.B, 255, Constants.LEDs.startIdx, count);
    }
  }

  public void setLEDs(Color color) {
    for (LED led : mLeds) {
      led.setLEDs(0, 0, 0);
    }
  }

  public void setLarson(Color color) {
    clearAnimation();
    animate(new LarsonAnimation(color.R, color.G, color.B, 255, 0.5, 94, LarsonAnimation.BounceMode.Front, 7, 8));
  }

  private Color tempStateColor(TempState state) {
    if (state == TempState.ERROR) {
      return Colors.red;
    } if (state == TempState.WARNING) {
      return Colors.yellow;
    } if (state == TempState.INTAKING) {
      return Colors.white;
    } if (state == TempState.ALGAE) {
      return Colors.teal;
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
      // Clear current animation
      clearAnimation();
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
        if (tempState == TempState.ERROR || tempState == TempState.WARNING) {
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

    if (RobotState.haveCoral() && !RobotState.coralReady()) {
      setTempState(TempState.INTAKING);
    }
    if (RobotState.coralReady()) {
      setLEDs(Colors.white, 48); //TODO: make this dynamic to the elevator postiong
    }
    if (RobotState.haveAlgae()) {
      setTempState(TempState.ALGAE);      
    }
    

    // Update the last states processed for reference in the next iteration
    lastTempState = tempState;
    lastBaseState = baseState;
  }
}