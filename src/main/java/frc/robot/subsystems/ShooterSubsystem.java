// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumMap;
import java.util.Map;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static frc.robot.Options.*;

public class ShooterSubsystem extends SubsystemBase {
  private static TalonFX top;
  private static TalonFX bottom;
  //private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
  private final VelocityVoltage topControl = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityVoltage bottomControl = new VelocityVoltage(0).withEnableFOC(true);
  private double topCurrentTarget = 0.0;
  private double bottomCurrentTarget = 0.0;
  SendableChooser<Speed> defaultShotChooser = new SendableChooser<>();
  private boolean autoAimingActive = false;

  private class ShooterSpeed {
    double topMotorSpeed;
    double bottomMotorSpeed;

    public ShooterSpeed(double top, double bottom) {
      topMotorSpeed = top;
      bottomMotorSpeed = bottom;
    }
  }

  private class ShooterCalibration {
    double distance;
    ShooterSpeed speed;

    public ShooterCalibration(double distance, ShooterSpeed speed) {
      this.distance = distance;
      this.speed = speed;
    }
  }

  public enum Speed {
    STOP,
    INTAKE,
    IDLE,
    AMP,
    SUBWOOFER,
    AMPSIDE,
    MIDLINE,
    PODIUM,
    FULL,
    VISION,
    SOURCESIDEOTF,
    AMPSIDEOTF,
    OTF,
    SHORTSLIDE,
    SLIDE,
    DUMP,
    SPECIAL,
    EJECT,
    BLOOP,
    SHUTTLE,
    FARSHUTTLE
  };

  private Speed nextShot = null;

  private final EnumMap<Speed, ShooterSpeed> shooterSpeeds = new EnumMap<>(Map.ofEntries(
      Map.entry(Speed.STOP, new ShooterSpeed(Constants.Shooter.stopSpeed, Constants.Shooter.stopSpeed)),
      Map.entry(Speed.INTAKE, new ShooterSpeed(Constants.Shooter.intakeSpeed, Constants.Shooter.intakeSpeed)),
      Map.entry(Speed.IDLE, new ShooterSpeed(Constants.Shooter.idleSpeed, Constants.Shooter.idleSpeed)),
      Map.entry(Speed.AMP, new ShooterSpeed(375, 1025)),
      Map.entry(Speed.SUBWOOFER, new ShooterSpeed(1360, 2830)),
      Map.entry(Speed.AMPSIDE, new ShooterSpeed(2850, 2050)),
      Map.entry(Speed.MIDLINE, new ShooterSpeed(2800, 2300)),
      Map.entry(Speed.PODIUM, new ShooterSpeed(3000, 1600)),
      Map.entry(Speed.FULL, new ShooterSpeed(Constants.Shooter.topSpeed, Constants.Shooter.topSpeed)),
      Map.entry(Speed.OTF, new ShooterSpeed(3000, 1600)),
      Map.entry(Speed.SOURCESIDEOTF, new ShooterSpeed(2950, 1925)),
      Map.entry(Speed.AMPSIDEOTF, new ShooterSpeed(2950, 1800)),
      Map.entry(Speed.SLIDE, new ShooterSpeed(2500, 1000)),
      Map.entry(Speed.SHORTSLIDE, new ShooterSpeed(2250, 900)),
      Map.entry(Speed.DUMP, new ShooterSpeed(2650, 2650)),
      Map.entry(Speed.SPECIAL, new ShooterSpeed(1180, 1180)),
      Map.entry(Speed.EJECT, new ShooterSpeed(-800, -800)),
      Map.entry(Speed.BLOOP, new ShooterSpeed(400, 400))
  ));

  private final ShooterCalibration[] shooterCalibration = {
    new ShooterCalibration(35.9, new ShooterSpeed(1200, 3200)),
    new ShooterCalibration(46.9, new ShooterSpeed(1500, 2500)),
    new ShooterCalibration(59.5, new ShooterSpeed(2200, 2200)),
    new ShooterCalibration(72.2, new ShooterSpeed(2700, 2200)),
    new ShooterCalibration(83.5, new ShooterSpeed(3000, 1900)),
    new ShooterCalibration(95.7, new ShooterSpeed(2900, 1700)),
    new ShooterCalibration(108.1, new ShooterSpeed(2800, 1550)),
    new ShooterCalibration(120.9, new ShooterSpeed(2800, 1425)),
    new ShooterCalibration(132.0, new ShooterSpeed(2700, 1400))
  };

  private final ShooterCalibration[] shuttleCalibration = {
    new ShooterCalibration(210-33, new ShooterSpeed(1450, 1450)),
    new ShooterCalibration(292-33, new ShooterSpeed(2100, 2200)),
    new ShooterCalibration(384-33, new ShooterSpeed(2700, 3000)),
    new ShooterCalibration(449-33, new ShooterSpeed(2900, 3200)), // Blair Robot Project FTW!
    new ShooterCalibration(506-33, new ShooterSpeed(3200, 3506)), // Thanks, YETI!
  };

  public ShooterSubsystem() {
    top = new TalonFX(Constants.Shooter.topShooterID, Constants.Shooter.shooterMotorCanBus);
    bottom = new TalonFX(Constants.Shooter.bottomShooterID, Constants.Shooter.shooterMotorCanBus);
    applyConfigs();

    SmartDashboard.putNumber("shooter/Top RPM adjustment", 0.0);
    SmartDashboard.putNumber("shooter/Bottom RPM adjustment", 0.0);

    defaultShotChooser.setDefaultOption("== VISION ==", Speed.VISION);
    for (Speed speed : Speed.values()) {
      if (speed != Speed.VISION) {
        defaultShotChooser.addOption(speed.toString(), speed);
      }
    }
    SmartDashboard.putData("shooter/Default shot", defaultShotChooser);
  }

  private void applyConfigs() {
    /* Configure the Shooter Motors */
    var m_ShooterMotorsConfiguration = new TalonFXConfiguration();
    /* Set Shooter motors to Brake */
    m_ShooterMotorsConfiguration.MotorOutput.NeutralMode = Constants.Shooter.motorNeutralValue;
    /* Set the Shooters motor direction */
    m_ShooterMotorsConfiguration.MotorOutput.Inverted = Constants.Shooter.motorOutputInverted;
    /* Config the peak outputs */
    m_ShooterMotorsConfiguration.Voltage.PeakForwardVoltage = Constants.Shooter.peakForwardVoltage;
    m_ShooterMotorsConfiguration.Voltage.PeakReverseVoltage = Constants.Shooter.peakReverseVoltage;

    // PID & FF configuration
    m_ShooterMotorsConfiguration.Slot0.kP = Constants.Shooter.kP;
    m_ShooterMotorsConfiguration.Slot0.kI = Constants.Shooter.kI;
    m_ShooterMotorsConfiguration.Slot0.kD = Constants.Shooter.kD;
    m_ShooterMotorsConfiguration.Slot0.kS = Constants.Shooter.kS;
    m_ShooterMotorsConfiguration.Slot0.kV = 1.0 / toRPS(Constants.Shooter.RPMsPerVolt);
    m_ShooterMotorsConfiguration.Slot0.kA = 0.0;
    m_ShooterMotorsConfiguration.Slot0.kG = 0.0;

    /* Apply Shooters Motor Configs */
    top.getConfigurator().apply(m_ShooterMotorsConfiguration);
    bottom.getConfigurator().apply(m_ShooterMotorsConfiguration);
  }

  private double toRPM(double rps) {
    return rps * 60.0;
  }

  private double toRPS(double rpm) {
    return rpm / 60.0;
  }

  public void setNextShot(Speed speed) {
    nextShot = speed;
  }

  public boolean isAutoAimingActive() {
    return autoAimingActive;
  }

  private ShooterSpeed speedFromDistance(double meters, ShooterCalibration[] calibrationTable) {
    double distance = Units.metersToInches(meters);
    ShooterCalibration priorEntry = null;
    ShooterSpeed speed = null;

    for (ShooterCalibration calibration : calibrationTable) {
      if (distance <= calibration.distance) {
        if (priorEntry == null) {
          // Anything closer that minimum calibration distance gets the same speed as minimum distance
          speed = calibration.speed;
        } else {
          // Linear interpolation between calibration entries
          double fraction = (distance - priorEntry.distance) / (calibration.distance - priorEntry.distance);

          speed = new ShooterSpeed(
            fraction * calibration.speed.topMotorSpeed + (1 - fraction) * priorEntry.speed.topMotorSpeed,
            fraction * calibration.speed.bottomMotorSpeed + (1 - fraction) * priorEntry.speed.bottomMotorSpeed
          );
        }

        break;
      }
      priorEntry = calibration;
    }

    // NOTE: Might be null if the calibration distance has been exceeded
    return speed;
  }

  public boolean shoot() {
    return setCurrentSpeed(nextShot);
  }

  private Speed defaultSpeed() {
    return defaultShotChooser.getSelected();
  }

  private boolean setCurrentSpeed(Speed speed) {
    ShooterSpeed shooterSpeed;
    double distance;

    if (speed == null) {
      speed = defaultSpeed();
    }

    if (speed == Speed.VISION) {
      if (PoseSubsystem.getZone() == PoseSubsystem.Zone.FAR) {
        speed = Speed.FARSHUTTLE;
      } else if (PoseSubsystem.getZone() == PoseSubsystem.Zone.MIDDLE) {
        speed = Speed.SHUTTLE;
      }
    }

    if (speed == Speed.VISION) {
      if (optShootWithVision.get()) {
        distance = VisionSubsystem.getInstance().distanceToSpeaker();
      } else {
        distance = PoseSubsystem.getInstance().distanceToSpeaker();
      }
      shooterSpeed = speedFromDistance(distance, shooterCalibration);
      if (shooterSpeed == null) {
        DogLog.log("Shooter/Status", String.format("ShooterSubsystem::setCurrentSpeed: distance of %01.1f too far", Units.metersToInches(distance))); 
        return false;
      }
      //System.out.printf("Shoot @ %01.2f ft: %d, %d%n", VisionSubsystem.getInstance().distanceToSpeaker(), (int)shooterSpeed.topMotorSpeed, (int)shooterSpeed.bottomMotorSpeed);
      autoAimingActive = true;
    } else if (speed == Speed.SHUTTLE) {
      distance = PoseSubsystem.getInstance().distanceToShuttle();
      shooterSpeed = speedFromDistance(distance, shuttleCalibration);
      if (shooterSpeed == null) {
        DogLog.log("Shooter/Status", String.format("ShooterSubsystem::setCurrentSpeed: distance of %01.1f failed to find shuttle speed", Units.metersToInches(distance))); 
        return false;
      }
      //System.out.printf("Shuttle @ %01.2f ft: %d, %d%n", VisionSubsystem.getInstance().distanceToSpeaker(), (int)shooterSpeed.topMotorSpeed, (int)shooterSpeed.bottomMotorSpeed);
      autoAimingActive = true;
    } else if (speed == Speed.FARSHUTTLE) {
      distance = PoseSubsystem.getInstance().distanceToFarShuttle();
      shooterSpeed = speedFromDistance(distance, shuttleCalibration);
      if (shooterSpeed == null) {
        DogLog.log("Shooter/Status", String.format("ShooterSubsystem::setCurrentSpeed: distance of %01.1f failed to find far shuttle speed", Units.metersToInches(distance))); 
        return false;
      }
      //System.out.printf("Shuttle @ %01.2f ft: %d, %d%n", VisionSubsystem.getInstance().distanceToSpeaker(), (int)shooterSpeed.topMotorSpeed, (int)shooterSpeed.bottomMotorSpeed);
      autoAimingActive = true;
    } else {
      shooterSpeed = shooterSpeeds.get(speed);
      autoAimingActive = (speed == Speed.DUMP || speed == Speed.SLIDE);
    }

    setCurrentSpeed(shooterSpeed);
    return true;
  }

  public void shoot(double topRPM, double bottomRPM) {
    setCurrentSpeed(new ShooterSpeed(topRPM, bottomRPM));
  }

  /*private void setVelocityTorque(TalonFX motor, double rpm) {
    motor.setControl(velocityTorqueCurrentFOC.withVelocity(toRPS(rpm)));
  }*/

  private void setCurrentSpeed(ShooterSpeed speed) {
    topCurrentTarget = speed.topMotorSpeed + SmartDashboard.getNumber("shooter/Top RPM adjustment", 0.0);
    bottomCurrentTarget = speed.bottomMotorSpeed + SmartDashboard.getNumber("shooter/Bottom RPM adjustment", 0.0);
    top.setControl(topControl.withVelocity(toRPS(topCurrentTarget)));
    bottom.setControl(bottomControl.withVelocity(toRPS(bottomCurrentTarget)));
    DogLog.log("Shooter/TopRPM", topCurrentTarget);
    DogLog.log("Shooter/BottomRPM", bottomCurrentTarget);
  }

  public void setVoltage(double voltage) {
    top.setControl(voltageOut.withOutput(voltage));
    bottom.setControl(voltageOut.withOutput(voltage));
  }

  public void setRPM(double rpm) {
    setCurrentSpeed(new ShooterSpeed(rpm, rpm));
  }

  public void idle() {
    setCurrentSpeed(Speed.IDLE);
  }

  public void intake() {
    setCurrentSpeed(Speed.INTAKE);
  }

  public void eject() {
    setCurrentSpeed(Speed.EJECT);
  }

  public void stop() {
    setCurrentSpeed(Speed.STOP);
  }

  public boolean isReady(boolean precise) {
    return (Math.abs(toRPM(top.getVelocity().getValueAsDouble()) - topCurrentTarget) < (precise ? Constants.Shooter.maxRPMErrorLong : Constants.Shooter.maxRPMError) &&
      Math.abs(toRPM(bottom.getVelocity().getValueAsDouble()) - bottomCurrentTarget) < (precise ? Constants.Shooter.maxRPMErrorLong : Constants.Shooter.maxRPMError));
  }

  public boolean usingVision() { 
    return (nextShot == Speed.VISION || (nextShot == null && defaultSpeed() == Speed.VISION)) && PoseSubsystem.getZone() == PoseSubsystem.Zone.SPEAKER;
  }

  public boolean shootingDefault() { 
    return nextShot == null;
  }

  public void toggleAmp() {
    if (nextShot == null || nextShot != Speed.AMP) {
      nextShot = Speed.AMP;
    } else {
      nextShot = null;
    }
  }

  public boolean amping() {
    return nextShot == Speed.AMP || (nextShot == null && defaultSpeed() == Speed.AMP);
  }

  public boolean dumping() {
    return nextShot == Speed.DUMP || (nextShot == null && defaultSpeed() == Speed.DUMP);
  }

  public boolean sliding() {
    return nextShot == Speed.SLIDE || (nextShot == null && defaultSpeed() == Speed.SLIDE);
  }

  public boolean shuttling() {
    return nextShot == Speed.SHUTTLE || (nextShot == null && defaultSpeed() == Speed.SHUTTLE);
  }

  public boolean farShuttling() {
    return nextShot == Speed.FARSHUTTLE || (nextShot == null && defaultSpeed() == Speed.FARSHUTTLE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double topVel = toRPM(top.getVelocity().getValueAsDouble());
    double bottomVel = toRPM(bottom.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("shooter/Top RPM", topVel);
    SmartDashboard.putNumber("shooter/Bottom RPM", bottomVel);
    SmartDashboard.putNumber("shooter/Top RPM tgt", topCurrentTarget);
    SmartDashboard.putNumber("shooter/Bottom RPM tgt", bottomCurrentTarget);
    SmartDashboard.putNumber("shooter/Top RPM err", topVel - topCurrentTarget);
    SmartDashboard.putNumber("shooter/Bottom RPM err", bottomVel - bottomCurrentTarget);
    SmartDashboard.putBoolean("shooter/ready", isReady(false));
    SmartDashboard.putString("shooter/Next shot", nextShot == null ? defaultSpeed().toString() : nextShot.toString());
    SmartDashboard.putBoolean("shooter/usingVision", usingVision());


    DogLog.log("Shooter/Top RPM", topVel);
    DogLog.log("Shooter/Bottom RPM", bottomVel);
    DogLog.log("Shooter/Top RPM tgt", topCurrentTarget);
    DogLog.log("Shooter/Bottom RPM tgt", bottomCurrentTarget);
    DogLog.log("Shooter/Top RPM err", topVel - topCurrentTarget);
    DogLog.log("Shooter/Bottom RPM err", bottomVel - bottomCurrentTarget);
    DogLog.log("Shooter/Ready", isReady(false));
    DogLog.log("Shooter/Next shot", nextShot == null ? defaultSpeed().toString() : nextShot.toString());
    DogLog.log("Shooter/usingVision", usingVision());
  }
}