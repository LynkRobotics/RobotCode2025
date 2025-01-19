// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Options.optAimingEnabled;

import java.util.function.DoubleSupplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.TunableOption;
import frc.robot.Constants;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PoseSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.LEDSubsystem.TempState;

public class ShootCommand extends Command {
  private final ShooterSubsystem shooter;
  private final IndexSubsystem index;
  private Swerve swerve = null;
  private boolean feeding = false;
  DoubleSupplier topSupplier = null;
  DoubleSupplier bottomSupplier = null;
  private final VisionSubsystem vision = VisionSubsystem.getInstance();
  private boolean cancelled = false;
  private boolean gone = false;
  private Timer postShotTimer = new Timer();
  private boolean autoAim = true;
  private boolean shooterReady = false;
  private boolean seenTarget = false;
  private static final TunableOption optSetPoseWhenShooting = new TunableOption("Set pose when shooting", true);

  public ShootCommand(ShooterSubsystem shooter, IndexSubsystem index) {
    addRequirements(shooter, index);
    this.shooter = shooter;
    this.index = index;
    assert(vision != null);
  }

  public ShootCommand(ShooterSubsystem shooter, IndexSubsystem index, Swerve swerve) {
    this(shooter, index);
    this.swerve = swerve;
  }

  public ShootCommand(ShooterSubsystem shooter, IndexSubsystem index, boolean autoAim) {
    this(shooter, index);
    this.autoAim = autoAim;
  }

  public ShootCommand(ShooterSubsystem shooter, IndexSubsystem index, Swerve swerve, boolean autoAim) {
    this(shooter, index, swerve);
    this.autoAim = autoAim;
  }

  public ShootCommand(ShooterSubsystem shooter, IndexSubsystem index, DoubleSupplier topSupplier, DoubleSupplier bottomSupplier) {
    this(shooter, index);
    this.topSupplier = topSupplier;
    this.bottomSupplier = bottomSupplier;
    this.autoAim = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("Spinning up shooter");
    shooterReady = false;
    LEDSubsystem.setTempState(TempState.SHOOTING);
    cancelled = false;
    seenTarget = false;
    feeding = false;
    gone = false;

    if (topSupplier != null && bottomSupplier != null) {
      shooter.shoot(topSupplier.getAsDouble(), bottomSupplier.getAsDouble());
    } else {
      if (!index.haveNote()) {
        DogLog.log("Shooter/Status", "ERROR: Cancelling ShootCommand without note");
        cancelled = true;
      }
      if (!shooter.shoot()) {
        DogLog.log("Shooter/Status", "ERROR: Cancelling ShootCommand due to shoot() failure");
        cancelled = true;
      }
      if (cancelled) {
        LEDSubsystem.setTempState(TempState.ERROR);
        cancel();
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (cancelled) {
      return;
    }
    // TODO Consider not requiring seeing the target to shoot
    if (shooter.usingVision() && !seenTarget) {
      seenTarget = vision.haveSpeakerTarget();
      if (!seenTarget) {
        DogLog.log("Shooter/Status", "Shoot Command waiting for speaker target");
        return;
      }
    }
    boolean precise = shooter.usingVision() && vision.distanceToSpeaker() > Constants.Shooter.farDistance;
    if (!feeding && shooter.isReady(precise)) {
      boolean aligned = !autoAim || !optAimingEnabled.get(); // "Aligned" if not automatic aiming
      if (!shooterReady) {
        DogLog.log("Shooter/Status", "Shooter is ready");
        shooterReady = true;
      }

      if (!aligned) {
        if (shooter.usingVision()) {
          // Aligned if vision is aligned with target
          aligned = vision.haveTarget() && Math.abs(vision.angleError().getDegrees()) < Constants.Vision.maxAngleError;
        } else if (shooter.dumping()) {
          aligned = PoseSubsystem.getInstance().dumpShotAligned();
        } else if (shooter.sliding()) {
          aligned = PoseSubsystem.getInstance().slideShotAligned();
        } else if (shooter.shuttling()) {
          aligned = PoseSubsystem.getInstance().shuttleShotAligned();
        } else if (shooter.farShuttling()) {
          aligned = PoseSubsystem.getInstance().farShuttleShotAligned();
        } else if (shooter.shootingDefault()) {
            PoseSubsystem.Zone zone = PoseSubsystem.getZone();

            if (zone == PoseSubsystem.Zone.FAR) {
              aligned = PoseSubsystem.getInstance().farShuttleShotAligned();
            } else if (zone == PoseSubsystem.Zone.MIDDLE) {
              aligned = PoseSubsystem.getInstance().shuttleShotAligned();
            } else {
              System.out.println("Unexpect case of shooting default without vision but in zone " + zone.toString());
              aligned = true;
            }
        } else {
          // "Aligned" because all other shots don't require alignment
          aligned = true;
        }
      }
      if (aligned) {
        index.feed();
        feeding = true;
        DogLog.log("Shooter/Status", String.format("Shooting from vision angle %01.1f deg @ %01.1f inches", vision.angleToSpeaker().getDegrees(), Units.metersToInches(vision.distanceToSpeaker())));
        if (shooter.usingVision() && DriverStation.isAutonomousEnabled()) {
          Pose2d pose = vision.lastPose();
          if (swerve == null) {
            DogLog.log("Shooter/Status", "Unable to set pose due to lack of Swerve subsystem");
          } else if (optSetPoseWhenShooting.get()) {
            Pose2d oldPose = PoseSubsystem.getInstance().getPose();
            DogLog.log("Shooter/Status", "Setting pose based on vision: " + PoseSubsystem.prettyPose(pose) + ", was " + PoseSubsystem.prettyPose(oldPose));
            PoseSubsystem.getInstance().setPose(pose);
          }
        }
      }
    }
    if (topSupplier == null || bottomSupplier == null) {
      // Update shooter speed every iteration, unless we specifically set a certain speed at the onset of the command
      if (!shooter.shoot()) {
        DogLog.log("Shooter/Status", "Cancelling ShootCommand due to shoot() failure [2]");
        cancelled = true;
        cancel();
        LEDSubsystem.setTempState(TempState.ERROR);
      }
    }
    if (feeding) {
      if (!gone && !index.haveNote()) {
        postShotTimer.restart();
        gone = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop feeding
    index.stop();

    // Restore idle speed
    shooter.idle();

    // Restore default shot
    shooter.setNextShot(null);

    // Adjust LED state
    if (interrupted) {
      LEDSubsystem.setTempState(TempState.ERROR);
    } else {
      LEDSubsystem.clearTempState();
    }
 }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (cancelled) {
      return true;
    }
    if (gone && postShotTimer.hasElapsed(Constants.Shooter.postShotTimeout)) {
      return true;
    }
    return false;
  }
}