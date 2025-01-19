package frc.robot.commands;

import frc.lib.util.TunableOption;
import frc.robot.Constants;
import frc.robot.subsystems.PoseSubsystem;
import frc.robot.Robot;
import frc.robot.Constants.Pose;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;
import static frc.robot.Options.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {
    private final Swerve s_Swerve;
    private final ShooterSubsystem s_Shooter;
    private final VisionSubsystem s_Vision;
    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;
    private final DoubleSupplier rotationSup;
    private DoubleSupplier speedLimitRotSupplier;
    private PoseSubsystem s_Pose = null;
    private Rotation2d lastAngle = null;
    private AimingMode aimingMode = AimingMode.MANUAL;
    private static final TunableOption optFullFieldAiming = new TunableOption("Full field aiming", true);

    private enum AimingMode {
        MANUAL,
        TARGET,
        MAINTAIN
    }

    public TeleopSwerve(Swerve s_Swerve, ShooterSubsystem s_Shooter, VisionSubsystem s_Vision, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, DoubleSupplier speedLimitRotSupplier) {
        this.s_Swerve = s_Swerve;
        this.s_Shooter = s_Shooter;
        this.s_Vision = s_Vision;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.speedLimitRotSupplier = speedLimitRotSupplier;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        double teleOpMult = SmartDashboard.getNumber("TeleOp Speed Governor", 1.0);
        translationVal *= teleOpMult;

        if (s_Pose == null) {
            s_Pose = PoseSubsystem.getInstance();
        }

        // Driver position is inverted for Red alliance, so adjust field-oriented controls
        if (Robot.isRed()) {
            translationVal *= -1.0;
            strafeVal *= -1.0;
        }

        /* Override rotation if using vision to aim */
        if (optAimingEnabled.get()) {
            if (s_Shooter.isAutoAimingActive()) {
                Rotation2d angleError;
                
                if (s_Shooter.usingVision()) {
                    if (optShootWithVision.get()) {
                        angleError = s_Vision.angleError();
                    } else {
                        angleError = PoseSubsystem.getInstance().angleError();
                    }
                } else if (s_Shooter.dumping()) {
                    angleError = PoseSubsystem.getInstance().dumpShotError();
                } else if (s_Shooter.sliding()) {
                    angleError = PoseSubsystem.getInstance().slideShotError();
                } else if (s_Shooter.shuttling()) {
                    angleError = PoseSubsystem.getInstance().shuttleShotError();
                } else if (s_Shooter.farShuttling()) {
                    angleError = PoseSubsystem.getInstance().farShuttleShotError();
                } else if (s_Shooter.shootingDefault()) {
                    PoseSubsystem.Zone zone = PoseSubsystem.getZone();
        
                    if (zone == PoseSubsystem.Zone.FAR) {
                      angleError = PoseSubsystem.getInstance().farShuttleShotError();
                    } else if (zone == PoseSubsystem.Zone.MIDDLE) {
                      angleError = PoseSubsystem.getInstance().shuttleShotError();
                    } else {
                      System.out.println("Unexpect case of isAutoAimingActive with default shot but in zone " + zone.toString());
                      angleError = new Rotation2d();
                    }
                } else {
                    System.out.println("Unexpected case of isAutoAimingActive but not usingVision nor dumping nor sliding nor (far)shuttling");
                    angleError = new Rotation2d();
                }
                
                if (aimingMode != AimingMode.TARGET) {
                    PoseSubsystem.angleErrorReset();
                    aimingMode = AimingMode.TARGET;
                }
                if (s_Shooter.usingVision() && !s_Vision.haveTarget()) {
                    rotationVal = 0.0;
                } else {
                    rotationVal = PoseSubsystem.angleErrorToSpeed(angleError);
                }
            } else if (Math.abs(rotationVal) < Constants.aimingOverride) {
                boolean haveNote = IndexSubsystem.getInstance().haveNote();

                if (haveNote && optFullFieldAiming.get()) {
                    PoseSubsystem.Zone zone = PoseSubsystem.getZone();
                    Rotation2d targetAngle;

                    if (zone == PoseSubsystem.Zone.SPEAKER) {
                        if (s_Shooter.amping()) {
                            if (optAimAtAmp.get()) {
                                //targetAngle = s_Pose.angleToAmp();
                                targetAngle = new Rotation2d(Units.degreesToRadians(-90.0));
                            } else {
                                targetAngle = null; // Don't override aiming
                            }
                        } else {
                            targetAngle = s_Pose.angleToSpeaker();
                        }
                    } else if (zone == PoseSubsystem.Zone.FAR) {
                        targetAngle = s_Pose.angleToFarShuttle();
                    } else {
                        targetAngle = s_Pose.angleToShuttle();
                    }

                    if (targetAngle != null) {
                        Rotation2d angleError = targetAngle.minus(s_Pose.getPose().getRotation());
                        rotationVal = PoseSubsystem.angleErrorToSpeed(angleError);
                    }
                } else if (haveNote && s_Vision.haveSpeakerTarget()) {              
                    if (aimingMode != AimingMode.TARGET) {
                        PoseSubsystem.angleErrorReset();
                        aimingMode = AimingMode.TARGET;
                    }
                    rotationVal = PoseSubsystem.angleErrorToSpeed(s_Vision.angleError());
                } else if (PoseSubsystem.getTargetAngle() != null) {
                    if (aimingMode != AimingMode.TARGET) {
                        PoseSubsystem.angleErrorReset();
                        aimingMode = AimingMode.TARGET;
                    }
                    rotationVal = PoseSubsystem.angleErrorToSpeed(PoseSubsystem.getTargetAngle().minus(PoseSubsystem.getInstance().getPose().getRotation()));
                } else {
                    if (aimingMode == AimingMode.TARGET) {
                        aimingMode = AimingMode.MANUAL;
                    }
                }
            } else {
                if (aimingMode == AimingMode.TARGET) {
                    aimingMode = AimingMode.MANUAL;
                }
            }
        } else {
            if (aimingMode == AimingMode.TARGET) {
                aimingMode = AimingMode.MANUAL;
            }
        }

        // Maintain angle when not explicitly changing
        Rotation2d currentAngle = s_Pose.getPose().getRotation();
        if (optMaintainAngle.get() && Math.abs(rotationVal) < Constants.aimingOverride) {
            if (lastAngle != null && aimingMode != AimingMode.TARGET) {
                if (aimingMode != AimingMode.MAINTAIN) {
                    PoseSubsystem.angleErrorReset(Pose.maintainPID);
                    lastAngle = currentAngle;
                    aimingMode = AimingMode.MAINTAIN;
                } else {
                    Rotation2d angleError = lastAngle.minus(currentAngle);
                    rotationVal = PoseSubsystem.angleErrorToSpeed(angleError, Pose.maintainPID);
                }
            }
        } else {
            lastAngle = currentAngle;
            aimingMode = AimingMode.MANUAL;
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
            rotationVal * Constants.Swerve.maxAngularVelocity * speedLimitRotSupplier.getAsDouble(), 
            true
        );
        // SmartDashboard.putNumber("rotationValue", speedLimitRotSupplier.getAsDouble());
    }
}