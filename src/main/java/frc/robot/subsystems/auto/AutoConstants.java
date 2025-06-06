package frc.robot.subsystems.auto;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.*;

import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveConstants;

public final class AutoConstants {
    public static final double backUpPushDistance = Units.inchesToMeters(4.0);
    public static final double backUpCSDistance = Units.inchesToMeters(12.0);

    public static final double maxSetupXError = Units.inchesToMeters(4.0);
    public static final double maxSetupYError = Units.inchesToMeters(8.0);
    public static final double maxSetupDegError = 15.0;

    public static final double scoreCoralTimeout = 3.5;
    public static final double scoreCoralTimeLeft = 4.0;

    // TODO Find out why this doesn't work
    public static final RobotConfig robotConfig = new RobotConfig(
        Mass.ofRelativeUnits(Constants.isRocky ? 145.0 : 132.0, Pounds),
        MomentOfInertia.ofRelativeUnits(Constants.isRocky ? 8.224 : 7.267, KilogramSquareMeters),
        new ModuleConfig(
            SwerveConstants.wheelCircumference / (Math.PI * 2.0),
            SwerveConstants.maxSpeed * 0.95, // Leave a little headroom for inefficiencies
            1.916, // 3847 Spectrum Vex GripLock v2 CoF
            DCMotor.getKrakenX60Foc(1),
            SwerveConstants.chosenModule.driveGearRatio,
            SwerveConstants.driveCurrentLimit,
            1),
        new Translation2d(SwerveConstants.wheelBase / 2.0, SwerveConstants.trackWidth / 2.0),
        new Translation2d(SwerveConstants.wheelBase / 2.0, -SwerveConstants.trackWidth / 2.0),
        new Translation2d(-SwerveConstants.wheelBase / 2.0, SwerveConstants.trackWidth / 2.0),
        new Translation2d(-SwerveConstants.wheelBase / 2.0, -SwerveConstants.trackWidth / 2.0));
}