package frc.robot.autos;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.*;

import java.util.Collections;
import java.util.Map;

import frc.robot.Constants;
import frc.robot.subsystems.pose.PoseConstants.ReefFace;
import frc.robot.subsystems.swerve.SwerveConstants;

public final class AutoConstants {
    public static final double backUpPushDistance = Units.inchesToMeters(4.0);
    public static final double backUpCSDistance = Units.inchesToMeters(12.0);

    public static final double maxSetupXError = Units.inchesToMeters(4.0);
    public static final double maxSetupYError = Units.inchesToMeters(8.0);
    public static final double maxSetupDegError = 15.0;

    public static final double scoreCoralTimeout = 3.5;
    public static final double scoreCoralTimeLeft = 4.0;

    public static final Map<ReefFace, ReefFace> mirroredFaces = Collections.unmodifiableMap(Map.ofEntries(
        Map.entry(ReefFace.AB, ReefFace.AB),
        Map.entry(ReefFace.CD, ReefFace.KL),
        Map.entry(ReefFace.EF, ReefFace.IJ),
        Map.entry(ReefFace.GH, ReefFace.GH),
        Map.entry(ReefFace.IJ, ReefFace.EF),
        Map.entry(ReefFace.KL, ReefFace.CD)));

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
            SwerveConstants.driveStatorCurrentLimit,
            1),
        new Translation2d(SwerveConstants.wheelBase.div(2.0), SwerveConstants.trackWidth.div(2.0)),
        new Translation2d(SwerveConstants.wheelBase.div(2.0), SwerveConstants.trackWidth.div(2.0).unaryMinus()),
        new Translation2d(SwerveConstants.wheelBase.div(2.0).unaryMinus(), SwerveConstants.trackWidth.div(2.0)),
        new Translation2d(SwerveConstants.wheelBase.div(2.0).unaryMinus(), SwerveConstants.trackWidth.div(2.0)).unaryMinus());
}