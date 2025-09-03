package frc.robot.autos;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import static edu.wpi.first.units.Units.*;

import java.util.Collections;
import java.util.Map;

import frc.robot.subsystems.pose.PoseConstants.ReefFace;
import frc.robot.subsystems.swerve.SwerveConstants;

public final class AutoConstants {
    public static final double backUpPushDistance = Units.Inches.of(4.0).in(Units.Meters);
    public static final double backUpCSDistance = Units.Inches.of(12.0).in(Units.Meters);

    public static final double maxSetupXError = Units.Inches.of(4.0).in(Units.Meters);
    public static final double maxSetupYError = Units.Inches.of(8.0).in(Units.Meters);
    public static final double maxSetupDegError = 15.0;

    public static final double scoreCoralTimeout = 3.5;
    public static final double scoreCoralTimeLeft = 4.0;

    // TODO Calculate from the field ... or use PathPlanner
    public enum AutoPose {
        GH_APPROACH(6.1, 4.03, 180.0),
        BARGE_APPROACH(7.5, 5.1, 0.0),
        IJ_APPROACH(5.25, 5.4, 60.0),
        EF_APPROACH(5.3, 2.45, -60.0),
        EF_CLEAR(4.4, 2.19, -60.0),
        LOLLIPOP1(1.22, 2.19, 180.0),
        LOLLIPOP1_APPROACH(3.0, 2.19, 180.0),
        LOLLIPOP1_THROUGH(0.6, 2.19, 180.0),
        AB_APPROACH(2.0, 4.03, 180.0),
        LOLLIPOP2(1.22, 4.03, 180.0),
        LOLLIPOP2_THROUGH(0.6, 4.03, 180.0),
        LOLLIPOP3_APPROACH(2.4, 4.7, 135.0),
        LOLLIPOP3_THROUGH(0.9, 6.2, 135.0),
        LOLLIPOP3(1.22, 5.87, 180.0);

        public final Pose2d pose;

        private AutoPose(double xMeters, double yMeters, double rotDegrees) {
            pose = new Pose2d(Units.Meters.of(xMeters), Units.Meters.of(yMeters), Rotation2d.fromDegrees(rotDegrees));
        }
    }

    public static final Map<ReefFace, ReefFace> mirroredFaces = Collections.unmodifiableMap(Map.ofEntries(
        Map.entry(ReefFace.AB, ReefFace.AB),
        Map.entry(ReefFace.CD, ReefFace.KL),
        Map.entry(ReefFace.EF, ReefFace.IJ),
        Map.entry(ReefFace.GH, ReefFace.GH),
        Map.entry(ReefFace.IJ, ReefFace.EF),
        Map.entry(ReefFace.KL, ReefFace.CD)));

    public static final RobotConfig robotConfig = new RobotConfig(
        Mass.ofRelativeUnits(135, Pounds),
        MomentOfInertia.ofRelativeUnits(6, KilogramSquareMeters), //6 kg m ^2: 1678 Choreo Constant
        new ModuleConfig(
            SwerveConstants.wheelCircumference / (Math.PI * 2.0),
            SwerveConstants.maxSpeed, 
            1.1, // 1678 Choreo Constant
            DCMotor.getKrakenX60Foc(1),
            SwerveConstants.chosenModule.driveGearRatio,
            SwerveConstants.driveStatorCurrentLimit,
            1),
        new Translation2d(SwerveConstants.wheelBase.div(2.0), SwerveConstants.trackWidth.div(2.0)),
        new Translation2d(SwerveConstants.wheelBase.div(2.0), SwerveConstants.trackWidth.div(2.0).unaryMinus()),
        new Translation2d(SwerveConstants.wheelBase.div(2.0).unaryMinus(), SwerveConstants.trackWidth.div(2.0)),
        new Translation2d(SwerveConstants.wheelBase.div(2.0).unaryMinus(), SwerveConstants.trackWidth.div(2.0)).unaryMinus());
}