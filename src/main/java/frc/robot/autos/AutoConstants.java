package frc.robot.autos;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import frc.robot.subsystems.drive.DriveConstants;

public final class AutoConstants {
    public static final RobotConfig robotConfig = new RobotConfig(
        Units.Pounds.of(135),
        Units.KilogramSquareMeters.of(6),
        new ModuleConfig(
            Units.Inches.of(1.94),
            DriveConstants.kMaxSpeed,
            1.1, // 1678 Choreo Constant
            DCMotor.getKrakenX60Foc(1),
            6.48,
            Units.Amps.of(80.0),
            1),
        new Translation2d(Units.Inches.of(10.5), Units.Inches.of(10.5)),
        new Translation2d(Units.Inches.of(10.5), Units.Inches.of(-10.5)),
        new Translation2d(Units.Inches.of(-10.5), Units.Inches.of(10.5)),
        new Translation2d(Units.Inches.of(-10.5), Units.Inches.of(-10.5)));
}