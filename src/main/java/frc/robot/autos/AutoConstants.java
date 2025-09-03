package frc.robot.autos;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import static edu.wpi.first.units.Units.*;

import frc.robot.subsystems.swerve.SwerveConstants;

public final class AutoConstants {
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