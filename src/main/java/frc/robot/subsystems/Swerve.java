package frc.robot.subsystems;

import frc.lib.util.LoggedCommands;
import frc.lib.util.SwerveModule;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.util.sendable.Sendable;
// import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private PoseSubsystem s_Pose = null;

    public SwerveModule[] mSwerveMods;

    public Swerve() {
        // To use Latch in 2025, we need to reverse what the front of the robot is
        if (Constants.isRocky) {
            mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
            };
        } else {
            mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod3.constants),
                new SwerveModule(1, Constants.Swerve.Mod2.constants),
                new SwerveModule(2, Constants.Swerve.Mod1.constants),
                new SwerveModule(3, Constants.Swerve.Mod0.constants)
            };
        }

        // SmartDashboard.putData("Swerve Drive", new Sendable() {
        //     @Override
        //     public void initSendable(SendableBuilder builder) {
        //         builder.setSmartDashboardType("SwerveDrive");

        //         builder.addDoubleProperty("Front Left Angle", () -> mSwerveMods[0].getPosition().angle.getRadians(), null);
        //         builder.addDoubleProperty("Front Left Velocity", () -> mSwerveMods[0].getState().speedMetersPerSecond, null);

        //         builder.addDoubleProperty("Front Right Angle", () -> mSwerveMods[1].getPosition().angle.getRadians(), null);
        //         builder.addDoubleProperty("Front Right Velocity", () -> mSwerveMods[1].getState().speedMetersPerSecond, null);

        //         builder.addDoubleProperty("Back Left Angle", () -> mSwerveMods[2].getPosition().angle.getRadians(), null);
        //         builder.addDoubleProperty("Back Left Velocity", () -> mSwerveMods[2].getState().speedMetersPerSecond, null);

        //         builder.addDoubleProperty("Back Right Angle", () -> mSwerveMods[3].getPosition().angle.getRadians(), null);
        //         builder.addDoubleProperty("Back Right Velocity", () -> mSwerveMods[3].getState().speedMetersPerSecond, null);

        //         builder.addDoubleProperty("Robot Angle", () -> s_Pose != null ? s_Pose.getGyroYaw().getRadians() : 0.0, null);            }
        // });
    }

    public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
        ChassisSpeeds desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    PoseSubsystem.getInstance().getHeading()
                                );

        driveRobotRelative(desiredChassisSpeeds, isOpenLoop);
    }

    public ChassisSpeeds getSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelativeAuto(ChassisSpeeds desirChassisSpeeds) {
        driveRobotRelative(desirChassisSpeeds, false);
    }

    public void driveRobotRelative(ChassisSpeeds desiredChassisSpeeds, boolean isOpenLoop) {
        ChassisSpeeds.discretize(desiredChassisSpeeds, 0.02); 
        
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds); 
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        DogLog.log("Swerve/Desired Module States", swerveModuleStates);
        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void alignStraight() {
        SwerveModuleState aligned = new SwerveModuleState(0.0, new Rotation2d());

        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(aligned, false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void resetModulesToAbsolute() {
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void setMotorsToCoast(){
        for(SwerveModule mod : mSwerveMods){
            mod.setCoastMode();  
        }
        DogLog.log("Swerve/Status", "Coasted Swerve Motors");
    }

    public void setDriveMotorsToCoast(){
        for(SwerveModule mod : mSwerveMods){
            mod.setDriveCoastMode();  
        }
        DogLog.log("Swerve/Status", "Coasted Swerve Drive Motors");
    }

    public Command CoastDriveMotors() {
        return LoggedCommands.runOnce("Set Swerve Drive to Coast", this::setDriveMotorsToCoast);
    }

    public void setMotorsToBrake(){
        for(SwerveModule mod : mSwerveMods){
            mod.setBrakeMode();  
        }
        DogLog.log("Swerve/Status", "Braked Swerve Motors");
    }

    public void setDriveMotorsToBrake(){
        for(SwerveModule mod : mSwerveMods){
            mod.setDriveBrakeMode();
        }
        DogLog.log("Swerve/Status", "Braked Swerve Drive Motors");
    }

    public Command BrakeDriveMotors() {
        return LoggedCommands.runOnce("Set Swerve Drive to Brake", this::setDriveMotorsToBrake);
    }

    public void stopSwerve(){
        drive(new Translation2d(0, 0), 0, false);
        DogLog.log("Swerve/Status", "Stopped Swerve");
    }

    public Command Stop() {
        return LoggedCommands.runOnce("Stop Swerve", this::stopSwerve, this);
    }

    @Override
    public void periodic() {
        if (s_Pose == null) {
            s_Pose = PoseSubsystem.getInstance();
        }

        boolean aligned = true;
        for(SwerveModule mod : mSwerveMods) {
            DogLog.log("Swerve/Mod/" + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Swerve/Mod/" + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Swerve/Mod/" + mod.moduleNumber + " Alignment Error", mod.alignmentError());
            SmartDashboard.putBoolean("Swerve/Mod/" + mod.moduleNumber + " Aligned", mod.isAligned());
            aligned = aligned && mod.isAligned();
        }
        SmartDashboard.putBoolean("Swerve/Modules Aligned", aligned);
        DogLog.log("Swerve/Actual Module States", getModuleStates());        
    }
}