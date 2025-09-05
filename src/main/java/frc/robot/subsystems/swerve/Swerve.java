package frc.robot.subsystems.swerve;

import frc.lib.util.LoggedCommands;
import frc.lib.util.SwerveModule;
import frc.robot.subsystems.pose.Pose;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public static final Swerve instance = new Swerve();
    
    public SwerveModule[] mSwerveMods;

    public Swerve() {
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, SwerveConstants.Mod0.constants),
            new SwerveModule(1, SwerveConstants.Mod1.constants),
            new SwerveModule(2, SwerveConstants.Mod2.constants),
            new SwerveModule(3, SwerveConstants.Mod3.constants)
        };

        SmartDashboard.putData(LoggedCommands.runOnce("Sync Swerve to CANcoders", this::resetModulesToAbsolute, this).ignoringDisable(true));

        SmartDashboard.putData("Drive Test", LoggedCommands.sequence("Drive Test",
            LoggedCommands.runOnce("Move forward", () -> drive(new Translation2d(0.25, 0.0).times(SwerveConstants.maxSpeed), 0.0, true), this),
            Commands.waitSeconds(2.0),
            Stop()));
    }

    public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
        ChassisSpeeds desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    Pose.instance.getHeading()
                                );

        driveRobotRelative(desiredChassisSpeeds, isOpenLoop);
    }

    public ChassisSpeeds getSpeeds() {
        return SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelativeAuto(ChassisSpeeds desirChassisSpeeds) {
        driveRobotRelative(desirChassisSpeeds, false);
    }

    public void driveRobotRelative(ChassisSpeeds desiredChassisSpeeds, boolean isOpenLoop) {
        DogLog.log("Swerve/Desired Chassis Speeds", desiredChassisSpeeds);
        DogLog.log("Swerve/Desired vx", desiredChassisSpeeds.vxMetersPerSecond);
        DogLog.log("Swerve/Desired vy", desiredChassisSpeeds.vyMetersPerSecond);
        DogLog.log("Swerve/Desired omega", desiredChassisSpeeds.omegaRadiansPerSecond);

        ChassisSpeeds.discretize(desiredChassisSpeeds, 0.02); 
        
        SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);

        DogLog.log("Swerve/Mod0 Speed", swerveModuleStates[0].speedMetersPerSecond);
        DogLog.log("Swerve/Mod1 Speed", swerveModuleStates[1].speedMetersPerSecond);
        DogLog.log("Swerve/Mod2 Speed", swerveModuleStates[2].speedMetersPerSecond);
        DogLog.log("Swerve/Mod3 Speed", swerveModuleStates[3].speedMetersPerSecond);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

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
            DogLog.log("Swerve/Mod " + mod.moduleNumber + "/Position", positions[mod.moduleNumber]);
        }
        DogLog.log("Swerve/Module Positions", positions);
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
        Command currentCommand = getCurrentCommand();
        DogLog.log("Swerve/Current Command", currentCommand == null ? "None" : currentCommand.getName());

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