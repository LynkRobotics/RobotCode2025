package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.commands.LoggedCommands;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private boolean speedLimit = false;

    public SwerveModule[] mSwerveMods;

    public Swerve() {
        Timer.delay(5); //Delaying the initalization of the swerve module should prevent a race condition with the CANcoders initializing, and causing just general funkiness
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
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

        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public Command alignStraight() {
        return LoggedCommands.runOnce("Align Swerve Modules",
            () -> {
                SwerveModuleState aligned = new SwerveModuleState(0.0, new Rotation2d());
                
                for(SwerveModule mod : mSwerveMods) {
                    mod.setDesiredState(aligned, false);
                }
            },
            this);
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

    public Command enableSpeedLimit() {
        return LoggedCommands.runOnce("Enabling Swerve Speed Limit",
            () -> {
                speedLimit = true;
            },
            this);
    }

    public Command disableSpeedLimit() {
        return LoggedCommands.runOnce("Disabling Swerve Speed Limit",
            () -> {
                speedLimit = false;
            },
            this);
    }

    public double getSpeedLimitRot() {
        return speedLimit ? Constants.Swerve.speedLimitRot : 1.0;
    }

    public Command setMotorsToCoast() {
        return LoggedCommands.runOnce("Setting All Swerve Motors To Coast",
            () -> {
                for(SwerveModule mod : mSwerveMods){
                    mod.setCoastMode();  
                }
            },
            this);
    }

    public Command setDriveMotorsToCoast() {
        return LoggedCommands.runOnce("Setting Drive Swerve Motors To Coast",
            () -> {
                for(SwerveModule mod : mSwerveMods){
                    mod.setCoastMode();  
                }
            },
            this);
    }

    public Command setMotorsToBrake() {
        return LoggedCommands.runOnce("Setting All Swerve Motors To Brake",
            () -> {
                for(SwerveModule mod : mSwerveMods){
                    mod.setBrakeMode();  
                }
            },
            this);
    }

    public Command setDriveMotorsToBrake() {
        return LoggedCommands.runOnce("Setting Drive Swerve Motors To Brake",
            () -> {
                for(SwerveModule mod : mSwerveMods){
                    mod.setBrakeMode();  
                }            },
            this);
    }

    public Command stopSwerve() {
        return LoggedCommands.runOnce("Stopping Swerve",
            () -> {
                drive(new Translation2d(0, 0), 0, false);
            },
            this);
    }

    @Override
    public void periodic() {
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Swerve/Mod/" + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            DogLog.log("Swerve/Mod/" + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Swerve/Mod/" + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            DogLog.log("Swerve/Mod/" + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Swerve/Mod/" + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
            DogLog.log("Swerve/Mod/" + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        DogLog.log("Swerve/Module States", getModuleStates());        
    }
}