package frc.lib.util;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.math.Conversions;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveConstants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0).withEnableFOC(true);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0).withEnableFOC(true);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0).withEnableFOC(true);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID, moduleConstants.canBusID);
        DogLog.log("Debug/Swerve", "Module " + moduleNumber + " before applying config: " + String.format("%1.3f", getCANcoder().getDegrees()) + " [ target " + String.format("%1.3f", angleOffset.getDegrees()) + " ]");
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);
        DogLog.log("Debug/Swerve", "Module " + moduleNumber + " immediately after config: " + String.format("%1.3f", getCANcoder().getDegrees()) + " [ target " + String.format("%1.3f", angleOffset.getDegrees()) + " ]");

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, moduleConstants.canBusID);
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute();
        DogLog.log("Debug/Swerve", "Module " + moduleNumber + " after config: " + String.format("%1.3f", getCANcoder().getDegrees()) + " [ target " + String.format("%1.3f", angleOffset.getDegrees()) + " ]");
        // Timer.delay(1.0);
        // DogLog.log("Debug/Swerve", "Module " + moduleNumber + " long after config: " + String.format("%1.3f", getCANcoder().getDegrees()) + " [ target " + String.format("%1.3f", angleOffset.getDegrees()) + " ]");

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, moduleConstants.canBusID);
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState.optimize(getState().angle); //TODO Test
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / SwerveConstants.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, SwerveConstants.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    public double alignmentError() {
        return getCANcoder().minus(angleOffset).getDegrees();
    }

    public boolean isAligned() {
        return Math.abs(alignmentError()) <= SwerveConstants.maxAngleError;
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void resetToAbsolute(){
        /*
         * Theory is that the CANcoder gets initialized and "freaks out" giving bad abs values when this function is called in the
         * initialization of the swerve subsystem, to maybe prevent this we give a little time to let the CANcoders "settle" and give
         * good numbers preventing us from having to restart robot code every single time
         * PLEASE WORK, IVE BEEN TRYING TO FIX YOU FOR 3 YEARS
         */
        Timer.delay(0.1); //Waiting for CanCoders to get actual values to prevent race conditions
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();  //Getting the cancoder values
        mAngleMotor.setPosition(absolutePosition); //Set angle motor to CANcoder value
    }

    public void setCoastMode(){
        mDriveMotor.setNeutralMode(NeutralModeValue.Coast);
        mAngleMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setDriveCoastMode(){
        mDriveMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setBrakeMode(){
        mDriveMotor.setNeutralMode(NeutralModeValue.Brake);
        mAngleMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setDriveBrakeMode(){
        mDriveMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValueAsDouble(), SwerveConstants.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble())
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getPosition().getValueAsDouble(), SwerveConstants.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble())
        );
    }
}
