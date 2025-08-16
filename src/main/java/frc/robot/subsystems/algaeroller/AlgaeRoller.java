package frc.robot.subsystems.algaeroller;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedCommands;

public class AlgaeRoller extends SubsystemBase {
    public static final AlgaeRoller instance = new AlgaeRoller();

    /* Devices */
    private final TalonFX deployMotor;
    private final TalonFX rollerMotor;
    
    /* Control Requests */    
    private final VoltageOut intakeControl = new VoltageOut(AlgaeRollerContants.intakeVoltage).withEnableFOC(true);
    private final VoltageOut expelControl = new VoltageOut(AlgaeRollerContants.expelVoltage).withEnableFOC(true);
    private final VoltageOut L1AssistControl = new VoltageOut(AlgaeRollerContants.L1AssistVoltage).withEnableFOC(true);
    
    AlgaeRoller() {
        /* Devices */
        deployMotor = new TalonFX(AlgaeRollerContants.deployMotorID, AlgaeRollerContants.canBus);
        deployMotor.getConfigurator().apply(AlgaeRollerContants.getDeployMotorConfig());
        rollerMotor = new TalonFX(AlgaeRollerContants.rollerMotorID, AlgaeRollerContants.canBus);
        rollerMotor.getConfigurator().apply(AlgaeRollerContants.getRollerMotorConfig());
    }

    public static boolean fullyDeployed() {
        return false; // TODO
    }

    public static boolean isClear() {
        return false; // TODO
    }

    public static Command Intake() {
        return LoggedCommands.print("Intake algae", "TODO Implement intake algae");
    }

    public Command Retract() {
        return LoggedCommands.print("Retract algae roller", "TODO Implement retract algae roller");
    }

    public static Command GuideL1Coral() {
        return LoggedCommands.print("Guide L1 coral", "TODO Implement guide L1 coral");
    }

    public static Command ClearElevatorPath() {
        return LoggedCommands.print("Clear elevator path", "TODO Implement clear elevator path");
    }

    @Override
    public void periodic() {
        Command currentCommand = getCurrentCommand();
        DogLog.log("Algae Roller/Current Command", currentCommand == null ? "None" : currentCommand.getName());
        DogLog.log("Algae Roller/Fully Deployed", fullyDeployed());
        DogLog.log("Algae Roller/Deploy Current", deployMotor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("Algae Roller/Deploy Velocity", deployMotor.getVelocity().getValueAsDouble());
        DogLog.log("Algae Roller/Deploy Position", deployMotor.getPosition().getValueAsDouble());
        DogLog.log("Algae Roller/Intake Current", rollerMotor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("Algae Roller/Intake Velocity", rollerMotor.getVelocity().getValueAsDouble());
    }

}