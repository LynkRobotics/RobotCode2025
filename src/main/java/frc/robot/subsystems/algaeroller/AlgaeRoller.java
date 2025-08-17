package frc.robot.subsystems.algaeroller;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedCommands;
import frc.robot.Ports;
import frc.robot.subsystems.algaeroller.AlgaeRollerContants.AlgaeRollerPosition;

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
        deployMotor = new TalonFX(Ports.ALGAE_DEPLOY.id, Ports.ALGAE_DEPLOY.bus.name);
        deployMotor.getConfigurator().apply(AlgaeRollerContants.getDeployMotorConfig());
        rollerMotor = new TalonFX(Ports.ALGAE_ROLLERS.id, Ports.ALGAE_ROLLERS.bus.name);
        rollerMotor.getConfigurator().apply(AlgaeRollerContants.getRollerMotorConfig());

        deployMotor.setPosition(AlgaeRollerPosition.STOWED.position);
        deployMotor.setControl(AlgaeRollerPosition.STOWED.control);

        SmartDashboard.putData("Algae Roller/Move to STOWED", LoggedCommands.runOnce("Move to STOWED", () -> moveTo(AlgaeRollerPosition.STOWED), this));
        SmartDashboard.putData("Algae Roller/Move to PROCESSOR", LoggedCommands.runOnce("Move to PROCESSOR", () -> moveTo(AlgaeRollerPosition.PROCESSOR), this));
        SmartDashboard.putData("Algae Roller/Move to CLEAR", LoggedCommands.runOnce("Move to CLEAR", () -> moveTo(AlgaeRollerPosition.CLEAR), this));
        SmartDashboard.putData("Algae Roller/Move to L1_SCORE", LoggedCommands.runOnce("Move to L1_SCORE", () -> moveTo(AlgaeRollerPosition.L1_SCORE), this));
        SmartDashboard.putData("Algae Roller/Move to DEPLOYED", LoggedCommands.runOnce("Move to DEPLOYED", () -> moveTo(AlgaeRollerPosition.DEPLOYED), this));
    }

    // TODO Add Zero() method / Command

    public static boolean fullyDeployed() {
        return false; // TODO
    }

    public boolean isClear() {
        return deployMotor.getPosition().getValue().gte(AlgaeRollerPosition.CLEAR.position.plus(AlgaeRollerContants.epsilon));
    }

    private void moveTo(AlgaeRollerPosition position) {
        DogLog.log("Algae Roller/Status", "Moving to " + position.name());
        deployMotor.setControl(position.control);
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
        DogLog.log("Algae Roller/Deploy Position (rotations)", deployMotor.getPosition().getValue().in(Units.Rotations));
        DogLog.log("Algae Roller/Deploy Position (degress)", deployMotor.getPosition().getValue().in(Units.Degrees));
        DogLog.log("Algae Roller/Intake Current", rollerMotor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("Algae Roller/Intake Velocity", rollerMotor.getVelocity().getValueAsDouble());
    }
}