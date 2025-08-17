// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedAlert;
import frc.lib.util.LoggedCommands;
import frc.robot.Field;
import frc.robot.Ports;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ClearState;
import frc.robot.subsystems.endeffector.EndEffectorConstants.EEControl;
import frc.robot.subsystems.endeffector.EndEffectorConstants.EEPosition;

public class EndEffector extends SubsystemBase {
    public static final EndEffector instance = new EndEffector();

    public enum EEState {
        EMPTY,
        HAVE_ALGAE,
        HAVE_CORAL
    }
    public EEState state = EEState.EMPTY;

    public enum EEIntakeState {
        STOPPED,
        INTAKING_ALGAE,
        SCORING_ALGAE,
        INTAKING_CORAL,
        SCORING_CORAL
    }
    public EEIntakeState intakeState = EEIntakeState.STOPPED;
    
    private EEPosition desiredPosition = EEPosition.START;
    private boolean atDesiredPosition = true;
    private boolean waitingToPivot = false;

    /* Devices */
    private final TalonFX positionMotor;
    private final TalonFX pieceMotor;
    private final CANdi candi;
    private final CANcoder directCancoder;
    private final CANcoder gearedCancoder;

    /* Control Requests */
    // private final VoltageOut algaeControl = new VoltageOut(EndEffectorConstants.algaeVoltage).withEnableFOC(false);

    public EndEffector() {
        /* Devices */
        positionMotor = new TalonFX(Ports.EE_POSITION.id, Ports.EE_POSITION.bus.name);
        pieceMotor = new TalonFX(Ports.EE_PIECE.id, Ports.EE_PIECE.bus.name);
        candi = new CANdi(Ports.EE_CANDI.id, Ports.EE_CANDI.bus.name);
        directCancoder = new CANcoder(Ports.ENCODER_41T.id, Ports.ENCODER_41T.bus.name);
        gearedCancoder = new CANcoder(Ports.ENCODER_40T.id, Ports.ENCODER_40T.bus.name);

        /* Configs */
        positionMotor.getConfigurator().apply(EndEffectorConstants.getPositionConfig());
        pieceMotor.getConfigurator().apply(EndEffectorConstants.getPieceConfig());
        candi.getConfigurator().apply(EndEffectorConstants.getCANdiConfig());
        directCancoder.getConfigurator().apply(EndEffectorConstants.getDirect41TCancoderConfig());
        gearedCancoder.getConfigurator().apply(EndEffectorConstants.getGeared40TCancoderConfig());

        // directCancoder.setPosition(directCancoder.getAbsolutePosition().getValue());
        // setCurrentPosition(directCancoder.getAbsolutePosition().getValue());
    }

    public boolean inPosition() {
        return atDesiredPosition;
    }

    public Command ExpelCoral(Field.ReefLevel level) {
        EEControl control = switch (level) {
            case L4 -> EEControl.CORAL_L4;
            case L3 -> EEControl.CORAL_L3;
            case L2 -> EEControl.CORAL_L2;
            case L1 -> EEControl.CORAL_L1;
        };
        double postClearDelay = switch (level) {
            case L2, L3 -> 0.18;
            default -> 0.05;  
        };

        return LoggedCommands.sequence("Expel coral for " + level,
            Commands.runOnce(() -> pieceMotor.setControl(control.control), this),
            // TODO Wait for beam break
            Commands.waitSeconds(postClearDelay),
            Commands.runOnce(pieceMotor::stopMotor, this));
    }

    public Command WaitForState(EEState desiredState) {
        return LoggedCommands.waitUntil("Wait for EE state " + desiredState, () -> state == desiredState );
    }

    public Command StartAlgaeIntake() {
        return LoggedCommands.runOnce("Start algae intake",
            () ->{
                if (state != EEState.EMPTY) {
                    LoggedAlert.Error("End Effector", "Bad state", "End Effector state was not empty: " + state);
                    state = EEState.EMPTY;
                }
                intakeState = EEIntakeState.INTAKING_ALGAE;
                pieceMotor.setControl(EEControl.ALGAE_INTAKE.control);
            }, this);
    }

    public Command StartCoralIntake() {
        return LoggedCommands.runOnce("Start coral intake",
            () ->{
                if (state != EEState.EMPTY) {
                    LoggedAlert.Error("End Effector", "Bad state", "End Effector state was not empty: " + state);
                    state = EEState.EMPTY;
                }
                intakeState = EEIntakeState.INTAKING_CORAL;
                pieceMotor.setControl(EEControl.CORAL_INTAKE.control);
            }, this);
    }

    public void move(EEPosition position) {
        if (desiredPosition == position) {
            // No change
            return;
        }
        
        desiredPosition = position;
        atDesiredPosition = false;
        if (Elevator.instance.isClear(ClearState.CLEAR_LOW)) {
            waitingToPivot = false;
            // TODO positionMotor.setControl();
        } else {
            waitingToPivot = true;
        }
    }

    public Command StopIntake() {
        return LoggedCommands.runOnce("Stop intake", () -> pieceMotor.stopMotor(), this);
    }

    public boolean coralDetected() {
        return !candi.getS1Closed().getValue();
    }

    public boolean algaeDetected() {
        return !candi.getS2Closed().getValue();
    }

    public Angle getAbsolutePosition() {
		Angle positionRemainder = directCancoder.getAbsolutePosition().getValue();
		Angle gearedEncoderPos = gearedCancoder.getAbsolutePosition().getValue();
		Angle gearedEncoder0RotsPosition = positionRemainder.times(EndEffectorConstants.gearedCancoderGearing);
		Angle diff = gearedEncoderPos.minus(gearedEncoder0RotsPosition);
		Angle diffRemainder = Units.Radians.of(MathUtil.angleModulus(diff.in(Units.Radians)));
		long fullRotations = Math.round(diffRemainder.in(Units.Rotations) / (EndEffectorConstants.gearedCancoderGearing - 1.0));
		Angle absolutePosition = positionRemainder.plus(Units.Rotations.of(fullRotations));
		return absolutePosition;
	}

    @Override
    public void periodic() {
        Command currentCommand = getCurrentCommand();
        Angle position = positionMotor.getPosition().getValue();
        Angle epsilon = Units.Rotations.of(0.5); // TODO How close until we say we're at the desired position

        DogLog.log("EndEffector/Current Command", currentCommand == null ? "None" : currentCommand.getName());
        DogLog.log("EndEffector/Desired Position", desiredPosition.name());
        DogLog.log("EndEffector/In Position", inPosition());
        DogLog.log("EndEffector/Position Motor/TorqueCurrent", positionMotor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("EndEffector/Position Motor/StatorCurrent", positionMotor.getStatorCurrent().getValueAsDouble());
        DogLog.log("EndEffector/Position Motor/Velocity", positionMotor.getVelocity().getValueAsDouble());
        DogLog.log("EndEffector/Position Motor/RotorVelocity", positionMotor.getRotorVelocity().getValueAsDouble());
        DogLog.log("EndEffector/Position Motor/Motor Temp", positionMotor.getDeviceTemp().getValueAsDouble());
        DogLog.log("EndEffector/Position Motor/Position (degrees)", position.in(Units.Degrees));
        DogLog.log("EndEffector/Piece Motor/TorqueCurrent", pieceMotor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("EndEffector/Piece Motor/StatorCurrent", pieceMotor.getStatorCurrent().getValueAsDouble());
        DogLog.log("EndEffector/Piece Motor/Velocity", pieceMotor.getVelocity().getValueAsDouble());
        DogLog.log("EndEffector/Piece Motor/RotorVelocity", pieceMotor.getRotorVelocity().getValueAsDouble());
        DogLog.log("EndEffector/Piece Motor/Motor Temp", pieceMotor.getDeviceTemp().getValueAsDouble());

        DogLog.log("EndEffector/CANdi connected", candi.isConnected());
        DogLog.log("EndEffector/Coral detected", coralDetected());
        DogLog.log("EndEffector/Algae detected", algaeDetected());

        DogLog.log("EndEffector/State", state.name());
        DogLog.log("EndEffector/Intake State", intakeState.name());

        DogLog.log("EndEffector/CANcoder Top 41T Abs Pos (deg)", directCancoder.getAbsolutePosition().getValue().in(Units.Degrees));
        DogLog.log("EndEffector/CANcoder Top 41T Abs Pos (rot)", directCancoder.getAbsolutePosition().getValue().in(Units.Rotations));
        DogLog.log("EndEffector/CANcoder Bot 40T Abs Pos (deg)", gearedCancoder.getAbsolutePosition().getValue().in(Units.Degrees));
        DogLog.log("EndEffector/CANcoder Bot 40T Abs Pos (rot)", gearedCancoder.getAbsolutePosition().getValue().in(Units.Rotations));
        DogLog.log("EndEffector/CANcoder Top 41T Pos (deg)", directCancoder.getPosition().getValue().in(Units.Degrees));
        DogLog.log("EndEffector/CANcoder Top 41T Pos (rot)", directCancoder.getPosition().getValue().in(Units.Rotations));
        DogLog.log("EndEffector/CANcoder Bot 40T Pos (deg)", gearedCancoder.getPosition().getValue().in(Units.Degrees));
        DogLog.log("EndEffector/CANcoder Bot 40T Pos (rot)", gearedCancoder.getPosition().getValue().in(Units.Rotations));

        DogLog.log("EndEffector/Absolute position (deg)", getAbsolutePosition().in(Units.Degrees));
        DogLog.log("EndEffector/Absolute position (rot)", getAbsolutePosition().in(Units.Rotations));

        if (!atDesiredPosition) {
            // TODO Debounce?
            if (desiredPosition.position.minus(position).abs(Units.Degrees) < epsilon.in(Units.Degrees)) {
                atDesiredPosition = true;
            }
        }
        if (waitingToPivot) {
            // TODO Do we need to worry about algae roller deploy?
            if (atDesiredPosition) {
                // Unclear how we'd get here, but if we are in position, there's no need to wait
                waitingToPivot = false;
            } else if (Elevator.instance.isClear(Elevator.ClearState.CLEAR_LOW)) {
                // If the elevator is clear, we can pivot
                waitingToPivot = false;
                // TODO positionMotor.setControl();
            }
        }
        
        if (intakeState == EEIntakeState.INTAKING_ALGAE) {
            if (false) { // TODO Detect algae grab; remember to use debounce; wait 0.2s after detection (velocity below 2000 degrees per second or beam break)
                state = EEState.HAVE_ALGAE;
                pieceMotor.setControl(EEControl.ALGAE_HOLD.control);
                intakeState = EEIntakeState.STOPPED;
            }
        } else if (intakeState == EEIntakeState.INTAKING_CORAL) {
            // TODO Also check to stall?
            if (coralDetected()) { // TODO Use debounce
                state = EEState.HAVE_CORAL;
                pieceMotor.setControl(EEControl.CORAL_HOLD.control);
                intakeState = EEIntakeState.STOPPED;
            }
        }
    }
}