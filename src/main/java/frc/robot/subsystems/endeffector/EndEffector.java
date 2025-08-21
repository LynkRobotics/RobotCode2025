// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedAlert;
import frc.lib.util.LoggedCommands;
import frc.robot.Field;
import frc.robot.Ports;
import frc.robot.subsystems.algaeroller.AlgaeRoller;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffectorConstants.EEControl;
import frc.robot.subsystems.endeffector.EndEffectorConstants.EEPosition;

public class EndEffector extends SubsystemBase {
    public static final EndEffector instance = new EndEffector();

    public enum EEState {
        EMPTY,
        HAVE_ALGAE,
        HAVE_CORAL
    }
    private EEState state = EEState.EMPTY;

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

    private Debouncer coralDebouncer = new Debouncer(EndEffectorConstants.coralSensorDebounce.in(Units.Seconds), Debouncer.DebounceType.kBoth);
    private Debouncer algaeDebouncer = new Debouncer(EndEffectorConstants.algaeSensorDebounce.in(Units.Seconds), Debouncer.DebounceType.kBoth);

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

        Angle directAbsPosition = directCancoder.getAbsolutePosition().getValue();
        directCancoder.setPosition(directAbsPosition);
        positionMotor.setPosition(directAbsPosition);

        // For basic debugging
        for (EEPosition position : EEPosition.values()) {
            SmartDashboard.putData("EndEffector/Move to " + position, LoggedCommands.runOnce("Move to " + position, () ->moveTo(position), this));;
        }
        SmartDashboard.putData("EndEffector/Expel Coral L3", ExpelCoral(Field.ReefLevel.L3));
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
            LoggedCommands.waitUntil("Wait for coral to clear", () -> state == EEState.EMPTY),
            Commands.waitSeconds(postClearDelay),
            Commands.runOnce(pieceMotor::stopMotor, this));
    }

    public boolean haveCoral() {
        return state == EEState.HAVE_CORAL;
    }

    public boolean haveAlgae() {
        return state == EEState.HAVE_ALGAE;
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

    public void moveTo(EEPosition position) {
        if (desiredPosition == position) {
            // No change
            return;
        }
        
        desiredPosition = position;
        atDesiredPosition = false;
        if (okToMove()) {
            waitingToPivot = false;
            positionMotor.setControl(desiredPosition.control);
        } else {
            waitingToPivot = true;
        }
    }

    public Command TriggerMoveTo(EEPosition position) {
        return LoggedCommands.runOnce("Move to EE position " + position, () -> moveTo(position), this);
    }

    public Command StopIntake() {
        return LoggedCommands.runOnce("Stop intake", () -> pieceMotor.stopMotor(), this);
    }

    private boolean coralDetectedRaw() {
        return !candi.getS1Closed().getValue();
    }

    private boolean coralDetected() {
        return coralDebouncer.calculate(coralDetectedRaw());
    }

    private boolean algaeDetectedRaw() {
        return !candi.getS2Closed().getValue();
    }

    private boolean algaeDetected() {
        return algaeDebouncer.calculate(algaeDetectedRaw());
    }

    private boolean okToMove() {
        return Elevator.instance.isClear(Elevator.ClearState.CLEAR_LOW) &&
            (Elevator.instance.isClear(Elevator.ClearState.CLEAR_HIGH) || AlgaeRoller.instance.isClear());
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

    public Command WaitForAlgae() {
        return LoggedCommands.waitUntil("Wait for algae intake", () -> state == EEState.HAVE_ALGAE);
    }

    @Override
    public void periodic() {
        Command currentCommand = getCurrentCommand();
        Angle position = positionMotor.getPosition().getValue();

        DogLog.log("EndEffector/Current Command", currentCommand == null ? "None" : currentCommand.getName());
        DogLog.log("EndEffector/Desired Position", desiredPosition.name());
        DogLog.log("EndEffector/In Position", inPosition());
        DogLog.log("EndEffector/Waiting to Pivot", waitingToPivot);
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
            if (desiredPosition.position.minus(position).abs(Units.Degrees) < EndEffectorConstants.pivotEpsilon.in(Units.Degrees)) {
                atDesiredPosition = true;
                // Hack if necessary to reseed the desired position
                // if (desiredPosition == EEPosition.GROUND_INTAKE) {
                //     positionMotor.setPosition(directCancoder.getAbsolutePosition().getValue());            
                // }
            }
        }
        if (waitingToPivot) {
            if (atDesiredPosition) {
                // Unclear how we'd get here, but if we are in position, there's no need to wait
                waitingToPivot = false;
            } else if (okToMove()) {
                // Conditionals now allow for movement
                waitingToPivot = false;
                positionMotor.setControl(desiredPosition.control);
            }
        }
        
        if (intakeState == EEIntakeState.INTAKING_ALGAE) {
            // TODO Also check stall
            if (algaeDetected()) { // TODO Wait 0.2s before/after detection? can't we just bump the debounce up?
                state = EEState.HAVE_ALGAE;
                pieceMotor.setControl(EEControl.ALGAE_HOLD.control);
                intakeState = EEIntakeState.STOPPED;
            }
        } else if (intakeState == EEIntakeState.INTAKING_CORAL) {
            // TODO Also check to stall?
            if (coralDetected()) {
                state = EEState.HAVE_CORAL;
                pieceMotor.setControl(EEControl.CORAL_HOLD.control);
                intakeState = EEIntakeState.STOPPED;
            }
        }
        if (state == EEState.HAVE_CORAL) {
            if (!coralDetected()) {
                state = EEState.EMPTY;
                DogLog.log("EndEffector/Status", "Coral lost");
                pieceMotor.stopMotor();
            }
        } else if (state == EEState.HAVE_ALGAE) {
            if (!algaeDetected()) {
                state = EEState.EMPTY;
                DogLog.log("EndEffector/Status", "Algae lost");
            }
        } else {
            if (coralDetected()) {
                state = EEState.HAVE_CORAL;
                DogLog.log("EndEffector/Status", "Coral surprisingly detected");
                pieceMotor.setControl(EEControl.CORAL_HOLD.control);
            } else if (algaeDetected()) {
                state = EEState.HAVE_ALGAE;
                DogLog.log("EndEffector/Status", "Algae surprisingly detected");
                pieceMotor.setControl(EEControl.ALGAE_HOLD.control);
            }
        }
    }
}