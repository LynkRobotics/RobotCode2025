package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedAlert;
import frc.lib.util.TunableOption;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommands;

public class RobotState extends SubsystemBase {
    private static RobotState instance;
    private static PoseSubsystem pose;
    private static final CANdi candi;
    private static final DigitalInput indexSensor;
    private static boolean elevatorAtZero = false;
    private static CoralState coralState = CoralState.REJECTING;
    private static AlgaeState algaeState = AlgaeState.NONE;

    private static final TunableOption optOverrideElevatorPathBlocked = new TunableOption("Override Elevator Path Blocked", false);
    private static final TunableOption optOverrideReefElevatorZone = new TunableOption("Override Reef Safe Elevator Zone", true);
    private static final TunableOption optOverrideElevatorDownAllowed = new TunableOption("Override Elevator Down Allowed", false);

    public enum CoralState {
        REJECTING,
        INTAKING,
        FEEDING,
        ADVANCING,
        READY,
        SCORING
    }

    public enum AlgaeState {
        NONE,
        INTAKING,
        HOLDING,
        SCORING
    }

    static {
        indexSensor = new DigitalInput(Constants.indexSensorID);
        candi = new CANdi(Constants.candiID, Constants.candiBus);

        applyConfigs();
    }
    
    public RobotState() {
        if (instance == null) {
            instance = this;
        } else {
            System.out.println("Multiple RobotState instances detected!");
            System.exit(1);
        }
    }

    public static RobotState getInstance() {
        return instance;
    }

    public static void applyConfigs() {
        var CANdiConfig = new CANdiConfiguration();

        CANdiConfig.DigitalInputs.S1CloseState = S1CloseStateValue.CloseWhenNotLow;
        CANdiConfig.DigitalInputs.S2CloseState = S2CloseStateValue.CloseWhenNotLow;

        candi.getConfigurator().apply(CANdiConfig);
    }

    public static boolean getIntakeSensor() {
        return indexSensor.get();
    }

    public static boolean getFinalSensor() {
        return candi.getS1Closed().getValue();
    }
    
    public static boolean getFlipperSensor() {
        return candi.getS2Closed().getValue();
    }

    public static boolean haveCoral() {
        return algaeState == AlgaeState.NONE && (!getIntakeSensor() || !getFlipperSensor() || !getFinalSensor());
    }

    public static boolean elevatorPathBlocked() {
        return (!getIntakeSensor() || !getFlipperSensor()) && !optOverrideElevatorPathBlocked.get();
    }

    // TODO Consider allowing a range, so that Elevator doesn't oscillate  
    public static boolean raisedElevatorAllowable() {
        if (pose == null) pose = PoseSubsystem.getInstance();
        if (pose == null) return false;
        return (pose.inReefElevatorZone() || optOverrideReefElevatorZone.get()) && pose.isUpright();
    }
    public static boolean elevatorDownAllowed() {
        if (pose == null) pose = PoseSubsystem.getInstance();
        if (pose == null) return true;
        return pose.elevatorDownAllowed() || optOverrideElevatorDownAllowed.get();
    }
    
    public static void setElevatorAtZero(boolean atZero) {
        elevatorAtZero = atZero;
        DogLog.log("State/Elevator at zero", elevatorAtZero);
    }

    public static boolean getElevatorAtZero() {
        return elevatorAtZero;
    }

    public static Command WaitForCoralReady() {
        return LoggedCommands.waitUntil("Wait for Coral Ready", () -> coralState == CoralState.READY);
    }

    public static Command ScoreGamePiece() {
        return LoggedCommands.either("Score Game Piece",
            Commands.either(
                LoggedCommands.runOnce("Score Algae", () -> algaeState = AlgaeState.SCORING),
                LoggedCommands.runOnce("Cannot Score Algae", () -> LoggedAlert.Warning("Robot State", "Cannot Score", "Cannot score algae without holding algae")),
                () -> algaeState == AlgaeState.HOLDING),
            Commands.either(
                LoggedCommands.sequence("Score Coral",
                    LoggedCommands.runOnce("Initiate Scoring", () -> coralState = CoralState.SCORING),
                    LoggedCommands.waitUntil("Wait for Score", RobotState::getFinalSensor)),
                LoggedCommands.runOnce("Cannot Score Coral", () -> LoggedAlert.Warning("Robot State", "Cannot Score", "Cannot score coral without coral ready")),
                () -> coralState == CoralState.READY),
            RobotState::haveAlgae);
    }

    public static Command IntakeAlgae() {
        return LoggedCommands.runOnce("Intake Algae", () -> algaeState = AlgaeState.INTAKING);
    }

    public static Command SetCoralMode() {
        return LoggedCommands.runOnce("Set Coral Mode", () -> algaeState = AlgaeState.NONE);
    }

    public static CoralState getCoralState() {
        return coralState;
    }

    public static AlgaeState getAlgaeState() {
        return algaeState;
    }

    public static boolean haveAlgae() {
        return algaeState == AlgaeState.HOLDING || algaeState == AlgaeState.SCORING;
    }

    public static void setHaveAlgae() {
        if (algaeState == AlgaeState.NONE || algaeState == AlgaeState.INTAKING) {
            algaeState = AlgaeState.HOLDING;
        }
    }

    public static void setNoAlgae() {
        algaeState = AlgaeState.NONE;
    }

    @Override
    public void periodic() {
        boolean intakeSensor = getIntakeSensor();
        boolean flipperSensor = getFlipperSensor();
        boolean finalSensor = getFinalSensor();

        DogLog.log("State/Index sensor", intakeSensor);
        DogLog.log("State/Flipper sensor", flipperSensor);
        DogLog.log("State/Final sensor", finalSensor);
        DogLog.log("State/Coral State", coralState);
        DogLog.log("State/Algae State", algaeState);

        SmartDashboard.putString("State/Active Game Piece", haveAlgae() ? "#48B6AB" : haveCoral() ? "#FFFFFF" : "#A0A0A0");

        if (algaeState != AlgaeState.NONE) {
            coralState = CoralState.REJECTING;
        } else if (intakeSensor && flipperSensor && finalSensor) {
            // No Coral is present
            coralState = elevatorAtZero ? CoralState.INTAKING : CoralState.REJECTING;
        } else if (!intakeSensor && finalSensor) {
            coralState = elevatorAtZero ? CoralState.FEEDING : CoralState.REJECTING;
        } else if (!flipperSensor) {
            coralState = CoralState.ADVANCING;
        } else {
            assert(!finalSensor);
            if (coralState != CoralState.SCORING) {
                coralState = CoralState.READY;
            }
        }
    }
}