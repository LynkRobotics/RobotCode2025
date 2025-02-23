package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedAlert;
import frc.lib.util.LoggedCommands;
import frc.lib.util.TunableOption;
import frc.robot.Constants;
import frc.robot.Constants.Elevator.Stop;

public class RobotState extends SubsystemBase {
    private static RobotState instance;
    private static PoseSubsystem pose;
    private static final CANdi candi;
    private static final DigitalInput indexSensor;
    private static boolean elevatorAtZero = false;
    private static GamePieceState gamePieceState = GamePieceState.NONE;
    private static final Timer L1Timer = new Timer();
    private static Stop activeStop = Stop.SAFE;
    private static Stop nextStop = Stop.SAFE;
    private static final Timer algaeScoreTimer = new Timer();

    private static final TunableOption optOverrideElevatorPathBlocked = new TunableOption("Override Elevator Path Blocked", false);
    private static final TunableOption optOverrideReefElevatorZone = new TunableOption("Override Reef Safe Elevator Zone", true);
    private static final TunableOption optOverrideElevatorDownAllowed = new TunableOption("Override Elevator Down Allowed", false);

    public enum GamePieceState {
        NONE,
        INTAKING_ALGAE,
        HOLDING_ALGAE,
        SCORING_ALGAE,
        INTAKING_CORAL,
        FEEDING_CORAL,
        ADVANCING_CORAL,
        HOLDING_CORAL,
        SCORING_CORAL
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
        return !haveAlgae() && (!getIntakeSensor() || !getFlipperSensor() || !getFinalSensor());
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
        return LoggedCommands.waitUntil("Wait for Coral Ready", () -> gamePieceState == GamePieceState.HOLDING_CORAL);
    }

    public static Command WaitForCoral() {
        return LoggedCommands.waitUntil("Wait for Coral", RobotState::haveCoral);
    }

    public static Command WaitForCoral(double timeout) {
        return LoggedCommands.race("Wait for Coral with timeout", WaitForCoral(), Commands.waitSeconds(timeout));
    }

    public static Command ScoreGamePiece() {
        return LoggedCommands.either("Score Game Piece",
            Commands.either(
                LoggedCommands.runOnce("Score Algae", () -> gamePieceState = GamePieceState.SCORING_ALGAE),
                LoggedCommands.runOnce("Cannot Score Algae", () -> LoggedAlert.Warning("Robot State", "Cannot Score", "Cannot score algae without holding algae")),
                () -> gamePieceState == GamePieceState.HOLDING_ALGAE),
            Commands.either(
                LoggedCommands.sequence("Score Coral",
                    LoggedCommands.runOnce("Initiate Scoring", () -> gamePieceState = GamePieceState.SCORING_CORAL),
                    LoggedCommands.waitUntil("Wait for Score", RobotState::getFinalSensor)),
                LoggedCommands.runOnce("Cannot Score Coral", () -> LoggedAlert.Warning("Robot State", "Cannot Score", "Cannot score coral without coral ready")),
                () -> gamePieceState == GamePieceState.HOLDING_CORAL),
            RobotState::haveAlgae);
    }

    public static Command IntakeAlgae() {
        // TODO Reject if we have coral
        return LoggedCommands.runOnce("Intake Algae", () -> gamePieceState = GamePieceState.INTAKING_ALGAE);
    }

    public static Command SetCoralMode() {
        return LoggedCommands.runOnce("Set Coral Mode", () -> gamePieceState = GamePieceState.NONE);
    }

    public static GamePieceState getGamePieceState() {
        return gamePieceState;
    }

    public static boolean haveAlgae() {
        return gamePieceState == GamePieceState.HOLDING_ALGAE || gamePieceState == GamePieceState.SCORING_ALGAE;
    }

    public static void setHaveAlgae() {
        if (gamePieceState == GamePieceState.NONE || gamePieceState == GamePieceState.INTAKING_ALGAE) {
            gamePieceState = GamePieceState.HOLDING_ALGAE;
        } else {
            LoggedAlert.Warning("Robot State", "Invalid Algae Transition", "Cannot hold algae from " + gamePieceState + " state");
        }
    }

    public static boolean coralReady() {
        return gamePieceState == GamePieceState.HOLDING_CORAL;
    }

    public static boolean coralScoring() {
        return gamePieceState == GamePieceState.SCORING_CORAL;
    }

    public static void setNoAlgae() {
        gamePieceState = GamePieceState.NONE;
    }

    public static void updateActiveStop(Stop stop) {
        activeStop = stop;
    }

    public static Stop getActiveStop() {
        return activeStop;
    }

    public static void updateNextStop(Stop stop) {
        nextStop = stop;
    }

    public static Stop getNextStop() {
        return nextStop;
    }

    @Override
    public void periodic() {
        boolean intakeSensor = getIntakeSensor();
        boolean flipperSensor = getFlipperSensor();
        boolean finalSensor = getFinalSensor();

        DogLog.log("State/Index sensor", intakeSensor);
        DogLog.log("State/Flipper sensor", flipperSensor);
        DogLog.log("State/Final sensor", finalSensor);
        DogLog.log("State/Game Piece State", gamePieceState);
        DogLog.log("State/Have coral", haveCoral());
        DogLog.log("State/Have algae", haveAlgae());

        SmartDashboard.putString("State/Active Game Piece", haveAlgae() ? "#48B6AB" : haveCoral() ? "#FFFFFF" : "#888888");

        if (!haveAlgae() && gamePieceState != GamePieceState.INTAKING_ALGAE) {
            if (gamePieceState == GamePieceState.SCORING_ALGAE) {
                if (!algaeScoreTimer.isRunning()) {
                    algaeScoreTimer.restart();
                } else if (algaeScoreTimer.get() > Constants.EndEffector.algaeRunTime) {
                    gamePieceState = GamePieceState.NONE;
                }
            } else if (intakeSensor && flipperSensor && finalSensor) {
                // No Coral is present
                if (gamePieceState == GamePieceState.SCORING_CORAL && activeStop == Stop.L1) {
                    if (!L1Timer.isRunning()) {
                        L1Timer.restart();
                    } else if (L1Timer.hasElapsed(Constants.EndEffector.L1RunTime)) {
                        L1Timer.stop();
                        gamePieceState = GamePieceState.NONE;
                    }
                } else {
                    gamePieceState = elevatorAtZero ? GamePieceState.INTAKING_CORAL : GamePieceState.NONE;
                }
            } else if (!intakeSensor && finalSensor) {
                gamePieceState = elevatorAtZero ? GamePieceState.FEEDING_CORAL : GamePieceState.NONE;
            } else if (!flipperSensor) {
                gamePieceState = GamePieceState.ADVANCING_CORAL;
            } else {
                assert(!finalSensor);
                if (gamePieceState != GamePieceState.SCORING_CORAL) {
                    gamePieceState = GamePieceState.HOLDING_CORAL;
                }
            }
        }
    }
}