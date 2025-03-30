package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;

import dev.doglog.DogLog;
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
import static frc.robot.Options.optAlgaeBargeOnly;

public class RobotState extends SubsystemBase {
    private static RobotState instance;
    private static PoseSubsystem pose;
    private static final CANrange funnelSensor;
    private static final CANrange indexSensor;
    private static final CANrange flipperSensor;
    private static final CANrange finalSensor;
    private static boolean elevatorAtZero = false;
    private static GamePieceState gamePieceState = GamePieceState.NONE;
    private static ClimbState climbState = ClimbState.NONE;
    private static final Timer L1Timer = new Timer();
    private static Stop activeStop = Stop.SAFE;
    private static Stop nextStop = Stop.SAFE;
    private static final Timer algaeScoreTimer = new Timer();
    private static final Timer unjamTimer = new Timer();
    private static final Timer algaeScoredTimer = new Timer();
    private static boolean turboMode = false;

    private static final TunableOption optOverrideElevatorPathBlocked = new TunableOption("Override Elevator Path Blocked", true);
    private static final TunableOption optOverrideReefElevatorZone = new TunableOption("Override Reef Safe Elevator Zone", true);
    private static final TunableOption optOverrideElevatorDownAllowed = new TunableOption("Override Elevator Down Allowed", false);

    public enum GamePieceState {
        NONE,
        INTAKING_ALGAE,
        HOLDING_ALGAE,
        SCORING_BARGE_ALGAE,
        SCORING_PROCESSOR_ALGAE,
        INTAKING_CORAL,
        UNJAMMING_CORAL,
        FEEDING_CORAL,
        ADVANCING_CORAL,
        HOLDING_CORAL,
        SCORING_CORAL
    }

    public enum ClimbState {
        NONE,
        STARTED,
        CLIMBING,
        CLIMBED
    }

    static {
        funnelSensor = new CANrange(4, "rio");
        indexSensor = new CANrange(1, "rio");
        flipperSensor = new CANrange(2, "rio");
        finalSensor = new CANrange(3, "rio");
        
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
        var CANrangeConfig = new CANrangeConfiguration(); //should there be different configs for different canranges
        CANrangeConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        CANrangeConfig.FovParams.FOVCenterX = 11.8; // Maximum
        CANrangeConfig.FovParams.FOVCenterY = 11.8; // Maximum
        CANrangeConfig.FovParams.FOVRangeX = 6.75; // Minimum
        CANrangeConfig.FovParams.FOVRangeY = 6.75; // Minimum
        CANrangeConfig.ProximityParams.ProximityThreshold = 0.25;
        CANrangeConfig.ProximityParams.ProximityHysteresis = 0.02;
        flipperSensor.getConfigurator().apply(CANrangeConfig); //should these be different configs?

        CANrangeConfig.FovParams.FOVCenterX = 0; // Reset to default
        CANrangeConfig.FovParams.FOVCenterY = 0; // Reset to default
        finalSensor.getConfigurator().apply(CANrangeConfig); //should these be different configs?

        CANrangeConfig.ProximityParams.ProximityThreshold = 0.10;
        indexSensor.getConfigurator().apply(CANrangeConfig);

        CANrangeConfig.FovParams.FOVCenterX = -11.8; // Maximum
        CANrangeConfig.FovParams.FOVCenterY = -11.8; // Maximum
        CANrangeConfig.FovParams.FOVRangeX = 27.0; // Maximum
        CANrangeConfig.ProximityParams.ProximityThreshold = 0.37;
        CANrangeConfig.ProximityParams.ProximityHysteresis = 0.05;
        funnelSensor.getConfigurator().apply(CANrangeConfig);
    }

    /*
     * NOTE: For compatibility, we invert the sensor results
     * TODO Revisit
     */

    public static boolean getFunnelSensor() {
        return !funnelSensor.getIsDetected().getValue();
    }

    public static boolean getIndexSensor() {
        return !indexSensor.getIsDetected().getValue();
    }

    public static boolean getFinalSensor() {
        return !finalSensor.getIsDetected().getValue();
    }
    
    public static boolean getFlipperSensor() {
        return !flipperSensor.getIsDetected().getValue();
    }

    public static void setTurboMode(boolean turbo) {
        turboMode = turbo;
    }

    public static boolean getTurboMode() {
        return turboMode;
    }

    public static boolean haveCoral() {
        return !haveAlgae() && (!getFunnelSensor() ||!getIndexSensor() || !getFlipperSensor() || !getFinalSensor());
    }

    public static boolean elevatorPathBlocked() {
        return (!getIndexSensor() || !getFlipperSensor()) && !optOverrideElevatorPathBlocked.get();
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

    public static boolean algaeToProcessor() {
        if (pose == null) pose = PoseSubsystem.getInstance();
        if (pose == null) return false;

        return !optAlgaeBargeOnly.get() && pose.nearProcessor();
    }

    public static Command ScoreGamePiece() {
        return LoggedCommands.either("Score Game Piece",
            Commands.either(
                Commands.either(
                    LoggedCommands.runOnce("Score Algae into Processor", () -> gamePieceState = GamePieceState.SCORING_PROCESSOR_ALGAE),
                    LoggedCommands.runOnce("Score Algae into Barge", () -> gamePieceState = GamePieceState.SCORING_BARGE_ALGAE),
                    RobotState::algaeToProcessor),
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
        return gamePieceState == GamePieceState.HOLDING_ALGAE || gamePieceState == GamePieceState.SCORING_BARGE_ALGAE || gamePieceState == GamePieceState.SCORING_PROCESSOR_ALGAE;
    }

    public static boolean scoredAlgaeRecently() {
        return algaeScoredTimer.isRunning() && !algaeScoredTimer.hasElapsed(Constants.algaeScoredTimeout);
    }

    public static boolean intakingAlgae() {
        return gamePieceState == GamePieceState.INTAKING_ALGAE;
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

    private static void unjamCoral() {
        gamePieceState = GamePieceState.UNJAMMING_CORAL;
    }

    public static Command UnjamCoral() {
        return LoggedCommands.runOnce("Unjam Coral", () -> unjamCoral());
    }

    public static void setClimbState(ClimbState state) {
        climbState = state;
    }

    public static ClimbState getClimbState() {
        return climbState;
    }

    @Override
    public void periodic() {
        boolean funnelSensor = getFunnelSensor();
        boolean indexSensor = getIndexSensor();
        boolean flipperSensor = getFlipperSensor();
        boolean finalSensor = getFinalSensor();

        if (gamePieceState == GamePieceState.SCORING_BARGE_ALGAE || gamePieceState == GamePieceState.SCORING_PROCESSOR_ALGAE) {
            if (!algaeScoreTimer.isRunning()) {
                algaeScoreTimer.restart();
                algaeScoredTimer.restart();
            } else if (algaeScoreTimer.get() > Constants.EndEffector.algaeRunTime) {
                gamePieceState = GamePieceState.NONE;
                algaeScoreTimer.stop();
            }
        } else if (gamePieceState != GamePieceState.HOLDING_ALGAE && gamePieceState != GamePieceState.INTAKING_ALGAE) {
            if (gamePieceState == GamePieceState.UNJAMMING_CORAL) {
                if (!unjamTimer.isRunning()) {
                    unjamTimer.restart();
                } else if (unjamTimer.hasElapsed(Constants.Index.unjamTime)) {
                    unjamTimer.stop();
                    gamePieceState = GamePieceState.NONE;
                }
            } else if (indexSensor && flipperSensor && finalSensor) {
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
            } else if (!indexSensor && finalSensor) {
                // Coral is in indexer and not at final sensor
                gamePieceState = elevatorAtZero ? GamePieceState.FEEDING_CORAL : GamePieceState.NONE;
            } else if (!flipperSensor) {
                // Coral is blocking flipper sensor -- could be either at index or at final sensor)
                if (indexSensor && gamePieceState == GamePieceState.SCORING_CORAL) {
                    // Just continue scoring, even if coral "slid back"
                } else {
                    gamePieceState = GamePieceState.ADVANCING_CORAL;
                }
            } else {
                assert(!finalSensor);
                if (gamePieceState != GamePieceState.SCORING_CORAL) {
                    gamePieceState = GamePieceState.HOLDING_CORAL;
                }
            }
        }

        DogLog.log("State/Funnel sensor", funnelSensor);
        DogLog.log("State/Index sensor", indexSensor);
        DogLog.log("State/Flipper sensor", flipperSensor);
        DogLog.log("State/Final sensor", finalSensor);
        DogLog.log("State/Game Piece State", gamePieceState);
        DogLog.log("State/Climb State", climbState);
        DogLog.log("State/Have coral", haveCoral());
        DogLog.log("State/Have algae", haveAlgae());

        SmartDashboard.putString("State/Active Game Piece", haveAlgae() ? "#48B6AB" : haveCoral() ? "#FFFFFF" : "#888888");
        SmartDashboard.putBoolean("State/Coral Ready", coralReady());
    }
}