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
    private static GamePiece activePiece = GamePiece.CORAL;
    private static boolean elevatorAtZero = false;
    private static CoralState coralState = CoralState.REJECTING;
    private static boolean haveAlgae = false;

    private static final TunableOption optOverrideElevatorPathBlocked = new TunableOption("Override Elevator Path Blocked", false);
    private static final TunableOption optOverrideReefElevatorZone = new TunableOption("Override Reef Safe Elevator Zone", true);
    private static final TunableOption optOverrideElevatorDownAllowed = new TunableOption("Override Elevator Down Allowed", false);

    public enum GamePiece {
        CORAL,
        ALGAE
    };

    public enum CoralState {
        REJECTING,
        INTAKING,
        FEEDING,
        ADVANCING,
        READY,
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
        return activePiece == GamePiece.CORAL && (!getIntakeSensor() || !getFlipperSensor() || !getFinalSensor());
    }

    public static boolean elevatorPathBlocked() {
        return (!getIntakeSensor() || !getFlipperSensor()) && !optOverrideElevatorPathBlocked.get();
    }

    public static void setActiveGamePiece(GamePiece piece) {
        activePiece = piece;
        DogLog.log("State/Game piece", activePiece);
    }

    public static GamePiece getActiveGamePiece() {
        return activePiece;
    }

    public static Command SwitchGamePiece(GamePiece piece) {
        return LoggedCommands.runOnce("Switch Game Piece to " + piece, () -> setActiveGamePiece(piece));
    }

    public static void toggleGamePiece() {
        setActiveGamePiece(activePiece == GamePiece.CORAL ? GamePiece.ALGAE : GamePiece.CORAL);
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

    public static Command ToggleGamePiece() {
        return LoggedCommands.runOnce("Toggle Game Piece", RobotState::toggleGamePiece);
    }

    public static Command WaitForCoralReady() {
        return LoggedCommands.waitUntil("Wait for Coral Ready", () -> coralState == CoralState.READY);
    }

    public static Command ScoreGamePiece() {
        // TODO Support Algae -- does Algae need a state? (NONE, INTAKE, HOLD, SCORE, etc.) rather than a haveAlgae boolean
        return Commands.either(
            LoggedCommands.sequence("Score Game Piece",
                LoggedCommands.runOnce("Initiate Scoring", () -> coralState = CoralState.SCORING),
                LoggedCommands.waitUntil("Wait for Score", RobotState::getFinalSensor)),
            LoggedCommands.runOnce("Cannot Score", () -> LoggedAlert.Warning("Robot State", "Cannot Score", "Cannot score coral without coral ready")),
            () -> coralState == CoralState.READY);
    }

    public static CoralState getCoralState() {
        return coralState;
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

        SmartDashboard.putString("State/Active Game Piece", getActiveGamePiece() == GamePiece.CORAL ? "#FFFFFF" : "#48B6AB");

        if (activePiece != GamePiece.CORAL) {
            assert(activePiece == GamePiece.ALGAE);
            coralState = CoralState.REJECTING;
            if (finalSensor) {
                haveAlgae = true;
            } else {
                // Only revert to coral if we have algae and then lose it
                if (haveAlgae) {
                    // TODO Do we need to allow time to score algae first?
                    // Maybe have the scoring trigger lock the state until released?
                    haveAlgae = false;
                    activePiece = GamePiece.CORAL;
                }
            }
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