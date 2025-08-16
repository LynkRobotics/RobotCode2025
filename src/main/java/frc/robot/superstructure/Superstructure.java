package frc.robot.superstructure;

import static frc.robot.Options.optAlgaeBargeOnly;
import static frc.robot.Options.optInvertAlgae;
import static frc.robot.Options.optMirrorAuto;

import java.util.EnumMap;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedAlert;
import frc.lib.util.LoggedCommands;
import frc.robot.autos.AutoConstants;
import frc.robot.commands.pidswerve.PIDSwerve;
import frc.robot.commands.pidswerve.PIDSwerveConstants.PIDSpeed;
import frc.robot.subsystems.algaeroller.AlgaeRoller;
import frc.robot.subsystems.controls.Controls;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.Stop;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.EndEffector.EEState;
import frc.robot.subsystems.endeffector.EndEffectorConstants.EEPosition;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.pose.Pose;
import frc.robot.subsystems.pose.PoseConstants;
import frc.robot.subsystems.pose.PoseConstants.ReefFace;
import frc.robot.subsystems.robotstate.RobotState;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants.CameraMode;

public class Superstructure extends SubsystemBase {
    public static final Superstructure instance = new Superstructure();

    public static enum EEPose {
        L1(EEPosition.L1, Stop.L1),
        L2(EEPosition.L2, Stop.L2),
        L3(EEPosition.L3, Stop.L3),
        L4(EEPosition.L4, Stop.L4),
        BARGE(EEPosition.BARGE, Stop.BARGE),
        GROUND_INTAKE(EEPosition.CORAL_INTAKE, Stop.STOW),
        REEF_INTAKE_L2(EEPosition.REEF_INTAKE, Stop.L2),
        REEF_INTAKE_L3(EEPosition.REEF_INTAKE, Stop.L3),
        // REEF_PREP
        PROCESSOR(EEPosition.PROCESSOR, Stop.STOW),
        ALGAE_HOLD(EEPosition.ALGAE_HOLD, Stop.ALGAE_HOLD),
        CORAL_HOLD(EEPosition.CORAL_HOLD, Stop.CORAL_HOLD),
        CLIMB(EEPosition.CLIMB, Stop.CLIMB);

        public EEPosition position;
		public Stop stop;

        EEPose(EEPosition position, Stop stop) {
            this.position = position;
            this.stop = stop;
        }
    }

    EnumMap<ReefFace, Command> coralLeftCommands = new EnumMap<>(ReefFace.class);
    EnumMap<ReefFace, Command> coralRightCommands = new EnumMap<>(ReefFace.class);
    EnumMap<ReefFace, Command> deAlgaefyLeftCommands = new EnumMap<>(ReefFace.class);
    EnumMap<ReefFace, Command> deAlgaefyRightCommands = new EnumMap<>(ReefFace.class);

    public Superstructure() {
        for (ReefFace face: ReefFace.values()) {
            setFaceCommands(face);
        }
    }

    private void setFaceCommands(ReefFace face) {
        coralLeftCommands.put(face, ScoreCoral(face, true));
        coralRightCommands.put(face, ScoreCoral(face, false));
        deAlgaefyLeftCommands.put(face, DeAlgaefy(face));
        deAlgaefyRightCommands.put(face, DeAlgaefy(face));
    }
    
    public static Command ScoreCoral(ReefFace face, boolean left) {
        return Commands.either(
            LoggedCommands.sequence("Auto Align " + (left ? "Left " : "Right ") + face.toString() + " & Score",
                LoggedCommands.parallel("PID Align " + (left ? "Left " : "Right ") + face.toString(),
                    Commands.sequence(
                        Vision.SwitchToFrontVision(),
                        Commands.race(
                            Commands.sequence(
                                Commands.either(
                                    Commands.sequence(
                                        new PIDSwerve(Swerve.instance, Pose.instance, left ? face.approachLeft : face.approachRight, true, false, PIDSpeed.TURBO),
                                        new PIDSwerve(Swerve.instance, Pose.instance, left ? face.alignLeft : face.alignRight, true, true)
                                    ),
                                    Commands.sequence(
                                        new PIDSwerve(Swerve.instance, Pose.instance, left ? face.approachLeft : face.approachRight, true, false, PIDSpeed.FAST), //, Constants.maxVisionDiffCoral),
                                        Commands.either(
                                            LoggedCommands.log("Elevator reached stop in time"),
                                            LoggedCommands.sequence("Pause to wait for elevator to catch up",
                                                Swerve.instance.Stop(),
                                                Elevator.instance.WaitForNearNext()),
                                            Elevator.instance::nearNextStop),
                                        new PIDSwerve(Swerve.instance, Pose.instance, left ? face.alignLeft : face.alignRight, true, true)
                                    ),
                                    () -> false)), //RobotState.getNextStop() == Stop.L2 || RobotState.getNextStop() == Stop.L3)),
                            Commands.either(
                                Commands.sequence(
                                    LoggedCommands.waitSeconds("Score coral watchdog", AutoConstants.scoreCoralTimeout),
                                    Commands.runOnce(() -> LoggedAlert.Error("Auto", "Timed out", "Timed out moving to score coral"))
                                ),
                                Commands.idle(),
                                () -> DriverStation.isAutonomousEnabled() && DriverStation.getMatchTime() >= (AutoConstants.scoreCoralTimeout + AutoConstants.scoreCoralTimeLeft))),
                        Swerve.instance.Stop()),
                    Commands.sequence(
                        Superstructure.WaitForCoralReady(),
                        LoggedCommands.deadline("Wait for auto up",
                            Elevator.instance.WaitForNext(),
                            Elevator.instance.AutoElevatorUp(left ? face.alignLeft.getTranslation() : face.alignRight.getTranslation())))),
                Superstructure.ScoreGamePiece()
            ),
            LoggedCommands.log("Cannot score coral without coral"),
            () -> RobotState.haveCoral())
        .handleInterrupt(() -> Vision.setCameraMode(CameraMode.DEFAULT));
    }

    public static Command DeAlgaefy(ReefFace face) {
        return DeAlgaefy(face, true);
    }

    // Safely move End Effector (and Elevator) to required pose
    // TODO Should this be a class?
    public static Command SafeEEPose(EEPosition position, Stop stop) {
        return LoggedCommands.print("Safe EE Pose", "TODO Implement Safe EE Pose");
    }

    public static Command DeAlgaefy(ReefFace face, boolean extendedBackup) {
        Stop algaeStop = face.algaeHigh ? Stop.L3_ALGAE: Stop.L2_ALGAE;
        Stop algaeInvertStop = face.algaeHigh ? Stop.L2_ALGAE : Stop.L3_ALGAE;

        return LoggedCommands.sequence("Fully acquire Algae from " + face.toString(),
            LoggedCommands.deadline("Acquire Algae from " + face.toString(),
                Commands.sequence(
                    EndEffector.instance.WaitForState(EEState.HAVE_ALGAE),
                    Controls.instance.TriggerRumble()),
                Commands.sequence(
                    Vision.SwitchToFrontVision(),
                    EndEffector.instance.StartAlgaeIntake(),
                    LoggedCommands.parallel("Prepare for reef algae intake",
                        Commands.sequence(
                            new PIDSwerve(Swerve.instance, Pose.instance, face.approachMiddle, true, false),
                            Swerve.instance.Stop()                            
                        ),
                        Commands.sequence(
                            Commands.either(
                                SafeEEPose(EEPosition.ALGAE_INTAKE, algaeInvertStop),
                                SafeEEPose(EEPosition.ALGAE_INTAKE, algaeStop),
                                optInvertAlgae
                            ),
                            AlgaeRoller.instance.Retract()
                        )),
                    new PIDSwerve(Swerve.instance, Pose.instance, face.alignMiddle, true, true),
                    Swerve.instance.Stop())),
            new PIDSwerve(Swerve.instance, Pose.instance, extendedBackup ? face.algaeBackupExtended : face.algaeBackupShort, true, false))
            .handleInterrupt(() -> {
                // if (!RobotState.haveAlgae()) RobotState.setNoAlgae();
                Vision.setCameraMode(CameraMode.DEFAULT);
            });
    }

    public Command SetStop(Stop stop) {
        return LoggedCommands.sequence("Set stop to " + stop,
            // RobotState.SetCoralMode(),
            Commands.runOnce(() -> Elevator.instance.setNextStop(stop)));
    }

    // Example of state?
    public boolean shouldMirror() {
        return optMirrorAuto.get() && DriverStation.isAutonomousEnabled();
    }

    public Command BargeShot() {
        return BargeShot(0.0);
    }

    public Command BargeShot(double adjustment) {
        return LoggedCommands.print("Barge shot", "TODO Implement barge shot");
    }

    private Command ProcessorAlign() {
        return LoggedCommands.sequence("Align to processor",
            new PIDSwerve(Swerve.instance, Pose.instance, PoseConstants.processorApproach, true, false, PIDSpeed.FAST),
            new PIDSwerve(Swerve.instance, Pose.instance, PoseConstants.processorScore, true, true, PIDSpeed.FAST));
    }

    public Command SmartScore(boolean left) {
        return Commands.either(
            LoggedCommands.proxy(Commands.select(left ? coralLeftCommands : coralRightCommands, () -> Pose.nearestFace(Pose.instance.getPose().getTranslation()))),
            Commands.either(
                Commands.either(
                    LoggedCommands.proxy(ProcessorAlign()),
                    LoggedCommands.proxy(BargeShot()),
                    () -> { return optAlgaeBargeOnly.get() || Pose.instance.nearProcessor(); }),
                LoggedCommands.proxy(Commands.select(left ? deAlgaefyLeftCommands : deAlgaefyRightCommands, () -> Pose.nearestFace(Pose.instance.getPose().getTranslation()))),
                RobotState::haveAlgae),
            RobotState::haveCoral);
    }

    public static Command IntakeAlgae() {
        return LoggedCommands.print("Intake Algae", "TODO Implement Intake Algae");
    }

    public static Command ScoreGamePiece() {
        return LoggedCommands.print("Score Game Piece", "TODO Implement Score Game Piece");
    }

    public static Command WaitForCoral() {
        return LoggedCommands.print("Wait until coral", "TODO Implement Wait until coral is ready");
    }

    public static Command WaitForCoralReady() {
        return LoggedCommands.print("Wait until coral is ready", "TODO Implement Wait until coral is ready");
    }
}