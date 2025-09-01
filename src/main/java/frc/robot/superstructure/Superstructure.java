package frc.robot.superstructure;

import static frc.robot.Options.optAlgaeBargeOnly;
import static frc.robot.Options.optInvertAlgae;
import static frc.robot.Options.optMirrorAuto;

import java.util.EnumMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedAlert;
import frc.lib.util.LoggedCommands;
import frc.lib.util.TunableOption;
import frc.robot.autos.AutoConstants;
import frc.robot.commands.pidswerve.PIDSwerve;
import frc.robot.commands.pidswerve.PIDSwerveConstants.PIDSpeed;
import frc.robot.subsystems.algaeroller.AlgaeRoller;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.controls.Controls;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ClearState;
import frc.robot.subsystems.elevator.ElevatorConstants.Stop;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.EndEffector.EEState;
import frc.robot.subsystems.endeffector.EndEffectorConstants.EEPosition;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.pose.Pose;
import frc.robot.subsystems.pose.PoseConstants;
import frc.robot.subsystems.pose.PoseConstants.ReefFace;
import frc.robot.Field.ReefLevel;

public class Superstructure extends SubsystemBase {
    public static final Superstructure instance = new Superstructure();
    private static final TunableOption optOverrideClimberTiming = new TunableOption("Override Climber Timing", false);
    private static final int climberTimeCutoff = 30; // seconds
    private ReefLevel activeReefLevel = ReefLevel.L4;

    public static enum EEPose {
        L1(EEPosition.L1, Stop.L1),
        L2(EEPosition.L23, Stop.L2),
        L3(EEPosition.L23, Stop.L3),
        L4(EEPosition.L4, Stop.L4),
        BARGE_PREP(EEPosition.BARGE, Stop.BARGE_PREP),
        BARGE(EEPosition.BARGE, Stop.BARGE),
        GROUND_CORAL(EEPosition.GROUND_INTAKE, Stop.STOW),
        GROUND_ALGAE(EEPosition.GROUND_INTAKE, Stop.FEED_ALGAE),
        REEF_INTAKE_L2(EEPosition.REEF_INTAKE, Stop.L2_ALGAE),
        REEF_INTAKE_L2_LIFT(EEPosition.REEF_INTAKE, Stop.L2_ALGAELIFT),
        REEF_INTAKE_L3(EEPosition.REEF_INTAKE, Stop.L3_ALGAE),
        REEF_INTAKE_L3_LIFT(EEPosition.REEF_INTAKE, Stop.L3_ALGAELIFT),
        PROCESSOR(EEPosition.GROUND_INTAKE, Stop.STOW),
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

        for (EEPose pose: EEPose.values()) {
            SmartDashboard.putData("Superstructure/Move EE to " + pose, TriggerMoveToEEPose(pose));
        }
    }

    public Command SetActiveReefLevel(ReefLevel level) {
        return LoggedCommands.runOnce("Change active reef level to " + level, () -> activeReefLevel = level);
    }

    public Command TriggerMoveToEEPose(EEPose pose) {
        return LoggedCommands.sequence("Move to EE Pose " + pose.name(),
            Commands.either(
                AlgaeRoller.instance.TriggerAtleastClear(),
                Commands.none(),
                () -> pose.stop.position.lte(Stop.CLEAR_HIGH.position) || !Elevator.instance.isClear(ClearState.CLEAR_HIGH)),
            EndEffector.instance.TriggerMoveTo(pose.position),
            Elevator.instance.TriggerMoveTo(pose.stop));
    }

    private Command TriggerMoveToEEPoseDirect(EEPose pose) {
        return LoggedCommands.sequence("Move directly to EE Pose " + pose.name(),
            EndEffector.instance.TriggerMoveTo(pose.position),
            Elevator.instance.TriggerMoveToDirect(pose.stop));
    }

    private Command TriggerMoveToActiveCoral() {
        return Commands.either(
            Commands.either(
                TriggerMoveToEEPoseDirect(EEPose.L1),
                Commands.either(
                    Commands.sequence(
                        AlgaeRoller.instance.TriggerStowWhenClear(),
                        TriggerMoveToEEPose(EEPose.L4)),
                    Commands.sequence(
                        AlgaeRoller.instance.TriggerStow(),
                        Commands.either(
                            TriggerMoveToEEPoseDirect(EEPose.L2),
                            TriggerMoveToEEPoseDirect(EEPose.L3),
                            () -> activeReefLevel == ReefLevel.L2)),
                    () -> activeReefLevel == ReefLevel.L4),
                () -> activeReefLevel == ReefLevel.L1),
            Commands.either(
                TriggerMoveToEEPose(EEPose.L1),
                Commands.either(
                    TriggerMoveToEEPose(EEPose.L4),
                    Commands.either(
                        TriggerMoveToEEPose(EEPose.L2),
                        TriggerMoveToEEPose(EEPose.L3),
                        () -> activeReefLevel == ReefLevel.L2),
                    () -> activeReefLevel == ReefLevel.L4),
                () -> activeReefLevel == ReefLevel.L1),
            () -> EndEffector.instance.inHighClearRange());
    }

    public Command WaitForEEPose() {
        return LoggedCommands.waitUntil("Wait for EE Pose", () -> EndEffector.instance.inPosition() && Elevator.instance.atTarget());
    }

    private void setFaceCommands(ReefFace face) {
        coralLeftCommands.put(face, ScoreCoral(face, true));
        coralRightCommands.put(face, ScoreCoral(face, false));
        deAlgaefyLeftCommands.put(face, DeAlgaefy(face));
        deAlgaefyRightCommands.put(face, DeAlgaefy(face));
    }
    
    public Command ScoreCoral(ReefFace face, boolean left) {
        return Commands.either(
            LoggedCommands.sequence("Auto Align " + (left ? "Left " : "Right ") + face.toString() + " & Score",
                LoggedCommands.parallel("PID Align " + (left ? "Left " : "Right ") + face.toString(),
                    Commands.sequence(
                        Commands.race(
                            Commands.either(
                                new PIDSwerve(Swerve.instance, Pose.instance, left ? face.alignCoralL1Left : face.alignCoralL1Right, true, true),
                                new PIDSwerve(Swerve.instance, Pose.instance, left ? face.alignCoralLeft : face.alignCoralRight, true, true),
                                () -> activeReefLevel == ReefLevel.L1),
                            Commands.either(
                                Commands.sequence(
                                    LoggedCommands.waitSeconds("Score coral watchdog", AutoConstants.scoreCoralTimeout),
                                    Commands.runOnce(() -> LoggedAlert.Error("Auto", "Timed out", "Timed out moving to score coral"))
                                ),
                                Commands.idle(),
                                () -> DriverStation.isAutonomousEnabled() && DriverStation.getMatchTime() >= (AutoConstants.scoreCoralTimeout + AutoConstants.scoreCoralTimeLeft))),
                        Swerve.instance.Stop()),
                    Commands.sequence(
                        TriggerMoveToActiveCoral(),
                        Commands.either(
                            AlgaeRoller.instance.TriggerL1Assist(), // TODO And wait?
                            AlgaeRoller.instance.TriggerStowWhenStopped(),
                            () -> activeReefLevel == ReefLevel.L1),
                        WaitForEEPose())),
                        // LoggedCommands.deadline("Wait for auto up",
                        //     Elevator.instance.WaitForNext(),
                        //     Elevator.instance.AutoElevatorUp(left ? face.alignCoralLeft.getTranslation() : face.alignCoralRight.getTranslation())))),
                PlaceCoral(),
                Commands.either(
                    Commands.none(),
                    TriggerMoveToEEPose(EEPose.GROUND_CORAL),
                    () -> EndEffector.instance.haveCoral()
                ),
                AlgaeRoller.instance.TriggerStowWhenStopped()),
            LoggedCommands.log("Cannot score coral without coral"),
            EndEffector.instance::haveCoral);
    }

    public Command DeAlgaefy(ReefFace face) {
        return DeAlgaefy(face, true);
    }

    public Command DeAlgaefy(ReefFace face, boolean extendedBackup) {
        EEPose algaePose = face.algaeHigh ? EEPose.REEF_INTAKE_L3 : EEPose.REEF_INTAKE_L2;
        EEPose algaeInvertPose = face.algaeHigh ? EEPose.REEF_INTAKE_L2 : EEPose.REEF_INTAKE_L3;
        EEPose algaeLiftPose = face.algaeHigh ? EEPose.REEF_INTAKE_L3_LIFT : EEPose.REEF_INTAKE_L2_LIFT;
        EEPose algaeInvertLiftPose = face.algaeHigh ? EEPose.REEF_INTAKE_L2_LIFT : EEPose.REEF_INTAKE_L3_LIFT;

        return LoggedCommands.sequence("Fully acquire Algae from " + face.toString(),
            LoggedCommands.deadline("Acquire Algae from " + face.toString(),
                Commands.sequence(
                    EndEffector.instance.WaitForAlgae(),
                    Controls.instance.TriggerRumble()),
                Commands.sequence(
                    LoggedCommands.deadline("Reef algae prep",
                        Commands.sequence(
                            EndEffector.instance.StartAlgaeIntake(),
                            Commands.either(
                                TriggerMoveToEEPose(algaeInvertPose),
                                TriggerMoveToEEPose(algaePose),
                                optInvertAlgae),
                            AlgaeRoller.instance.TriggerStowWhenStopped(),
                            WaitForEEPose()),
                        Commands.sequence(
                            new PIDSwerve(Swerve.instance, Pose.instance, face.approachAlgaeMiddle, true, false),
                            Swerve.instance.Stop()                            
                        )),
                    new PIDSwerve(Swerve.instance, Pose.instance, face.alignAlgae, true, true),
                    Swerve.instance.Stop())),
            Commands.either(
                TriggerMoveToEEPoseDirect(algaeInvertLiftPose),
                TriggerMoveToEEPoseDirect(algaeLiftPose),
                optInvertAlgae),    
            new PIDSwerve(Swerve.instance, Pose.instance, extendedBackup ? face.algaeBackupExtended : face.algaeBackupShort, true, false));
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

    public Command PrepBargeShot() {
        return LoggedCommands.sequence("Prepare barge shot",
        TriggerMoveToEEPose(EEPose.BARGE_PREP),
        AlgaeRoller.instance.TriggerStowWhenClear(),
        WaitForEEPose(),
        TriggerMoveToEEPoseDirect(EEPose.BARGE));
    }

    private Command ProcessorAlign() {
        return LoggedCommands.sequence("Align to processor",
            new PIDSwerve(Swerve.instance, Pose.instance, PoseConstants.processorApproach, true, false, PIDSpeed.FAST),
            new PIDSwerve(Swerve.instance, Pose.instance, PoseConstants.processorScore, true, true, PIDSpeed.FAST));
    }

    public Command SmartScore(boolean left) {
        return LoggedCommands.either("Smart Score",
            LoggedCommands.proxy(LoggedCommands.select("Coral select", left ? coralLeftCommands : coralRightCommands, () -> Pose.nearestFace(Pose.instance.getPose().getTranslation()))),
            Commands.either(
                Commands.either(
                    LoggedCommands.proxy(PrepBargeShot()),
                    LoggedCommands.proxy(ProcessorAlign()),
                    () -> { return optAlgaeBargeOnly.get() || !Pose.instance.nearProcessor(); }),
                LoggedCommands.proxy(Commands.select(left ? deAlgaefyLeftCommands : deAlgaefyRightCommands, () -> Pose.nearestFace(Pose.instance.getPose().getTranslation()))),
                EndEffector.instance::haveAlgae),
            EndEffector.instance::haveCoral);
    }

    public static Command IntakeAlgae() {
        return LoggedCommands.print("Intake Algae", "TODO Implement Intake Algae");
    }

    public Command PlaceCoral() {
        return LoggedCommands.either("Place Coral",
            PlaceL1Coral(),
            Commands.either(
                EndEffector.instance.ExpelCoral(ReefLevel.L4),
                Commands.either(
                    EndEffector.instance.ExpelCoral(ReefLevel.L2),
                    EndEffector.instance.ExpelCoral(ReefLevel.L3),
                    () -> activeReefLevel == ReefLevel.L2),
                () -> activeReefLevel == ReefLevel.L4),
            () -> activeReefLevel == ReefLevel.L1);
    }

    public Command PlaceL1Coral() {
        return LoggedCommands.sequence("Place L1 Coral",
            AlgaeRoller.instance.GuideL1Coral(),
            EndEffector.instance.ExpelCoral(ReefLevel.L1),
            AlgaeRoller.instance.StopIntake(),
            TriggerMoveToEEPose(EEPose.GROUND_CORAL),
            AlgaeRoller.instance.TriggerStowWhenStopped())
            .handleInterrupt(() -> {
                EndEffector.instance.StopIntake().schedule();
                // AlgaeRoller.instance.StopAndClear().schedule(); // HACK
            });
    }

    public Command PlacePiece() {
        return LoggedCommands.either("Place game piece",
            PlaceCoral(),
            PlaceBargeAlgae(),
            () -> EndEffector.instance.haveCoral());
    }

    public Command PlaceBargeAlgae() {
        return LoggedCommands.sequence("Place barge algae",
            EndEffector.instance.PlaceBargeAlgae(),
            TriggerMoveToEEPose(EEPose.GROUND_CORAL),
            AlgaeRoller.instance.TriggerStowWhenStopped());
    }

    public static Command WaitForCoral() {
        return LoggedCommands.print("Wait until coral", "TODO Implement Wait until coral is ready");
    }

    public static Command WaitForCoralReady() {
        return LoggedCommands.print("Wait until coral is ready", "TODO Implement Wait until coral is ready");
    }

    public Command CoralHold() {
        return LoggedCommands.sequence("Hold coral",
            TriggerMoveToEEPose(EEPose.CORAL_HOLD),
            AlgaeRoller.instance.TriggerStowWhenStopped());
    }

    public Command AlgaeHold() {
        return LoggedCommands.sequence("Hold algae",
            TriggerMoveToEEPose(EEPose.ALGAE_HOLD),
            AlgaeRoller.instance.TriggerStowWhenStopped());
    }

    public Command SmartCoralIntake() {
        return Commands.either(
            AlgaeHold(),
            Commands.either(
                CoralHold(),
                IntakeCoral(),
                () -> EndEffector.instance.haveCoral()),
            () -> EndEffector.instance.haveAlgae());
    }

    public Command SmartAlgaeIntake() {
        return Commands.either(
            AlgaeHold(),
            Commands.either(
                CoralHold(),
                IntakeGroundAlgae(),
                () -> EndEffector.instance.haveCoral()),
            () -> EndEffector.instance.haveAlgae());
    }

    private Command IntakeExpel = Intake.instance.Expel();

    public Command StartCoralIntake() {
        return LoggedCommands.sequence("Start Coral Intake",
            TriggerMoveToEEPose(EEPose.GROUND_CORAL),
            AlgaeRoller.instance.TriggerStowWhenStopped(),
            WaitForEEPose(),
            Commands.parallel(
                EndEffector.instance.StartCoralIntake(),
                Intake.instance.Deploy()));
    }

    public Command FinishCoralIntake() {
        return LoggedCommands.sequence("Finish Coral Intake",
        EndEffector.instance.StopIntake(),
        Intake.instance.Stop(),
        TriggerMoveToEEPose(EEPose.CORAL_HOLD),
        AlgaeRoller.instance.TriggerStowWhenStopped());
    }

    // TODO Use StartCoralIntake() && Finish Coral Intake
    private Command IntakeCoral() {
        // NOTE: Must not be holding any game piece already!
        return LoggedCommands.sequence("Intaking Coral",
            TriggerMoveToEEPose(EEPose.GROUND_CORAL),
            AlgaeRoller.instance.TriggerStowWhenStopped(),
            WaitForEEPose(),
            Commands.parallel(
                EndEffector.instance.StartCoralIntake(),
                Intake.instance.Deploy()),
            EndEffector.instance.WaitForState(EEState.HAVE_CORAL),
            TriggerMoveToEEPose(EEPose.CORAL_HOLD),
            AlgaeRoller.instance.TriggerStowWhenStopped(),
            Controls.instance.TriggerRumble())
            .finallyDo((interrupted) -> {
                IntakeExpel.schedule(); // TODO Make this a fixed command instead of new object?
                if (interrupted) {
                    // We didn't get coral, so stop the intake
                    EndEffector.instance.StopIntake().schedule();
                }
            });
    }

    public static Command StopIntake() {
        return LoggedCommands.parallel("Stopping coral intake",
            Intake.instance.Expel(),
            EndEffector.instance.StopIntake()
        );
    }

    public Command GrabCage() {
        // TODO Reject if holding coral (need to expel it first)
        return LoggedCommands.either("Grab cage",
            Commands.sequence(
                AlgaeRoller.instance.StopAndClear(),
                TriggerMoveToEEPose(EEPose.GROUND_CORAL),
                EndEffector.instance.StopIntake(),
                Intake.instance.Stop(),
                WaitForEEPose(),
                AlgaeRoller.instance.TriggerStow(),
                Climber.instance.GrabCage()),
            Commands.sequence(
                LoggedCommands.log("Cannot deploy before cutoff time (" + climberTimeCutoff + ")"),
                Commands.runOnce(() -> LoggedAlert.Error("Climber", "Too Early", "Cannot deploy before cutoff time"))),
            () -> optOverrideClimberTiming.get() || DriverStation.getMatchTime() <= climberTimeCutoff);
    }

    public Command Climb() {
        return LoggedCommands.sequence("Climb",
            Climber.instance.RetractAndWait(),
            Intake.instance.FullStow());
    }

    public Command IntakeGroundAlgae() {
        return LoggedCommands.sequence("Intake ground algae",
            Commands.deadline(
                EndEffector.instance.WaitForAlgae(),
                Commands.sequence(
                    AlgaeRoller.instance.TriggerDeploy(),
                    TriggerMoveToEEPose(EEPose.GROUND_ALGAE),
                    WaitForEEPose(),
                    EndEffector.instance.StartAlgaeIntake(),
                    AlgaeRoller.instance.TriggerDeploy(),
                    AlgaeRoller.instance.StartIntake())),
            AlgaeRoller.instance.StopIntake(),
            TriggerMoveToEEPose(EEPose.ALGAE_HOLD),
            AlgaeRoller.instance.TriggerStowWhenStopped())
            .handleInterrupt(() -> AlgaeRoller.instance.StopAndClear().schedule());
    }

    public ReefLevel activeReefLevel() {
        return activeReefLevel;
    }
}