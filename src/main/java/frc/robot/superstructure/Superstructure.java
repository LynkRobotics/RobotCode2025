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
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants.CameraMode;
import frc.robot.Field.ReefLevel;;

public class Superstructure extends SubsystemBase {
    public static final Superstructure instance = new Superstructure();
    private static ReefLevel activeReefLevel = ReefLevel.L4;

    public static enum EEPose {
        L1(EEPosition.L1, Stop.L1),
        L2(EEPosition.L23, Stop.L2),
        L3(EEPosition.L23, Stop.L3),
        L4(EEPosition.L4, Stop.L4),
        BARGE(EEPosition.BARGE, Stop.BARGE),
        GROUND_INTAKE(EEPosition.GROUND_INTAKE, Stop.STOW),
        REEF_INTAKE_L2(EEPosition.REEF_INTAKE, Stop.L2),
        REEF_INTAKE_L3(EEPosition.REEF_INTAKE, Stop.L3),
        // REEF_PREP
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
            // SmartDashboard.putData("Superstructure/Move EE to " + pose, LoggedCommands.runOnce("Move EE to " + pose, () ->moveTo(pose),
            //     AlgaeRoller.instance, EndEffector.instance, Elevator.instance));
            SmartDashboard.putData("Superstructure/Move EE to " + pose, TriggerMoveToEEPose(pose));
        }

        SmartDashboard.putData("Align C", LoggedCommands.sequence("Align C",
            new PIDSwerve(Swerve.instance, Pose.instance, ReefFace.CD.alignCoralLeft, true, true),
            Swerve.instance.Stop()));
        SmartDashboard.putData("Align D", LoggedCommands.sequence("Align D",
            new PIDSwerve(Swerve.instance, Pose.instance, ReefFace.CD.alignCoralRight, true, true),
            Swerve.instance.Stop()));
        SmartDashboard.putData("Align CD algae", LoggedCommands.sequence("Align D",
            new PIDSwerve(Swerve.instance, Pose.instance, ReefFace.CD.alignAlgaeMiddle, true, true),
            Swerve.instance.Stop()));
    }

    public static Command SetActiveReefLevel(ReefLevel level) {
        return LoggedCommands.runOnce("Change active reef level to " + level, () -> activeReefLevel = level);
    }

    private Command TriggerMoveToEEPose(EEPose pose) {
        return LoggedCommands.sequence("Move to EE Pose " + pose.name(),
            Commands.either(
                AlgaeRoller.instance.TriggerAtleastClear(),
                Commands.none(),
                () -> pose.stop.position.lte(Stop.CLEAR_HIGH.position) || !Elevator.instance.isClear(ClearState.CLEAR_HIGH)),
            EndEffector.instance.TriggerMoveTo(pose.position),
            Elevator.instance.TriggerMoveTo(pose.stop));
    }

    private Command TriggerMoveToActiveCoral() {
        return Commands.either(
            TriggerMoveToEEPose(EEPose.L2),
            Commands.either(
                TriggerMoveToEEPose(EEPose.L3),
                TriggerMoveToEEPose(EEPose.L4),
                () -> activeReefLevel == ReefLevel.L3),
            () -> activeReefLevel == ReefLevel.L2);
    }

    private Command WaitForEEPose() {
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
                            Commands.sequence(
                                Commands.either(
                                    Commands.sequence(
                                        // new PIDSwerve(Swerve.instance, Pose.instance, left ? face.approachCoralLeft : face.approachCoralRight, true, false, PIDSpeed.TURBO),
                                        new PIDSwerve(Swerve.instance, Pose.instance, left ? face.alignCoralLeft : face.alignCoralRight, true, true)
                                    ),
                                    Commands.sequence(
                                        new PIDSwerve(Swerve.instance, Pose.instance, left ? face.approachCoralLeft : face.approachCoralRight, true, false, PIDSpeed.FAST), //, Constants.maxVisionDiffCoral),
                                        Commands.either(
                                            LoggedCommands.log("Elevator reached stop in time"),
                                            LoggedCommands.sequence("Pause to wait for elevator to catch up",
                                                Swerve.instance.Stop(),
                                                Elevator.instance.WaitForNearNext()),
                                            Elevator.instance::nearNextStop),
                                        new PIDSwerve(Swerve.instance, Pose.instance, left ? face.alignCoralLeft : face.alignCoralRight, true, true)
                                    ),
                                    () -> true)), //RobotState.getNextStop() == Stop.L2 || RobotState.getNextStop() == Stop.L3)),
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
                        AlgaeRoller.instance.TriggerStowWhenStopped(),
                        WaitForEEPose())),
                        // LoggedCommands.deadline("Wait for auto up",
                        //     Elevator.instance.WaitForNext(),
                        //     Elevator.instance.AutoElevatorUp(left ? face.alignCoralLeft.getTranslation() : face.alignCoralRight.getTranslation())))),
                Superstructure.PlaceCoral(),
                TriggerMoveToEEPose(EEPose.GROUND_INTAKE),
                AlgaeRoller.instance.TriggerStowWhenStopped()
            ),
            LoggedCommands.log("Cannot score coral without coral"),
            EndEffector.instance::haveCoral);
    }

    public static Command DeAlgaefy(ReefFace face) {
        return DeAlgaefy(face, true);
    }

    // Safely move End Effector (and Elevator) to required pose
    // TODO Should this be a class?
    public static Command SafeEEPose(EEPose pose) {
        return LoggedCommands.print("Safe EE Pose", "TODO Implement Safe EE Pose for " + pose.name());
    }

    public static Command DeAlgaefy(ReefFace face, boolean extendedBackup) {
        EEPose algaePose = face.algaeHigh ? EEPose.REEF_INTAKE_L3 : EEPose.REEF_INTAKE_L2;
        EEPose algaeInvertPose = face.algaeHigh ? EEPose.REEF_INTAKE_L2 : EEPose.REEF_INTAKE_L3;

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
                            new PIDSwerve(Swerve.instance, Pose.instance, face.approachAlgaeMiddle, true, false),
                            Swerve.instance.Stop()                            
                        ),
                        Commands.sequence(
                            Commands.either(
                                SafeEEPose(algaeInvertPose),
                                SafeEEPose(algaePose),
                                optInvertAlgae
                            ),
                            AlgaeRoller.instance.TriggerStowWhenStopped()
                        )),
                    new PIDSwerve(Swerve.instance, Pose.instance, face.alignAlgaeMiddle, true, true),
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
                EndEffector.instance::haveAlgae),
            EndEffector.instance::haveCoral);
    }

    public static Command IntakeAlgae() {
        return LoggedCommands.print("Intake Algae", "TODO Implement Intake Algae");
    }

    public static Command PlaceCoral() {
        return LoggedCommands.either("Place Coral",
            EndEffector.instance.ExpelCoral(ReefLevel.L2),
            Commands.either(
                EndEffector.instance.ExpelCoral(ReefLevel.L3),
                EndEffector.instance.ExpelCoral(ReefLevel.L4),
                () -> activeReefLevel == ReefLevel.L3),
            () -> activeReefLevel == ReefLevel.L2);
    }

    public static Command WaitForCoral() {
        return LoggedCommands.print("Wait until coral", "TODO Implement Wait until coral is ready");
    }

    public static Command WaitForCoralReady() {
        return LoggedCommands.print("Wait until coral is ready", "TODO Implement Wait until coral is ready");
    }

    public Command CoralHold() {
        return LoggedCommands.print("Coral hold", "TODO Implement Coral hold");
    }

    public Command AlgaeHold() {
        return LoggedCommands.print("Algae hold", "TODO Implement Algae hold");
    }

    public Command SmartIntake() {
        return Commands.either(
            AlgaeHold(),
            Commands.either(
                CoralHold(),
                IntakeCoral(),
                () -> EndEffector.instance.haveCoral()),
            () -> EndEffector.instance.haveAlgae());
    }

    private Command IntakeExpel = Intake.instance.Expel();

    private Command IntakeCoral() {
        // NOTE: Must not be holding any game piece already!
        return LoggedCommands.sequence("Intaking Coral",
            TriggerMoveToEEPose(EEPose.GROUND_INTAKE),
            WaitForEEPose(),
            Commands.parallel(
                EndEffector.instance.StartCoralIntake(),
                Intake.instance.Deploy()),
            EndEffector.instance.WaitForState(EEState.HAVE_CORAL),
            TriggerMoveToEEPose(EEPose.CORAL_HOLD),
            AlgaeRoller.instance.TriggerStowWhenStopped(), // Will happen asynchronously as soon as possible
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
        return LoggedCommands.sequence("Grab cage",
            // Ensure all rollers idle
            TriggerMoveToEEPose(EEPose.GROUND_INTAKE),
            Intake.instance.Stop(), // TODO Only if no coral in index
            WaitForEEPose(),
            AlgaeRoller.instance.TriggerStow(),
            Climber.instance.GrabCage());
    }

    public Command IntakeGroundAlgae() {
        return LoggedCommands.sequence("Intake ground algae",
            Commands.deadline(
                EndEffector.instance.WaitForAlgae(),
                Commands.sequence(
                    AlgaeRoller.instance.TriggerDeploy(),
                    TriggerMoveToEEPose(EEPose.GROUND_INTAKE),
                    WaitForEEPose(),
                    EndEffector.instance.StartAlgaeIntake(),
                    AlgaeRoller.instance.StartIntake())),
            AlgaeRoller.instance.StopIntake(),
            TriggerMoveToEEPose(EEPose.ALGAE_HOLD),
            AlgaeRoller.instance.TriggerStowWhenStopped())
            .handleInterrupt(() -> AlgaeRoller.instance.StopAndClear().schedule());
    }
}