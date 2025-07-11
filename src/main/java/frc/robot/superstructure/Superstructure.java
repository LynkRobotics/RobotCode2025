package frc.robot.superstructure;

import static frc.robot.Options.optAutoReefAiming;
import static frc.robot.Options.optL1Outside;
import static frc.robot.Options.optInvertAlgae;
import static frc.robot.Options.optMirrorAuto;

import java.util.EnumMap;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedAlert;
import frc.lib.util.LoggedCommands;
import frc.robot.autos.AutoConstants;
import frc.robot.commands.pidswerve.PIDSwerve;
import frc.robot.commands.pidswerve.PIDSwerveConstants.PIDSpeed;
import frc.robot.subsystems.controls.Controls;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.Stop;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.pose.Pose;
import frc.robot.subsystems.pose.PoseConstants;
import frc.robot.subsystems.pose.PoseConstants.Cage;
import frc.robot.subsystems.pose.PoseConstants.ReefFace;
import frc.robot.subsystems.robotstate.RobotState;
import frc.robot.subsystems.robotstate.RobotState.ClimbState;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants.CameraMode;

public class Superstructure extends SubsystemBase {
    public static final Superstructure instance = new Superstructure();

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
        coralLeftCommands.put(face, Superstructure.instance.ScoreCoral(face, true));
        coralRightCommands.put(face, Superstructure.instance.ScoreCoral(face, false));
        deAlgaefyLeftCommands.put(face, Superstructure.instance.DeAlgaefy(face));
        deAlgaefyRightCommands.put(face, Superstructure.instance.DeAlgaefy(face));
    }
    
    public Command ScoreCoral(ReefFace face, boolean left) {
        return Commands.either(
            LoggedCommands.sequence("Auto Align " + (left ? "Left " : "Right ") + face.toString() + " & Score",
                LoggedCommands.parallel("PID Align " + (left ? "Left " : "Right ") + face.toString(),
                    Commands.sequence(
                        Vision.SwitchToFrontVision(),
                        Commands.race(
                            Commands.sequence(
                                Commands.either(
                                    Commands.sequence(
                                        // Commands.either(
                                            // LoggedCommands.log("Skipping approach pose for L2/L3 due to proximity to reef"),
                                            new PIDSwerve(Swerve.instance, Pose.instance, left ? face.approachLeft : face.approachRight, true, false, PIDSpeed.TURBO),
                                            // new PIDSwerve(Swerve.instance, Pose.instance, left ? face.approachLeft : face.approachRight, true, false, PIDSpeed.FAST),
                                            // () -> Pose.instance.getPose().getTranslation().getDistance(Constants.Pose.reefCenter) <= Constants.Pose.approachDistanceToReefCenter),
                                        // new PIDSwerve(Swerve.instance, Pose.instance, left ? face.alignBonusLeft : face.alignBonusRight, true, true)
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
                                        Commands.either(
                                            Commands.either(
                                                new PIDSwerve(Swerve.instance, Pose.instance, left ? face.leftL1Outside : face.rightL1Outside, true, true),
                                                new PIDSwerve(Swerve.instance, Pose.instance, left ? face.leftL1 : face.rightL1, true, true),
                                                optL1Outside::get),
                                            new PIDSwerve(Swerve.instance, Pose.instance, left ? face.alignLeft : face.alignRight, true, true),
                                            () -> RobotState.getNextStop() == Stop.L1
                                        )),
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
                        Commands.either(
                            LoggedCommands.log("Skip waiting for coral ready in turbo mode"),
                            RobotState.WaitForCoralReady(),
                            RobotState::getTurboMode),
                        LoggedCommands.deadline("Wait for auto up",
                            Elevator.instance.WaitForNext(),
                            Commands.either(
                                Commands.either(
                                    Elevator.instance.AutoElevatorUp(left ? face.leftL1Outside.getTranslation() : face.rightL1Outside.getTranslation()),
                                    Elevator.instance.AutoElevatorUp(left ? face.leftL1.getTranslation() : face.rightL1.getTranslation()),
                                    optL1Outside::get),
                                Elevator.instance.AutoElevatorUp(left ? face.alignLeft.getTranslation() : face.alignRight.getTranslation()),
                                // Elevator.instance.SmoothElevatorUp(left ? face.approachLeft.getTranslation() : face.approachRight.getTranslation()), // TODO Not always Smooth!
                                () -> RobotState.getNextStop() == Stop.L1)))),
                Commands.parallel(
                    RobotState.ScoreGamePiece(),
                    Commands.either(
                        Commands.sequence(
                            Commands.waitSeconds(ElevatorConstants.L1RaiseDelay),
                            Commands.parallel(
                                Elevator.instance.TimeBasedMove(Stop.L1_SCORE, 0.25),
                                Commands.sequence(
                                    Commands.waitSeconds(0.30),
                                    Commands.either(
                                        new PIDSwerve(Swerve.instance, Pose.instance, (left ? face.leftL1Outside : face.rightL1Outside).transformBy(new Transform2d(PoseConstants.L1MoveForward, left ? Units.inchesToMeters(0) : Units.inchesToMeters(0), Rotation2d.kZero)), true, true, PIDSpeed.FAST),
                                        new PIDSwerve(Swerve.instance, Pose.instance, (left ? face.leftL1 : face.rightL1).transformBy(new Transform2d(PoseConstants.L1MoveForward, 0, Rotation2d.kZero)), true, true, PIDSpeed.FAST),
                                        optL1Outside)))
                        ),
                        Commands.none(),
                        () -> RobotState.getNextStop() == Stop.L1
                    )
                )),
            LoggedCommands.log("Cannot score coral without coral"),
            () -> RobotState.haveCoral() || RobotState.getTurboMode())
        .handleInterrupt(() -> Vision.setCameraMode(CameraMode.DEFAULT));
    }

    public Command DeAlgaefy(ReefFace face) {
        return DeAlgaefy(face, true);
    }

    public Command DeAlgaefy(ReefFace face, boolean extendedBackup) {
        Stop algaeStop = face.algaeHigh ? Stop.L3_ALGAE: Stop.L2_ALGAE;
        Stop algaeInvertStop = face.algaeHigh ? Stop.L2_ALGAE : Stop.L3_ALGAE;

        return LoggedCommands.sequence("Fully acquire Algae from " + face.toString(),
            LoggedCommands.deadline("Acquire Algae from " + face.toString(),
                Commands.sequence(
                    LoggedCommands.waitUntil("Wait for Algae", RobotState::haveAlgae),
                    Controls.instance.TriggerRumble()),
                LoggedCommands.sequence("Auto Align Middle " + face.toString(),
                    Vision.SwitchToFrontVision(),
                    RobotState.IntakeAlgae(),
                    LoggedCommands.parallel("PID Align Middle " + face.toString(),
                        Commands.sequence(
                            new PIDSwerve(Swerve.instance, Pose.instance, face.approachMiddle, true, false),
                            new PIDSwerve(Swerve.instance, Pose.instance, face.alignMiddle, true, true),
                            Swerve.instance.Stop()),
                        Commands.either(
                            LoggedCommands.deadline("Wait for auto up to " + algaeInvertStop,
                                Elevator.instance.WaitForStop(algaeInvertStop),
                                Elevator.instance.AutoElevatorUp(face.alignMiddle.getTranslation(), algaeInvertStop)),
                            LoggedCommands.deadline("Wait for auto up to " + algaeStop,
                                Elevator.instance.WaitForStop(algaeStop),
                                Elevator.instance.AutoElevatorUp(face.alignMiddle.getTranslation(), algaeStop)),
                            optInvertAlgae)))),
            new PIDSwerve(Swerve.instance, Pose.instance, extendedBackup ? face.algaeBackupExtended : face.algaeBackupShort, true, false))
            .handleInterrupt(() -> {
                if (!RobotState.haveAlgae()) RobotState.setNoAlgae();
                Vision.setCameraMode(CameraMode.DEFAULT);
            });
    }

    private Command DeLollipop() {
        return LoggedCommands.deadline("Acquire Algae from lollipop",
                Commands.sequence(
                    LoggedCommands.waitUntil("Wait for Algae", RobotState::haveAlgae),
                    Controls.instance.TriggerRumble()),
                RobotState.IntakeAlgae())
            .handleInterrupt(() -> { if (!RobotState.haveAlgae()) RobotState.setNoAlgae(); });
    }

    public Command SetStop(Stop stop) {
        return LoggedCommands.sequence("Set stop to " + stop,
            RobotState.SetCoralMode(),
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
        return LoggedCommands.sequence("Score Algae into Barge",
                Commands.defer(() -> new PIDSwerve(Swerve.instance, Pose.instance, Pose.instance.bargeShotPose(adjustment), false, false, PIDSpeed.TURBO).ignoreY(), Set.of(Swerve.instance)),
                LoggedCommands.deadline("Toss Algae",
                    Commands.sequence(
                        Elevator.instance.WaitForStop(Stop.L4_SCORE)),
                    Swerve.instance.HoldX(),
                    Elevator.instance.Move(Stop.L4_SCORE),
                    LoggedCommands.sequence("Wait to release Algae",
                        LoggedCommands.waitUntil("Wait for Algae Release Point", () -> Elevator.instance.aboveStop(Stop.ALGAE_RELEASE)),
                        RobotState.ScoreGamePiece())),
                Elevator.instance.FastZero()); // TODO Defer so that drive control returns?
    }

    private Command AlignToCage(Cage cage) {
        Pose2d cageAlignPose = new Pose2d(cage.location(), Rotation2d.k180deg).transformBy(PoseConstants.cageOffset);

        return LoggedCommands.sequence("Align to cage " + cage,
            LoggedCommands.runOnce("Disable reef aiming", optAutoReefAiming::disable),
            Vision.SwitchToRearVision(),
            LoggedCommands.runOnce("Enable end game mode", () -> RobotState.setClimbState(ClimbState.STARTED)),
            new PIDSwerve(Swerve.instance, Pose.instance, cageAlignPose.transformBy(PoseConstants.cageApproachOffset), true, false, PIDSpeed.FAST),
            new PIDSwerve(Swerve.instance, Pose.instance, cageAlignPose, true, true, PIDSpeed.SLOW));
    }

    private Command ProcessorAlign() {
        return LoggedCommands.sequence("Align to processor",
            new PIDSwerve(Swerve.instance, Pose.instance, PoseConstants.processorApproach, true, false, PIDSpeed.FAST),
            new PIDSwerve(Swerve.instance, Pose.instance, PoseConstants.processorScore, true, true, PIDSpeed.FAST));
    }

    public Command AlignToNearestCage() {
        return Commands.defer(() -> AlignToCage(Pose.instance.nearestCage()), Set.of(Swerve.instance));
    }

    public Command SmartScore(boolean left) {
        return Commands.either(
            LoggedCommands.proxy(Commands.select(left ? coralLeftCommands : coralRightCommands, () -> Pose.nearestFace(Pose.instance.getPose().getTranslation()))),
            Commands.either(
                Commands.either(
                    LoggedCommands.proxy(ProcessorAlign()),
                    LoggedCommands.proxy(BargeShot()),
                    RobotState::algaeToProcessor),
                Commands.either(
                    LoggedCommands.proxy(Commands.select(left ? deAlgaefyLeftCommands : deAlgaefyRightCommands, () -> Pose.nearestFace(Pose.instance.getPose().getTranslation()))),
                    DeLollipop(),
                    optAutoReefAiming::get),
                RobotState::haveAlgae),
            RobotState::haveCoral);
    }
}
