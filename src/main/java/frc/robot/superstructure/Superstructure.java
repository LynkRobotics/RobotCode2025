package frc.robot.superstructure;

import static frc.robot.Options.optAlgaeBargeOnly;
import static frc.robot.Options.optFullBargeAuto;
import static frc.robot.Options.optInvertAlgae;
import static frc.robot.Options.optMirrorAuto;

import java.util.EnumMap;
import java.util.Set;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.pose.Pose;
import frc.robot.subsystems.pose.PoseConstants;
import frc.robot.subsystems.pose.PoseConstants.ReefFace;
import frc.robot.Robot;
import frc.robot.Field.ReefLevel;

public class Superstructure extends SubsystemBase {
    public static final Superstructure instance = new Superstructure();
    private static final TunableOption optOverrideClimberTiming = new TunableOption("Override Climber Timing", false);
    private static final int climberTimeCutoff = 30; // seconds
    private ReefLevel activeReefLevel = ReefLevel.L4;
    private Canandcolor canandcolor = new Canandcolor(0);
    private static final double hueMaxDelta = 0.075; 
    private static final double satMaxDelta = 0.300;
    private static final double valMaxDelta = 0.500;
    private static final FieldColor[] fieldColors = FieldColor.values();
    private final double colorSeekSpeed = 0.2 * SwerveConstants.maxSpeed; // Any faster and we'll overshoot by too much
    private final Rotation2d bargeRotation = Rotation2d.fromDegrees(-25); // 1678 uses 20 degrees, but our sensor is too far back

    public static enum FieldColor {
        RED(0.062, 0.912, 0.695),
        BLUE(0.558, 0.729, 0.370),
        CARPET(0.255, 0.590, 0.180);

        public final double hue, sat, val;

        FieldColor(double h, double s, double v) {
            hue = h;
            sat = s;
            val = v;
        }
    }

    public static enum EEPose {
        L1(EEPosition.L1, Stop.L1),
        L2(EEPosition.L23, Stop.L2),
        L3(EEPosition.L23, Stop.L3),
        L4(EEPosition.L4, Stop.L4),
        L4_PREP(EEPosition.CORAL_HOLD, Stop.L4_PREP),
        L4_HOLDHIGH(EEPosition.CORAL_HOLD, Stop.L4),
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

    public static enum SuperState {
        INTAKING_CORAL,
        INTAKING_ALGAE,
        SCORING_CORAL,
        SCORING_ALGAE,
        DEFAULT,
        NONE
    }

    private SuperState superState = SuperState.DEFAULT;
    private Command defaultPositionCommand = AssumeDefaultPosition();

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

        SmartDashboard.putData("Superstructure/Sensor & Position Reset",
            LoggedCommands.sequence("Sensor & Position Reset",
                EndEffector.instance.SensorReset(),
                AssumeDefaultPosition(),
                WaitForEEPose()));
    }

    private FieldColor getFieldColor() {
        return getFieldColor(canandcolor.getHSVHue(), canandcolor.getHSVSaturation(), canandcolor.getHSVValue());
    }

    private FieldColor getFieldColor(double hue, double sat, double val) {
        for (FieldColor color: fieldColors) {
            if (Math.abs(color.hue - hue) < hueMaxDelta && Math.abs(color.sat - sat) < satMaxDelta && Math.abs(color.val - val) < valMaxDelta) {
                return color;
            }
        }

        return null;
    }

    private Command WaitForColor(FieldColor color) {
        return LoggedCommands.waitUntil("Waiting for color " + color.name(), () -> getFieldColor() == color);
    }

    public Command AwayUntilColor(FieldColor color) {
        return LoggedCommands.sequence("Drive until color " + color.name(),
            LoggedCommands.runOnce("Drive forward", () -> Swerve.instance.drive(new Translation2d(Robot.isRed() ? -colorSeekSpeed : colorSeekSpeed, 0), 0.0, true)),
            WaitForColor(color),
            Swerve.instance.Stop());
    }

    public Command ForwardUntilColor(FieldColor color) {
        return LoggedCommands.sequence("Drive until color " + color.name(),
            LoggedCommands.runOnce("Drive forward",
                () -> Swerve.instance.driveRobotRelativeAuto(new ChassisSpeeds(colorSeekSpeed, 0.0, 0.0))),
            WaitForColor(color),
            Swerve.instance.Stop());
    }

    // Turn to the angle we want to use for the barge
    private Command BargeTurn() {
        return LoggedCommands.defer("Turn for barge", () -> {
            Pose2d pose = Pose.instance.getPose();
            Rotation2d rotation = Robot.isRed() ? bargeRotation.plus(Rotation2d.k180deg) : bargeRotation;
            return new PIDSwerve(Swerve.instance, Pose.instance, new Pose2d(pose.getX(), pose.getY(), rotation), false, false);
        }, Set.of(Swerve.instance));
    }

    public Command DriveUntilBarge() {
        return LoggedCommands.sequence("Drive until barge",
            BargeTurn(),
            AwayUntilColor(Robot.isRed() ? FieldColor.RED : FieldColor.BLUE),
            Swerve.instance.Stop());
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

    private Command TriggerMoveToL4() {
        return LoggedCommands.sequence("Move to L4 sequence",
            Commands.either(
                LoggedCommands.log("No need to prep L4"),
                Commands.sequence(
                    TriggerMoveToEEPose(EEPose.L4_PREP),
                    AlgaeRoller.instance.TriggerStowWhenClear(),
                    WaitForEEPose().until(() -> EndEffector.instance.inPosition()), // Elevator target can change as soon as EE is in position
                    Commands.either(
                        LoggedCommands.log("Already at L4_PREP when End Effector in position"),
                        Commands.sequence(
                            TriggerMoveToEEPose(EEPose.L4_HOLDHIGH),
                            WaitForEEPose().until(() -> Elevator.instance.nearOrAbove(Stop.L4_PREP))),
                        () -> Elevator.instance.nearOrAbove(Stop.L4_PREP))),
                () -> Elevator.instance.nearOrAbove(Stop.L4_PREP)),
            TriggerMoveToEEPose(EEPose.L4),
            AlgaeRoller.instance.TriggerStowWhenClear());
    }

    // NOTE: Can block in the case of L4, which uses an interim position
    private Command TriggerMoveToActiveCoral() {
        return Commands.either(
            Commands.either(
                TriggerMoveToEEPoseDirect(EEPose.L1),
                Commands.either(
                    TriggerMoveToL4(),
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
                    TriggerMoveToL4(),
                    Commands.either(
                        TriggerMoveToEEPose(EEPose.L2),
                        TriggerMoveToEEPose(EEPose.L3),
                        () -> activeReefLevel == ReefLevel.L2),
                    () -> activeReefLevel == ReefLevel.L4),
                () -> activeReefLevel == ReefLevel.L1),
            () -> EndEffector.instance.inHighClearRange()); // All scoring positions are in high clear range, so it's safe to move if EE is already in the high clear range
    }

    public Command WaitForEEPose() {
        return LoggedCommands.waitUntil("Wait for EE Pose", () -> EndEffector.instance.inPosition() && Elevator.instance.atFinalTarget());
    }

    private void setFaceCommands(ReefFace face) {
        coralLeftCommands.put(face, ScoreCoral(face, true));
        coralRightCommands.put(face, ScoreCoral(face, false));
        deAlgaefyLeftCommands.put(face, DeAlgaefy(face));
        deAlgaefyRightCommands.put(face, DeAlgaefy(face));
    }

    private Command SetSuperState(SuperState state) {
        return LoggedCommands.runOnce("Set Superstate to " + state, () -> superState = state);
    }

    private void setSuperStateDone(boolean interrupted) {
        if (interrupted) {
            DogLog.log(LoggedCommands.key, "Interrupted, setting superstate to NONE");
        } else {
            DogLog.log(LoggedCommands.key, "Completed, setting superstate to NONE");
        }
        superState = SuperState.NONE;
    }
    
    public Command ScoreCoral(ReefFace face, boolean left) {
        return Commands.either(
            LoggedCommands.sequence("Auto Align " + (left ? "Left " : "Right ") + face.toString() + " & Score",
                SetSuperState(SuperState.SCORING_CORAL),
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
                PlaceCoral())
            .finallyDo(this::setSuperStateDone),
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
            SetSuperState(SuperState.INTAKING_ALGAE),
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
            new PIDSwerve(Swerve.instance, Pose.instance, extendedBackup ? face.algaeBackupExtended : face.algaeBackupShort, true, false))
            .finallyDo(this::setSuperStateDone);
    }

    public boolean shouldMirror() {
        return optMirrorAuto.get() && DriverStation.isAutonomousEnabled();
    }

    public Command PrepBargeShot() {
        return LoggedCommands.sequence("Prepare barge shot",
            SetSuperState(SuperState.SCORING_ALGAE),
            TriggerMoveToEEPose(EEPose.BARGE_PREP),
            AlgaeRoller.instance.TriggerStowWhenClear(),
            WaitForEEPose(),
            TriggerMoveToEEPoseDirect(EEPose.BARGE),
            WaitForEEPose(),
            LoggedCommands.idle("Idle to maintain barge pose"))
            .finallyDo(this::setSuperStateDone);
    }

    public Command FullBargeShot() {
        return LoggedCommands.sequence("Full barge shot",
            SetSuperState(SuperState.SCORING_ALGAE),
            Commands.parallel(
                DriveUntilBarge(),
                Commands.sequence(
                    TriggerMoveToEEPose(EEPose.BARGE_PREP),
                    AlgaeRoller.instance.TriggerStowWhenClear(),
                    WaitForEEPose())),
            TriggerMoveToEEPoseDirect(EEPose.BARGE),
            WaitForEEPose(),
            PlaceBargeAlgae())
            .finallyDo(this::setSuperStateDone);
    }

    private Command ProcessorAlign() {
        return LoggedCommands.sequence("Align to processor",
            SetSuperState(SuperState.SCORING_ALGAE),
            new PIDSwerve(Swerve.instance, Pose.instance, PoseConstants.processorApproach, true, false, PIDSpeed.FAST),
            new PIDSwerve(Swerve.instance, Pose.instance, PoseConstants.processorScore, true, true, PIDSpeed.FAST))
            .finallyDo(this::setSuperStateDone);
    }

    public Command SmartScore(boolean left) {
        return LoggedCommands.either("Smart Score",
            LoggedCommands.proxy(LoggedCommands.select("Coral select", left ? coralLeftCommands : coralRightCommands, () -> Pose.nearestFace(Pose.instance.getPose().getTranslation()))),
            Commands.either(
                Commands.either(
                    Commands.either(
                        LoggedCommands.proxy(FullBargeShot()),
                        LoggedCommands.proxy(PrepBargeShot()),
                        optFullBargeAuto::get),
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
        return LoggedCommands.sequence("Place Coral",
            SetSuperState(SuperState.SCORING_CORAL),
            Commands.either(
                PlaceL1Coral(),
                Commands.either(
                    EndEffector.instance.ExpelCoral(ReefLevel.L4),
                    Commands.either(
                        EndEffector.instance.ExpelCoral(ReefLevel.L2),
                        EndEffector.instance.ExpelCoral(ReefLevel.L3),
                        () -> activeReefLevel == ReefLevel.L2),
                    () -> activeReefLevel == ReefLevel.L4),
                () -> activeReefLevel == ReefLevel.L1))
            .finallyDo(this::setSuperStateDone);
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
                AlgaeRoller.instance.StopAndClear().schedule(); // HACK
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
            SetSuperState(SuperState.SCORING_ALGAE),
            EndEffector.instance.PlaceBargeAlgae())
            .finallyDo(this::setSuperStateDone);
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
            AlgaeRoller.instance.TriggerStowWhenStoppedAndPivotClear());
    }

    public Command AlgaeHold() {
        return LoggedCommands.sequence("Hold algae",
            TriggerMoveToEEPose(EEPose.ALGAE_HOLD),
            AlgaeRoller.instance.TriggerStowWhenStopped());
    }

    public Command SmartCoralIntake() {
        return Commands.either(
            CoralHold(),
            IntakeCoral(),
            () -> EndEffector.instance.haveCoral());
    }

    public Command SmartAlgaeIntake() {
        return Commands.either(
            IntakeGroundAlgae(),
            AssumeDefaultPosition(),
            () -> EndEffector.instance.haveNothing());
    }

    public Command AssumeDefaultPosition() {
        return LoggedCommands.sequence("Assume default position",
            AlgaeRoller.instance.StopIntake(),
            Commands.either(
                AlgaeHold(),
                Commands.either(
                    CoralHold(),
                    Commands.sequence(
                        TriggerMoveToEEPose(EEPose.GROUND_CORAL),
                        AlgaeRoller.instance.TriggerStowWhenStopped()),
                    () -> EndEffector.instance.haveCoral()),
                () -> EndEffector.instance.haveAlgae()),
            SetSuperState(SuperState.DEFAULT));
    }

    private Command IntakeExpel = Intake.instance.Expel();
    private Command IntakeHold = Intake.instance.HoldCoral();
    private Command EEStop = EndEffector.instance.StopIntake();

    public Command StartCoralIntake() {
        return LoggedCommands.sequence("Start Coral Intake",
            SetSuperState(SuperState.INTAKING_CORAL),
            TriggerMoveToEEPose(EEPose.GROUND_CORAL),
            AlgaeRoller.instance.TriggerStowWhenStopped(),
            Commands.parallel(
                EndEffector.instance.StartCoralIntake(),
                Intake.instance.Deploy()));
    }

    public Command FinishCoralIntake() {
        return LoggedCommands.sequence("Finish Coral Intake",
            Commands.either(
                Commands.none(),
                EndEffector.instance.StopIntake(),
                EndEffector.instance::haveCoral),
            Intake.instance.Stop(),
            SetSuperState(SuperState.NONE));
    }

    // TODO Use StartCoralIntake() && Finish Coral Intake
    private Command IntakeCoral() {
        // NOTE: Must not be holding any game piece already!
        return LoggedCommands.sequence("Intaking Coral",
            SetSuperState(SuperState.INTAKING_CORAL),
            Commands.either(
                LoggedCommands.none("Not moving algae because already holding algae"),
                Commands.sequence(
                    TriggerMoveToEEPose(EEPose.GROUND_CORAL),
                    AlgaeRoller.instance.TriggerStowWhenStopped()),
                EndEffector.instance::haveAlgae),
            Commands.parallel(
                EndEffector.instance.StartCoralIntake(),
                Intake.instance.Deploy()),
            Commands.either(
                LoggedCommands.waitUntil("Waiting for coral in indexer", Intake.instance::coralInIndexer),
                EndEffector.instance.WaitForState(EEState.HAVE_CORAL),
                EndEffector.instance::haveAlgae),
            Controls.instance.TriggerRumble())
            .finallyDo((interrupted) -> {
                if (Intake.instance.coralInIndexer()) {
                    IntakeHold.schedule();
                } else {
                    IntakeExpel.schedule();
                }
                if (interrupted && EndEffector.instance.haveNothing()) {
                    // We don't have anything, so stop the intake
                    EEStop.schedule();
                }
                setSuperStateDone(interrupted);
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
            SetSuperState(SuperState.INTAKING_ALGAE),
            Commands.deadline(
                EndEffector.instance.WaitForAlgae(),
                Commands.sequence(
                    AlgaeRoller.instance.TriggerDeploy(),
                    TriggerMoveToEEPose(EEPose.GROUND_ALGAE),
                    WaitForEEPose(),
                    EndEffector.instance.StartAlgaeIntake(),
                    AlgaeRoller.instance.TriggerDeploy(),
                    AlgaeRoller.instance.StartIntake())),
            AlgaeRoller.instance.StopIntake())
            .finallyDo(this::setSuperStateDone);
    }

    public ReefLevel activeReefLevel() {
        return activeReefLevel;
    }

    public Command SystemRecovery() {
        return LoggedCommands.sequence("System Recovery",
            AlgaeRoller.instance.StopIntake(),
            EndEffector.instance.StopIntake(),
            Climber.instance.Stop(),
            Intake.instance.ZeroIntake(),
            AlgaeRoller.instance.SafeClear(),
            // TODO Elevator.instance.SafeClear(),
            EndEffector.instance.SensorReset(),
            EndEffector.instance.ResetPosition(),
            EndEffector.instance.TriggerMoveTo(EEPosition.CORAL_HOLD),
            LoggedCommands.waitUntil("End Effector in coral hold position", () -> EndEffector.instance.inPosition()),
            EndEffector.instance.ExpelCoral(ReefLevel.L4),
            EndEffector.instance.TriggerMoveTo(EEPosition.GROUND_INTAKE),
            LoggedCommands.waitUntil("End Effector in ground intake position", () -> EndEffector.instance.inPosition()),
            Elevator.instance.Zero(),
            AlgaeRoller.instance.Zero(),
            Intake.instance.Expel(),
            Commands.waitSeconds(2.0),
            Intake.instance.Stop());
    }

    @Override
    public void periodic() {
        DogLog.log("Superstructure/Active Reef Level", activeReefLevel);
        DogLog.log("Superstructure/Super State", superState);
        DogLog.log("Superstructure/Color", getFieldColor());
        DogLog.log("Superstructure/Hue", canandcolor.getHSVHue());
        DogLog.log("Superstructure/Saturation", canandcolor.getHSVSaturation());
        DogLog.log("Superstructure/Value", canandcolor.getHSVValue());

        if (superState == SuperState.NONE && !defaultPositionCommand.isScheduled() && DriverStation.isTeleopEnabled()) {
            defaultPositionCommand.schedule();
        }
    }
}