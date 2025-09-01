package frc.robot.subsystems.pose;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.Field;

public class PoseConstants {
    public static final int pigeonID = 0;

    public static final PIDController rotationPID = new PIDController(0.011, 0.0, 0.0);
    public static final double rotationKS = 0.015;
    public static final double rotationMax = 0.40;
    public static final double rotationIZone = 2.0; // degrees

    public static final double tiltWarning = 10.0;
    public static final double tiltError = 30.0;

    public static final Distance reefElevatorZoneRadius = Units.Inches.of(80.0); // TODO Revisit
    public static final Distance autoUpDistance = Units.Inches.of(38.0); // Increase for quicker auto scoring, but risky
    public static final Distance wingLength = Units.Inches.of(280);

    // Robot dimensions
    public static final Distance robotFrameLength = Units.Inches.of(26.0);
    public static final Distance robotFrameWidth = Units.Inches.of(26.0);
    public static final Distance bumperWidth = Units.Inches.of(2.75);

    private static final Distance centerToFrontBumper = robotFrameLength.div(2.0).plus(bumperWidth);
    private static final Distance reefCoralStandoff = Units.Inches.of(5.2); // How far between bumper and reef when scoring coral
    private static final Distance reefCoralOffset = centerToFrontBumper.plus(reefCoralStandoff);
    private static final Distance reefL1ExtraOffset = Units.Inches.of(2.0);
    private static final Distance reefAlgaeStandoff = Units.Inches.of(-3.0); // How far between bumper and reef when intaking algae
    private static final Distance reefAlgaeOffset = centerToFrontBumper.plus(reefAlgaeStandoff);
    private static final Distance reefApproachOffset = Units.Inches.of(6.0); // How far away from the desired position to approach first

    public static final Distance processorAreaY = Field.width.div(2.0).minus(Units.Meters.of(1.0)); // What portion of the field do we defined as "near the processor (instead of barge)"
    public static final Translation2d processor = Constants.atHQ ? new Translation2d(7.38, 0.46) : Constants.fieldLayout.getTagPose(16).get().toPose2d().getTranslation();
    public static final Pose2d processorScore = new Pose2d(processor.plus(new Translation2d(Units.Meters.zero(), centerToFrontBumper)), Rotation2d.kCW_90deg);
    public static final Distance processorApproachOffset = Units.Inches.of(24.0);
    public static final Pose2d processorApproach = processorScore.transformBy(new Transform2d(processorApproachOffset.unaryMinus(), Units.Meters.zero(), Rotation2d.kZero));

    // Locations from the Blue Alliance perspective
    public static final Translation2d reefCenter = new Translation2d(Units.Inches.of(176.75), Field.width.div(2.0)); // Position of the center of the reef
    private static final Distance reefToFaceDistance = reefCenter.getMeasureX().minus(Units.Inches.of(144.0)); // Distance from center of reef to center of reef face
    private static final Distance branchSeparation = Units.Inches.of(12.0 + 15.0 / 16.0); // Center-to-center separation between reef branches on the same face
    private static final Distance bargeShotDistanceFromCenter = Units.Inches.of(52.0); // How far from the center of field to our bumper for the barge shot
    public static final Distance bargeShotX = Field.length.div(2.0).minus(bargeShotDistanceFromCenter).minus(centerToFrontBumper); // X position of the barge shot

    // Offset to the reef face, not at the branches, but on the faces directly in front
    private static final Translation2d algaeOffset = new Translation2d(reefToFaceDistance.plus(reefAlgaeOffset), Units.Inches.of(3.0));
    private static final Translation2d centerCoralOffset = new Translation2d(reefToFaceDistance.plus(reefCoralOffset), Units.Meters.zero());
    private static final Translation2d branchOffset = new Translation2d(Units.Meter.zero(), branchSeparation.div(2.0));
    private static final Translation2d leftCoralOffset = centerCoralOffset.minus(branchOffset);
    private static final Translation2d rightCoralOffset = centerCoralOffset.plus(branchOffset);
    private static final Translation2d approachOffset = new Translation2d(reefApproachOffset, Units.Meters.zero());
    private static final Translation2d centerApproachOffset = algaeOffset.plus(approachOffset);
    public static final double approachDistanceToReefCenter = centerApproachOffset.getDistance(reefCenter);
    private static final Transform2d leftL1Transform = new Transform2d(reefL1ExtraOffset, branchSeparation.div(2.0), Rotation2d.kZero);
    private static final Transform2d rightL1Transform = new Transform2d(reefL1ExtraOffset, branchSeparation.div(-2.0), Rotation2d.kZero);
    private static final Transform2d extraAlgaeBackupShort = new Transform2d(Units.Inches.of(-9.0), Units.Inches.zero(), Rotation2d.kZero);
    private static final Transform2d extraAlgaeBackupExtended = new Transform2d(Units.Inches.of(-18.0), Units.Inches.zero(), Rotation2d.kZero);

    public static enum ReefFace {
        AB(-180, true),
        CD(-120, false),
        EF(-60, true),
        GH(0, false),
        IJ(60, true),
        KL(120, false);

        ReefFace(double directionDegrees, boolean algaeHigh) {
            directionFromCenter = Rotation2d.fromDegrees(directionDegrees);
            alignAlgae = new Pose2d(reefCenter.plus(algaeOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.kZero));
            alignCoralLeft = new Pose2d(reefCenter.plus(leftCoralOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.kZero));
            alignCoralL1Left = alignCoralLeft.transformBy(leftL1Transform);
            // leftL1Outside = alignLeft.transformBy(leftL1OutsideTransform);
            alignCoralRight = new Pose2d(reefCenter.plus(rightCoralOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.kZero));
            alignCoralL1Right = alignCoralRight.transformBy(rightL1Transform);
            // rightL1Outside = alignRight.transformBy(rightL1OutsideTransform);
            approachAlgaeMiddle = new Pose2d(reefCenter.plus(centerApproachOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.kZero));
            algaeBackupShort = approachAlgaeMiddle.plus(extraAlgaeBackupShort);
            algaeBackupExtended = approachAlgaeMiddle.plus(extraAlgaeBackupExtended);
            this.algaeHigh = algaeHigh;
        }

        public final Rotation2d directionFromCenter;
        public final Pose2d alignCoralLeft, alignAlgae, alignCoralRight;
        public final Pose2d alignCoralL1Left, alignCoralL1Right;
        // public final Pose2d leftL1Outside, rightL1Outside;
        public final Pose2d approachAlgaeMiddle;
        public final Pose2d algaeBackupShort, algaeBackupExtended;
        public final boolean algaeHigh;
    }
}