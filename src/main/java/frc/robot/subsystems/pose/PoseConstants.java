package frc.robot.subsystems.pose;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.auto.Constants;

public class PoseConstants {
    public static final int pigeonID = 1;

    public static final PIDController rotationPID = new PIDController(0.014, 0.0, 0.0); // kI was 0.050 for NCCMP 2024
    public static final double rotationKS = 0.02;
    public static final double rotationIZone = 2.0; // degrees

    public static final double tiltWarning = 10.0;
    public static final double tiltError = 30.0;

    // TODO What about AndyMark field?
    // NOTE That FlippingUtil might need to be impacted
    // TODO Consider using fieldLayout.getFieldLength(), etc.
    public static final double fieldWidth = FlippingUtil.fieldSizeY; // Units.inchesToMeters(26*12 + 5);
    public static final double fieldLength = FlippingUtil.fieldSizeX; // Units.inchesToMeters(57*12 + 6.875);

    public static final double reefElevatorZoneRadius = Units.inchesToMeters(80.0); // TODO Revisit
    public static final double autoUpDistance = Units.inchesToMeters(38.0); // Increase for quicker auto scoring, but risky
    public static final double wingLength = Units.inchesToMeters(280);

    public static final double robotFrameLength = Units.inchesToMeters(30);
    public static final double robotFrameWidth = Units.inchesToMeters(27);
    public static final double bumperWidth = Units.inchesToMeters(3.2);
    public static final double reefStandoff = Units.inchesToMeters(1.5);
    public static final double centerToFrontBumper = robotFrameLength / 2.0 + bumperWidth;
    public static final double reefOffset = centerToFrontBumper + reefStandoff;
    public static final double reefExtraOffset = Units.inchesToMeters(9.0);
    public static final double bonusStandoff = Units.inchesToMeters(4.0);

    public static final double processorAreaY = fieldWidth / 2.0 - 1.0;
    public static final Translation2d processor = Constants.atHQ ? new Translation2d(7.38, 0.46) : Constants.fieldLayout.getTagPose(16).get().toPose2d().getTranslation();
    public static final Pose2d processorScore = new Pose2d(processor.plus(new Translation2d(0.0, centerToFrontBumper)), Rotation2d.kCW_90deg);
    public static final double processorApproachOffset = Units.inchesToMeters(24.0);
    public static final Pose2d processorApproach = processorScore.transformBy(new Transform2d(-processorApproachOffset, 0.0, Rotation2d.kZero));

    // Locations from the Blue Alliance perspective
    public static final Translation2d reefCenter = new Translation2d(Units.inchesToMeters(176.75), fieldWidth / 2.0);
    public static final double reefToFaceDistance = reefCenter.getX() - Units.inchesToMeters(144.0);
    public static final double branchSeparation = Units.inchesToMeters(12.0 + 15.0 / 16.0);
    public static final double bargeShotDistanceFromCenter = Units.inchesToMeters(52.0);
    public static final double bargeShotX = fieldLength / 2.0 - bargeShotDistanceFromCenter - centerToFrontBumper;

    // Offset to the reef face, not at the branches, but on the faces directly in front
    public static final Translation2d centerOffset = new Translation2d(reefToFaceDistance + reefOffset - reefStandoff, 0.0); // NOTE: Undo reef standoff for algae
    private static final Translation2d leftOffset = new Translation2d(reefToFaceDistance + reefOffset, -branchSeparation / 2.0);
    private static final Translation2d rightOffset = new Translation2d(reefToFaceDistance + reefOffset, branchSeparation / 2.0);
    private static final Translation2d extraOffset = new Translation2d(reefExtraOffset, 0.0);
    private static final Translation2d centerApproachOffset = centerOffset.plus(extraOffset);
    private static final Translation2d leftApproachOffset = leftOffset.plus(extraOffset);
    private static final Translation2d rightApproachOffset = rightOffset.plus(extraOffset);
    public static final double approachDistanceToReefCenter = centerApproachOffset.getDistance(reefCenter);
    private static final Translation2d bonusOffset = new Translation2d(bonusStandoff, 0.0);
    private static final Translation2d leftBonusOffset = leftOffset.plus(bonusOffset);
    private static final Translation2d rightBonusOffset = rightOffset.plus(bonusOffset);
    private static final Transform2d leftL1Transform = new Transform2d(-Units.inchesToMeters(4.0), -branchSeparation / 2.0 + Units.inchesToMeters(2.5), Rotation2d.kZero);
    private static final Transform2d rightL1Transform = new Transform2d(-Units.inchesToMeters(4.0), branchSeparation / 2.0 - Units.inchesToMeters(2.5), Rotation2d.kZero);
    private static final Transform2d leftL1OutsideTransform = new Transform2d(-Units.inchesToMeters(4.0), Units.inchesToMeters(4.25), Rotation2d.kZero);
    private static final Transform2d rightL1OutsideTransform = new Transform2d(-Units.inchesToMeters(4.0), -Units.inchesToMeters(4.25), Rotation2d.kZero);
    public static final double L1MoveForward = Units.inchesToMeters(6);
    private static final Transform2d extraAlgaeBackupShort = new Transform2d(Units.inchesToMeters(-9.0), 0.0, Rotation2d.kZero);
    private static final Transform2d extraAlgaeBackupExtended = new Transform2d(Units.inchesToMeters(-18.0), 0.0, Rotation2d.kZero);

    public static final double elevatorNoDownDistance = reefToFaceDistance + reefOffset + Units.inchesToMeters(12.0);

    public static enum ReefFace {
        AB(-180, true),
        CD(-120, false),
        EF(-60, true),
        GH(0, false),
        IJ(60, true),
        KL(120, false);

        ReefFace(double directionDegrees, boolean algaeHigh) {
            directionFromCenter = Rotation2d.fromDegrees(directionDegrees);
            alignMiddle = new Pose2d(reefCenter.plus(centerOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
            alignLeft = new Pose2d(reefCenter.plus(leftOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
            leftL1 = alignLeft.transformBy(leftL1Transform);
            leftL1Outside = alignLeft.transformBy(leftL1OutsideTransform);
            alignRight = new Pose2d(reefCenter.plus(rightOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
            rightL1 = alignRight.transformBy(rightL1Transform);
            rightL1Outside = alignRight.transformBy(rightL1OutsideTransform);
            approachMiddle = new Pose2d(reefCenter.plus(centerApproachOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
            algaeBackupShort = approachMiddle.plus(extraAlgaeBackupShort);
            algaeBackupExtended = approachMiddle.plus(extraAlgaeBackupExtended);
            approachLeft = new Pose2d(reefCenter.plus(leftApproachOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
            approachRight = new Pose2d(reefCenter.plus(rightApproachOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
            alignBonusLeft = new Pose2d(reefCenter.plus(leftBonusOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
            alignBonusRight = new Pose2d(reefCenter.plus(rightBonusOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
            this.algaeHigh = algaeHigh;
        }

        public final Rotation2d directionFromCenter;
        public final Pose2d alignLeft, alignMiddle, alignRight;
        public final Pose2d leftL1, rightL1;
        public final Pose2d leftL1Outside, rightL1Outside;
        public final Pose2d approachLeft, approachMiddle, approachRight;
        public final Pose2d algaeBackupShort, algaeBackupExtended;
        public final Pose2d alignBonusLeft, alignBonusRight;
        public final boolean algaeHigh;
    }

    // Cage locations from 6328
    public static enum Cage {
        CLOSE(199.947), // 5.079 m
        MIDDLE(242.855), // 6.169 m
        FAR(286.779), // 7.284 m
        HQ(8.16, 2.37);

        private final Translation2d location;
        private static final Translation2d redFudge = new Translation2d(0.00, 0.0);
        private static final Translation2d blueFudge = new Translation2d(0.0, 0.0);

        Cage(double yInches) {
            // 8.774 m
            location = new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(yInches));
        }

        Cage(double xMeters, double yMeters) {
            location = new Translation2d(xMeters, yMeters);
        }

        public Translation2d location() {
            // NOTE: This is prior to flipping to red
            if (Robot.isRed()) {
                return location.plus(redFudge);
            } else {
                return location.plus(blueFudge);
            }
        }
    }

    // 30 in / 2 + 3.2 in - 2.0 in = 16.2 in = 0.4115 m

    // fieldSizeX = Units.feetToMeters(57.573); 17.548 m
    // fieldSizeY = Units.feetToMeters(26.417); 8.052 m

    // Hypothetical
    // Blue CLOSE = 8.363, 5.08
    // Blue MIDDLE = 8.363, 6.17
    // Blue FAR = 8.363, 7.28
    // Red CLOSE = 9.19, 2.97
    // Red MIDDLE = 9.19, 1.883
    // Red FAR = 9.19, 0.768

    // Ashville
    // Want X diff of 0.30 m?
    // Blue CLOSE = 8.36 X 8.49, 5.23 X 5.09
    // Blue MIDDLE = 8.47, 6.16
    // Blue FAR =
    // Red CLOSE = 9.11, 2.90
    // Red MIDDLE = 9.07, 1.84
    // Red FAR = X

    // Mecklenburg
    // Blue CLOSE = 8.35, 5.10   (8.774 - 0.42)
    // Red CLOSE = 9.11, 2.95    (8.774 + 0.34)

    public static final Transform2d cageOffset = new Transform2d(Units.inchesToMeters(8.0), 0, Rotation2d.kZero);
    public static final Transform2d cageApproachOffset = new Transform2d(Units.inchesToMeters(16.0), 0, Rotation2d.kZero);
}