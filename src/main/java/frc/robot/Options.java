package frc.robot;

import frc.lib.util.TunableOption;

public final class Options {
    /* Global Toggleable Options and their defaults */
    public static final TunableOption optShooterIntake = new TunableOption("Shooter intake", false);
    public static final TunableOption optDirectRPM = new TunableOption("Direct set RPM", false);
    public static final TunableOption optShootWithVision = new TunableOption("Shoot with Vision", true);
    public static final TunableOption optAimingEnabled = new TunableOption("Aiming enabled", true);
    public static final TunableOption optBrakeAfterTeleop = new TunableOption("Brake after teleOp", true);
    public static final TunableOption optMaintainAngle = new TunableOption("Maintain angle", true);
    public static final TunableOption optAimAtAmp = new TunableOption("Aim at Amp", true);
    // public static final TunableOption optVisionPoseInAuto = new TunableOption("Use Vision Pose in Auto", false);
    // public static final TunableOption optVisionPoseInTeleop = new TunableOption("Disable Vision Pose in Teleop", true);
}