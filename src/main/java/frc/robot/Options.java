package frc.robot;

import frc.lib.util.TunableOption;

public final class Options {
    /* Global Toggleable Options and their defaults */
    public static final TunableOption optUseTrigVision = new TunableOption("Use Trig Vision", false);
    public static final TunableOption optBackupPush = new TunableOption("Back Up in Auto", false);
    public static final TunableOption optMirrorAuto = new TunableOption("Mirror Auto to Left", false);
    public static final TunableOption optAutoCoralWait = new TunableOption("Wait for Coral in Auto", true);
    public static final TunableOption optAutoReefAiming = new TunableOption("Automatically Aim at Reef", true);
    public static final TunableOption optBonusCoralStandoff = new TunableOption("Bonus Coral Standoff", false);
    public static final TunableOption optAlgaeBargeOnly = new TunableOption("Algae into Barge Only", false);
    public static final TunableOption optInvertAlgae = new TunableOption("Invert Algae Location", false);
    public static final TunableOption optIndexEnabled = new TunableOption("Index Enabled", true);
}