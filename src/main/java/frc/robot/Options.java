package frc.robot;

import frc.lib.util.TunableOption;

public final class Options {
    /* Global Toggleable Options and their defaults */
    public static final TunableOption optBackupPush = new TunableOption("Back Up in Auto", false);
    public static final TunableOption optMirrorAuto = new TunableOption("Mirror Auto to Left", false);
    public static final TunableOption optAutoReefAiming = new TunableOption("Automatically Aim at Reef", true);
    public static final TunableOption optBonusCoralStandoff = new TunableOption("Bonus Coral Standoff", false);
}