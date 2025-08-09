package frc.robot;

import com.pathplanner.lib.util.FlippingUtil;

public class Field {
    // TODO What about AndyMark field?
    // NOTE That FlippingUtil might need to be impacted
    // TODO Consider using fieldLayout.getFieldLength(), etc.
    public static final double width = FlippingUtil.fieldSizeY; // Units.inchesToMeters(26*12 + 5);
    public static final double length = FlippingUtil.fieldSizeX; // Units.inchesToMeters(57*12 + 6.875);

    // TODO Should probably split some things out into FieldConstants
    public enum ReefLevel {
        L1,
        L2,
        L3,
        L4
    }

        
}