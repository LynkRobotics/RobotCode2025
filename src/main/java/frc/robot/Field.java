package frc.robot;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public class Field {
    // TODO What about AndyMark field?
    // NOTE That FlippingUtil might need to be impacted
    // TODO Consider using fieldLayout.getFieldLength(), etc.
    public static final Distance width = Units.Meters.of(FlippingUtil.fieldSizeY); // Units.inchesToMeters(26*12 + 5);
    public static final Distance length = Units.Meters.of(FlippingUtil.fieldSizeX); // Units.inchesToMeters(57*12 + 6.875);

    // TODO Should probably split some things out into FieldConstants
    public enum ReefLevel {
        L1,
        L2,
        L3,
        L4
    }
}