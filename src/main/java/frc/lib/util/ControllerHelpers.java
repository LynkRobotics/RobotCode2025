package frc.lib.util;

import edu.wpi.first.math.geometry.Translation2d;

public final class ControllerHelpers {

  public static Translation2d fromCircularDiscCoordinates(double x, double y) {
    // https://stackoverflow.com/a/32391780
    var mappedX =
        0.5 * signedSqrt(2 + Math.pow(x, 2) - Math.pow(y, 2) + 2 * x * Math.sqrt(2))
            - 0.5
                * signedSqrt(
                    2 + Math.pow(x, 2) - Math.pow(y, 2) - 2 * x * Math.sqrt(2));
    var mappedY =
        0.5 * signedSqrt(2 - Math.pow(x, 2) + Math.pow(y, 2) + 2 * y * Math.sqrt(2))
            - 0.5
                * signedSqrt(
                    2 - Math.pow(x, 2) + Math.pow(y, 2) - 2 * y * Math.sqrt(2));

    return desaturate(mappedX, mappedY);
  }

  private static Translation2d desaturate(double x, double y) {
    double absX = Math.abs(x);
    double absY = Math.abs(y);

    if (absX > 1 && absX > absY) {
      // X is too big and is the more problematic one

      var ratio = 1 / absX;

      return new Translation2d(x * ratio, y * ratio);
    } else if (absY > 1 && absY > absX) {
      // Y is too big and is the more problematic one
      var ratio = 1 / absY;

      return new Translation2d(x * ratio, y * ratio);

    } else {
      //  Everything fine
      return new Translation2d(x, y);
    }
  }

  /**
   * Computes the square root of the absolute value of the input while preserving the original sign.
   * 
   * <p>This method ensures that the result has the same sign as the input value.
   * 
   * @param value The input value for which the signed square root is to be calculated.
   * @return The signed square root of the absolute value of the input.
   */
  private static double signedSqrt(double value) {
    return Math.copySign(Math.sqrt(Math.abs(value)), value);
  }
}