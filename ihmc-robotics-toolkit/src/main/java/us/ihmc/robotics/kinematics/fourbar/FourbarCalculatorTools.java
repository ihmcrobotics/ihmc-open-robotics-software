package us.ihmc.robotics.kinematics.fourbar;

import static java.lang.Math.abs;
import static java.lang.Math.acos;
import static java.lang.Math.atan2;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;
import static us.ihmc.robotics.MathTools.checkIntervalContains;
import static us.ihmc.robotics.MathTools.cube;
import static us.ihmc.robotics.MathTools.square;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.robotics.MathTools;

public class FourbarCalculatorTools
{
   public static double getCosineAngleWithCosineLaw(double l_neighbour1, double l_neighbour2, double l_opposite)
   {
      checkIntervalContains(l_neighbour1, 0.0, Double.POSITIVE_INFINITY);
      checkIntervalContains(l_neighbour2, 0.0, Double.POSITIVE_INFINITY);
      checkIntervalContains(l_opposite, 0.0, Double.POSITIVE_INFINITY);

      double cosAngle = MathTools
            .clamp((square(l_neighbour1) + square(l_neighbour2) - square(l_opposite)) / (2.0 * l_neighbour1 * l_neighbour2), -1.0, 1.0);

      return cosAngle;
   }

   public static double getCosineAngleDotWithCosineLaw(double l_neighbour1, double l_neighbour2, double lDot_neighbour2, double l_opposite,
         double lDot_opposite)
   {
      checkIntervalContains(l_neighbour1, 0.0, Double.POSITIVE_INFINITY);
      checkIntervalContains(l_neighbour2, 0.0, Double.POSITIVE_INFINITY);
      checkIntervalContains(l_opposite, 0.0, Double.POSITIVE_INFINITY);

      double cosAngleDot = (square(l_neighbour2) * lDot_neighbour2 - 2.0 * l_neighbour2 * l_opposite * lDot_opposite - lDot_neighbour2 * square(l_neighbour1)
            + lDot_neighbour2 * square(l_opposite)) / (2.0 * square(l_neighbour2) * l_neighbour1);

      return cosAngleDot;
   }

   public static double getCosineAngleDDotWithCosineLaw(double l_neighbour1, double l_neighbour2, double lDot_neighbour2, double lDDot_neighbour2,
         double l_opposite, double lDot_opposite, double lDDot_opposite)
   {
      checkIntervalContains(l_neighbour1, 0.0, Double.POSITIVE_INFINITY);
      checkIntervalContains(l_neighbour2, 0.0, Double.POSITIVE_INFINITY);
      checkIntervalContains(l_opposite, 0.0, Double.POSITIVE_INFINITY);

      double cosAngleDDot =
            (cube(l_neighbour2) * lDDot_neighbour2 - 2 * square(l_neighbour2 * lDot_opposite) - 2 * square(l_neighbour2) * l_opposite * lDDot_opposite
                  + 4 * l_neighbour2 * lDot_neighbour2 * l_opposite * lDot_opposite + 2 * square(lDot_neighbour2 * l_neighbour1) - 2 * square(
                  lDot_neighbour2 * l_opposite) - l_neighbour2 * lDDot_neighbour2 * square(l_neighbour1) + l_neighbour2 * lDDot_neighbour2 * square(l_opposite))
                  / (2.0 * cube(l_neighbour2) * l_neighbour1);

      return cosAngleDDot;
   }

   public static double getAngleWithCosineLaw(double l_neighbour1, double l_neighbour2, double l_opposite)
   {
      double angle = acos(getCosineAngleWithCosineLaw(l_neighbour1, l_neighbour2, l_opposite));

      return angle;
   }

   public static double getAngleDotWithCosineLaw(double l_neighbour1, double l_neighbour2, double lDot_neighbour2, double l_opposite, double lDot_opposite)
   {
      checkIntervalContains(l_neighbour1, 0.0, Double.POSITIVE_INFINITY);
      checkIntervalContains(l_neighbour2, 0.0, Double.POSITIVE_INFINITY);
      checkIntervalContains(l_opposite, 0.0, Double.POSITIVE_INFINITY);

      double cosAngle = getCosineAngleWithCosineLaw(l_neighbour1, l_neighbour2, l_opposite);
      double cosAngleDot = getCosineAngleDotWithCosineLaw(l_neighbour1, l_neighbour2, lDot_neighbour2, l_opposite, lDot_opposite);
      double angleDot = -cosAngleDot / sqrt(1 - cosAngle * cosAngle);

      return angleDot;
   }

   public static double getAngleDDotWithCosineLaw(double l_neighbour1, double l_neighbour2, double lDot_neighbour2, double lDDot_neighbour2, double l_opposite,
         double lDot_opposite, double lDDot_opposite)
   {
      checkIntervalContains(l_neighbour1, 0.0, Double.POSITIVE_INFINITY);
      checkIntervalContains(l_neighbour2, 0.0, Double.POSITIVE_INFINITY);
      checkIntervalContains(l_opposite, 0.0, Double.POSITIVE_INFINITY);

      double cosAngle = getCosineAngleWithCosineLaw(l_neighbour1, l_neighbour2, l_opposite);
      double cosAngleDot = getCosineAngleDotWithCosineLaw(l_neighbour1, l_neighbour2, lDot_neighbour2, l_opposite, lDot_opposite);
      double cosAngleDDot = getCosineAngleDDotWithCosineLaw(l_neighbour1, l_neighbour2, lDot_neighbour2, lDDot_neighbour2, l_opposite, lDot_opposite,
                                                            lDDot_opposite);
      double angleDDot = (-cosAngleDDot + cosAngleDDot * square(cosAngle) - cosAngleDot * cosAngleDot * cosAngle) / pow(1 - square(cosAngle), 3.0 / 2.0);

      return angleDDot;
   }

   /**
    * @param random
    * @param sideLengths index 0 is AB, index 1 is BC, ...
    * @param validInitialAngles index 0 is angle A, index 1 is angle B, ...
    * @param minSideLength
    * @param maxSideLength
    */
   public static void generateRandomFourBar(Random random, double[] sideLengths, double[] validInitialAngles, double minSideLength, double maxSideLength)
   {
      double e = RandomNumbers.nextDouble(random, minSideLength, maxSideLength);
      double k1 = random.nextDouble();
      double k2 = random.nextDouble();
      double d1 = e * abs(random.nextGaussian());
      double d2 = e * abs(random.nextGaussian());

      double DE = e * k1;
      double DF = e * k2;
      double BE = e * (1 - k1);
      double BF = e * (1 - k2);

      double AE = d1;
      double CF = d2;

      double DA = sqrt(DE * DE + AE * AE);
      double DAE = atan2(DE, AE);
      double ADE = atan2(AE, DE);

      double AB = sqrt(AE * AE + BE * BE);
      double BAE = atan2(BE, AE);
      double ABE = atan2(AE, BE);

      double CD = sqrt(CF * CF + DF * DF);
      double CDF = atan2(CF, DF);
      double DCF = atan2(DF, CF);

      double BC = sqrt(BF * BF + CF * CF);
      double CBF = atan2(CF, BF);
      double BCF = atan2(BF, CF);

      double A = DAE + BAE;
      double B = ABE + CBF;
      double C = BCF + DCF;
      double D = ADE + CDF;

      sideLengths[0] = AB;
      sideLengths[1] = BC;
      sideLengths[2] = CD;
      sideLengths[3] = DA;

      validInitialAngles[0] = A;
      validInitialAngles[0] = B;
      validInitialAngles[0] = C;
      validInitialAngles[0] = D;
   }

}
