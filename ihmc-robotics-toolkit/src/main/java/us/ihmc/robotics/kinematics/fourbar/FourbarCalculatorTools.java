package us.ihmc.robotics.kinematics.fourbar;

import java.util.Random;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tools.EuclidCoreTools;

public class FourbarCalculatorTools
{
   /**
    * Calculates the angle at A for a triangle ABC defined by by its lengths.
    * 
    * @param AB the length of the side joining the vertices A and B.
    * @param AC the length of the side joining the vertices A and C.
    * @param BC the length of the side joining the vertices B and C.
    * @return the angle at the vertex A.
    */
   public static double angleWithCosineLaw(double AB, double AC, double BC)
   {
      return Math.acos(cosineAngleWithCosineLaw(AB, AC, BC));
   }

   /**
    * Calculates the cosine value of the angle at A for a triangle ABC defined by by its lengths.
    * 
    * @param AB the length of the side joining the vertices A and B.
    * @param AC the length of the side joining the vertices A and C.
    * @param BC the length of the side joining the vertices B and C.
    * @return the cosine of the angle at the vertex A.
    */
   public static double cosineAngleWithCosineLaw(double AB, double AC, double BC)
   {
      if (AB < 0.0 || AC < 0.0 || BC < 0.0)
         throw new IllegalArgumentException("The triangle side lengths cannot be negative: AB= " + AB + ", AC= " + AC + ", BC= " + BC);

      double ABSquared = EuclidCoreTools.square(AB);
      double ACSquared = EuclidCoreTools.square(AC);
      double BCSquared = EuclidCoreTools.square(BC);
      double cosAngle = (ABSquared + ACSquared - BCSquared) / (2.0 * AB * AC);
      if (cosAngle > 1.0)
         cosAngle = 1.0;
      else if (cosAngle < -1.0)
         cosAngle = -1.0;

      return cosAngle;
   }

   /**
    * Calculates the derivative of the angle at A for a triangle ABC defined by its lengths.
    * <p>
    * The angle is changing due to the sides AC and BC changing. Their rate of change is to provided.
    * This method assumes that the side AB is constant.
    * </p>
    * 
    * @param AB    the length of the side joining the vertices A and B.
    * @param AC    the length of the side joining the vertices A and C.
    * @param ACDot the derivative of the side AC.
    * @param BC    the length of the side joining the vertices B and C.
    * @param BCDot the derivative of the side BC.
    * @return the derivative of the angle at the vertex A.
    */
   public static double angleDotWithCosineLaw(double AB, double AC, double ACDot, double BC, double BCDot)
   {
      double cosAngle = cosineAngleWithCosineLaw(AB, AC, BC);
      double cosAngleDot = cosineAngleDotWithCosineLaw(AB, AC, ACDot, BC, BCDot);
      return -cosAngleDot / Math.sqrt(1 - cosAngle * cosAngle);
   }

   /**
    * Calculates the derivative of the cosine value for the angle at A for a triangle ABC defined by
    * its lengths.
    * <p>
    * The angle is changing due to the sides AC and BC changing. Their rate of change is to provided.
    * This method assumes that the side AB is constant.
    * </p>
    * 
    * @param AB    the length of the side joining the vertices A and B.
    * @param AC    the length of the side joining the vertices A and C.
    * @param ACDot the derivative of the side AC.
    * @param BC    the length of the side joining the vertices B and C.
    * @param BCDot the derivative of the side BC.
    * @return the derivative of the cosine value for the angle at the vertex A.
    */
   public static double cosineAngleDotWithCosineLaw(double AB, double AC, double ACDot, double BC, double BCDot)
   {
      if (AB < 0.0 || AC < 0.0 || BC < 0.0)
         throw new IllegalArgumentException("The triangle side lengths cannot be negative: AB= " + AB + ", AC= " + AC + ", BC= " + BC);

      double ABSquared = EuclidCoreTools.square(AB);
      double ACSquared = EuclidCoreTools.square(AC);
      double BCSquared = EuclidCoreTools.square(BC);

      return (0.5 * (ACSquared - ABSquared + BCSquared) * ACDot - AC * BC * BCDot) / (ACSquared * AB);
   }

   /**
    * Calculates the second derivative of the angle at A for a triangle ABC defined by its lengths.
    * <p>
    * The angle is changing due to the sides AC and BC changing. Their rate of change is to provided.
    * This method assumes that the side AB is constant.
    * </p>
    * 
    * @param AB     the length of the side joining the vertices A and B.
    * @param AC     the length of the side joining the vertices A and C.
    * @param ACDot  the derivative of the side AC.
    * @param ACDDot the second derivative of the side AC.
    * @param BC     the length of the side joining the vertices B and C.
    * @param BCDot  the derivative of the side BC.
    * @param BCDDot the second derivative of the side BC.
    * @return the second derivative of the angle at the vertex A.
    */
   public static double angleDDotWithCosineLaw(double AB, double AC, double ACDot, double ACDDot, double BC, double BCDot, double BCDDot)
   {
      double cosAngle = cosineAngleWithCosineLaw(AB, AC, BC);
      double cosAngleSquared = MathTools.square(cosAngle);
      double sinAngleSquared = 1.0 - cosAngleSquared;
      double sinAngle = Math.sqrt(sinAngleSquared);

      double cosAngleDot = cosineAngleDotWithCosineLaw(AB, AC, ACDot, BC, BCDot);
      double cosAngleDotSquared = cosAngleDot * cosAngleDot;

      double cosAngleDDot = cosineAngleDDotWithCosineLaw(AB, AC, ACDot, ACDDot, BC, BCDot, BCDDot);

      return -(cosAngleDDot * sinAngleSquared + cosAngleDotSquared * cosAngle) / MathTools.cube(sinAngle);
   }

   /**
    * Calculates the second derivative of the cosine value for the angle at A for a triangle ABC
    * defined by its lengths.
    * <p>
    * The angle is changing due to the sides AC and BC changing. Their rate of change is to provided.
    * This method assumes that the side AB is constant.
    * </p>
    * 
    * @param AB     the length of the side joining the vertices A and B.
    * @param AC     the length of the side joining the vertices A and C.
    * @param ACDot  the derivative of the side AC.
    * @param ACDDot the second derivative of the side AC.
    * @param BC     the length of the side joining the vertices B and C.
    * @param BCDot  the derivative of the side BC.
    * @param BCDDot the second derivative of the side BC.
    * @return the second derivative of the cosine value for the angle at the vertex A.
    */
   public static double cosineAngleDDotWithCosineLaw(double AB, double AC, double ACDot, double ACDDot, double BC, double BCDot, double BCDDot)
   {
      if (AB < 0.0 || AC < 0.0 || BC < 0.0)
         throw new IllegalArgumentException("The triangle side lengths cannot be negative: AB= " + AB + ", AC= " + AC + ", BC= " + BC);

      double ABSquared = EuclidCoreTools.square(AB);
      double ACSquared = MathTools.square(AC);
      double ACCubed = ACSquared * AC;
      double BCSquared = EuclidCoreTools.square(BC);

      double BCDotSquared = EuclidCoreTools.square(BCDot);
      double ACDotSquared = EuclidCoreTools.square(ACDot);

      double cosAngleDDot = 0.5 * (ACCubed + AC * (BCSquared - ABSquared)) * ACDDot;
      cosAngleDDot += -ACSquared * (BCDotSquared + BC * BCDDot) + (ABSquared - BCSquared) * ACDotSquared + 2.0 * AC * ACDot * BC * BCDot;
      cosAngleDDot /= ACCubed * AB;

      return cosAngleDDot;
   }

   /**
    * @param random
    * @param sideLengths        index 0 is AB, index 1 is BC, ...
    * @param validInitialAngles index 0 is angle A, index 1 is angle B, ...
    * @param minSideLength
    * @param maxSideLength
    */
   public static void generateRandomFourBar(Random random, double[] sideLengths, double[] validInitialAngles, double minSideLength, double maxSideLength)
   {
      double e = RandomNumbers.nextDouble(random, minSideLength, maxSideLength);
      double k1 = random.nextDouble();
      double k2 = random.nextDouble();
      double d1 = e * Math.abs(random.nextGaussian());
      double d2 = e * Math.abs(random.nextGaussian());

      double DE = e * k1;
      double DF = e * k2;
      double BE = e * (1 - k1);
      double BF = e * (1 - k2);

      double AE = d1;
      double CF = d2;

      double DA = Math.sqrt(DE * DE + AE * AE);
      double DAE = Math.atan2(DE, AE);
      double ADE = Math.atan2(AE, DE);

      double AB = Math.sqrt(AE * AE + BE * BE);
      double BAE = Math.atan2(BE, AE);
      double ABE = Math.atan2(AE, BE);

      double CD = Math.sqrt(CF * CF + DF * DF);
      double CDF = Math.atan2(CF, DF);
      double DCF = Math.atan2(DF, CF);

      double BC = Math.sqrt(BF * BF + CF * CF);
      double CBF = Math.atan2(CF, BF);
      double BCF = Math.atan2(BF, CF);

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
