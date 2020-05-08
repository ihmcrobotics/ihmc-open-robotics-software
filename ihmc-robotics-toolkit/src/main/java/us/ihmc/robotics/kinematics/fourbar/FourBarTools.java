package us.ihmc.robotics.kinematics.fourbar;

import java.util.Random;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.Bound;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;

public class FourBarTools
{
   public static Bound update(FourBarVertex vertex, double angle)
   {
      FourBarVertex A = vertex;
      FourBarVertex B = vertex.getNextVertex();
      FourBarVertex C = vertex.getOppositeVertex();
      FourBarVertex D = vertex.getPreviousVertex();

      if (angle <= A.getMinAngle())
      {
         A.setToMin();
         B.setToMax();
         C.setToMin();
         D.setToMax();
         return Bound.MIN;
      }
      else if (angle >= A.getMaxAngle())
      {
         A.setToMax();
         B.setToMin();
         C.setToMax();
         D.setToMin();
         return Bound.MAX;
      }

      FourBarEdge ABEdge = A.getNextEdge();
      FourBarEdge BCEdge = B.getNextEdge();
      FourBarEdge CDEdge = C.getNextEdge();
      FourBarEdge DAEdge = D.getNextEdge();
      FourBarDiagonal BDDiag = B.getDiagonal();

      double AB = ABEdge.getLength();
      double BC = BCEdge.getLength();
      double CD = CDEdge.getLength();
      double DA = DAEdge.getLength();

      A.setAngle(angle);
      double BD = EuclidGeometryTools.unknownTriangleSideLengthByLawOfCosine(DA, AB, angle);
      BDDiag.setLength(BD);
      C.setAngle(angleWithCosineLaw(BC, CD, BD));

      double angleDBC = angleWithCosineLaw(BC, BD, CD);
      double angleABD = angleWithCosineLaw(AB, BD, DA);
      B.setAngle(angleABD + angleDBC);
      D.setAngle(2.0 * Math.PI - A.getAngle() - B.getAngle() - C.getAngle());

      return null;
   }

   public static Bound update(FourBarVertex vertex, double angle, double angleDot)
   {
      Bound limit = update(vertex, angle);

      FourBarVertex A = vertex;
      FourBarVertex B = vertex.getNextVertex();
      FourBarVertex C = vertex.getOppositeVertex();
      FourBarVertex D = vertex.getPreviousVertex();

      if (limit == Bound.MIN)
      {
         if (angleDot <= 0.0)
         {
            A.setAngleDot(0.0);
            B.setAngleDot(0.0);
            C.setAngleDot(0.0);
            D.setAngleDot(0.0);
            return limit;
         }
      }
      else if (limit == Bound.MAX)
      {
         if (angleDot >= 0.0)
         {
            A.setAngleDot(0.0);
            B.setAngleDot(0.0);
            C.setAngleDot(0.0);
            D.setAngleDot(0.0);
            return limit;
         }
      }

      FourBarEdge ABEdge = A.getNextEdge();
      FourBarEdge BCEdge = B.getNextEdge();
      FourBarEdge CDEdge = C.getNextEdge();
      FourBarEdge DAEdge = D.getNextEdge();
      FourBarDiagonal BDBiag = B.getDiagonal();

      double AB = ABEdge.getLength();
      double BC = BCEdge.getLength();
      double CD = CDEdge.getLength();
      double DA = DAEdge.getLength();
      double BD = BDBiag.getLength();

      A.setAngleDot(angleDot);
      double BDDt = DA * AB * Math.sin(angle) * angleDot / BD;
      BDBiag.setLengthDot(BDDt);
      C.setAngleDot(angleDotWithCosineLaw(BC, CD, 0.0, BD, BDDt));

      double angleDtDBA = angleDotWithCosineLaw(AB, BD, BDDt, DA, 0.0);
      double angleDtDBC = angleDotWithCosineLaw(BC, BD, BDDt, CD, 0.0);
      B.setAngleDot(angleDtDBA + angleDtDBC);
      D.setAngleDot(-A.getAngleDot() - B.getAngleDot() - C.getAngleDot());

      return null;
   }

   public static Bound update(FourBarVertex vertex, double angle, double angleDot, double angleDDot)
   {
      Bound limit = update(vertex, angle, angleDot);

      FourBarVertex A = vertex;
      FourBarVertex B = vertex.getNextVertex();
      FourBarVertex C = vertex.getOppositeVertex();
      FourBarVertex D = vertex.getPreviousVertex();

      if (limit == Bound.MIN)
      {
         if (angleDot <= 0.0)
         {
            A.setAngleDDot(0.0);
            B.setAngleDDot(0.0);
            C.setAngleDDot(0.0);
            D.setAngleDDot(0.0);
            return limit;
         }
      }
      else if (limit == Bound.MAX)
      {
         if (angleDot >= 0.0)
         {
            A.setAngleDDot(0.0);
            B.setAngleDDot(0.0);
            C.setAngleDDot(0.0);
            D.setAngleDDot(0.0);
            return limit;
         }
      }

      FourBarEdge ABEdge = A.getNextEdge();
      FourBarEdge BCEdge = B.getNextEdge();
      FourBarEdge CDEdge = C.getNextEdge();
      FourBarEdge DAEdge = D.getNextEdge();
      FourBarDiagonal BDDiag = B.getDiagonal();

      double AB = ABEdge.getLength();
      double BC = BCEdge.getLength();
      double CD = CDEdge.getLength();
      double DA = DAEdge.getLength();
      double BD = BDDiag.getLength();
      double BDDt = BDDiag.getLengthDot();

      A.setAngleDDot(angleDDot);
      double BDDt2 = DA * AB / BD * (Math.cos(angle) * angleDot * angleDot + Math.sin(angle) * (angleDDot - BDDt * angleDot / BD));
      BDDiag.setLengthDDot(BDDt2);

      C.setAngleDDot(angleDDotWithCosineLaw(BC, CD, 0.0, 0.0, BD, BDDt, BDDt2));

      double angleDt2DBA = angleDDotWithCosineLaw(AB, BD, BDDt, BDDt2, DA, 0.0, 0.0);
      double angleDt2DBC = angleDDotWithCosineLaw(BC, BD, BDDt, BDDt2, CD, 0.0, 0.0);
      B.setAngleDDot(angleDt2DBA + angleDt2DBC);
      D.setAngleDDot(-A.getAngleDDot() - B.getAngleDDot() - C.getAngleDDot());

      return null;
   }

   public static void updateLimits(FourBarVertex vertex)
   {
      FourBarVertex A = vertex;
      FourBarEdge ABEdge = A.getNextEdge();
      FourBarEdge BCEdge = ABEdge.getNext();
      FourBarEdge CDEdge = BCEdge.getNext();
      FourBarEdge DAEdge = CDEdge.getNext();
      FourBarDiagonal ACDiag = A.getDiagonal();
      FourBarDiagonal BDDiag = ACDiag.getOther();

      double AB = ABEdge.getLength();
      double BC = BCEdge.getLength();
      double CD = CDEdge.getLength();
      double DA = DAEdge.getLength();
      double ACMax = ACDiag.getMaxLength();
      double BDMax = BDDiag.getMaxLength();

      if (DA + CD == ACMax)
         A.setMinAngle(angleWithCosineLaw(ACMax, AB, BC));
      else
         A.setMinAngle(angleWithCosineLaw(ACMax, DA, CD));

      if (DA + AB == BDMax)
         A.setMaxAngle(Math.PI);
      else
         A.setMaxAngle(angleWithCosineLaw(DA, AB, BDMax));
   }

   public static void updateDiagonalMaxLength(FourBarDiagonal diagonal)
   {
      FourBarDiagonal AC = diagonal;
      FourBarEdge AB = AC.getStart().getNextEdge();
      FourBarEdge BC = AB.getNext();
      FourBarEdge CD = BC.getNext();
      FourBarEdge DA = CD.getNext();

      /*
       * We collapse the 2 pairs of edges on each side of the diagonal, the one pair giving the smallest
       * length is the max diagonal length.
       */
      AC.setMaxLength(Math.min(DA.getLength() + CD.getLength(), AB.getLength() + BC.getLength()));
   }

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
