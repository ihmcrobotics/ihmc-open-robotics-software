package us.ihmc.robotics.kinematics.fourbar;

import us.ihmc.commons.MathTools;
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
      FourBarDiagonal ACDiag = A.getDiagonal();
      FourBarDiagonal BDDiag = B.getDiagonal();

      double AB = ABEdge.getLength();
      double BC = BCEdge.getLength();
      double CD = CDEdge.getLength();
      double DA = DAEdge.getLength();

      A.setAngle(angle);
      double BD = EuclidGeometryTools.unknownTriangleSideLengthByLawOfCosine(AB, DA, angle);
      BDDiag.setLength(BD);
      C.setAngle(angleWithCosineLaw(BC, CD, BD));
      double angleDBC = angleWithCosineLaw(BC, BD, CD);
      double angleABD = angleWithCosineLaw(AB, BD, DA);

      if (ABEdge.isCrossing() || BCEdge.isCrossing())
      { // Inverted four bar
         /*
          * @formatter:off
          *  +A------B+    +D------A+    +C------D+    +B------C+
          *    \    /        \    /        \    /        \    /  
          *     \  /          \  /          \  /          \  /   
          *      \/     or     \/     or     \/     or     \/    
          *      /\            /\            /\            /\    
          *     /  \          /  \          /  \          /  \   
          *    /    \        /    \        /    \        /    \  
          *  -C------D-    -B------C-    -A------B-    -D------A-
          * @formatter:on
          */
         if (!C.isConvex())
            C.setAngle(-C.getAngle());
         B.setAngle(Math.abs(angleABD - angleDBC));
         if (!B.isConvex())
            B.setAngle(-B.getAngle());
         D.setAngle(-A.getAngle() - B.getAngle() - C.getAngle());
      }
      else
      { // Assume the four bar is convex.
         B.setAngle(angleABD + angleDBC);
         D.setAngle(2.0 * Math.PI - A.getAngle() - B.getAngle() - C.getAngle());
      }

      ACDiag.setLength(EuclidGeometryTools.unknownTriangleSideLengthByLawOfCosine(AB, BC, B.getAngle()));

      return null;
   }

   public static Bound update(FourBarVertex vertex, double angle, double angleDot)
   {
      Bound limit = update(vertex, angle);

      FourBarVertex A = vertex;
      FourBarVertex B = vertex.getNextVertex();
      FourBarVertex C = vertex.getOppositeVertex();
      FourBarVertex D = vertex.getPreviousVertex();

      FourBarEdge ABEdge = A.getNextEdge();
      FourBarEdge BCEdge = B.getNextEdge();
      FourBarEdge CDEdge = C.getNextEdge();
      FourBarEdge DAEdge = D.getNextEdge();
      FourBarDiagonal ACDiag = A.getDiagonal();
      FourBarDiagonal BDDiag = B.getDiagonal();

      double AB = ABEdge.getLength();
      double BC = BCEdge.getLength();
      double CD = CDEdge.getLength();
      double DA = DAEdge.getLength();
      double AC = ACDiag.getLength();
      double BD = BDDiag.getLength();

      A.setAngleDot(angleDot);
      double BDDt = DA * AB * Math.sin(A.getAngle()) * angleDot / BD;
      BDDiag.setLengthDot(BDDt);
      C.setAngleDot(FourBarTools.angleDotWithCosineLaw(BC, CD, 0.0, BD, BDDt));
      double angleDtABD = FourBarTools.angleDotWithCosineLaw(AB, BD, BDDt, DA, 0.0);
      double angleDtDBC = FourBarTools.angleDotWithCosineLaw(BC, BD, BDDt, CD, 0.0);

      if (!C.isConvex())
         C.setAngleDot(-C.getAngleDot());

      if (ABEdge.isCrossing())
         angleDtABD = -angleDtABD;
      if (BCEdge.isCrossing())
         angleDtDBC = -angleDtDBC;

      B.setAngleDot(angleDtABD + angleDtDBC);
      if (!B.isConvex())
         B.setAngleDot(-B.getAngleDot());

      D.setAngleDot(-A.getAngleDot() - B.getAngleDot() - C.getAngleDot());

      double ACDt = AB * BC * Math.sin(B.getAngle()) * B.getAngleDot() / AC;
      ACDiag.setLengthDot(ACDt);

      return limit;
   }

   public static Bound update(FourBarVertex vertex, double angle, double angleDot, double angleDDot)
   {
      Bound limit = update(vertex, angle, angleDot);

      FourBarVertex A = vertex;
      FourBarVertex B = vertex.getNextVertex();
      FourBarVertex C = vertex.getOppositeVertex();
      FourBarVertex D = vertex.getPreviousVertex();

      FourBarEdge ABEdge = A.getNextEdge();
      FourBarEdge BCEdge = B.getNextEdge();
      FourBarEdge CDEdge = C.getNextEdge();
      FourBarEdge DAEdge = D.getNextEdge();
      FourBarDiagonal ACDiag = A.getDiagonal();
      FourBarDiagonal BDDiag = B.getDiagonal();

      double AB = ABEdge.getLength();
      double BC = BCEdge.getLength();
      double CD = CDEdge.getLength();
      double DA = DAEdge.getLength();
      double AC = ACDiag.getLength();
      double BD = BDDiag.getLength();
      double ACDt = ACDiag.getLengthDot();
      double BDDt = BDDiag.getLengthDot();

      A.setAngleDDot(angleDDot);
      double BDDt2 = DA * AB / BD * (Math.cos(A.getAngle()) * angleDot * angleDot + Math.sin(A.getAngle()) * (angleDDot - BDDt * angleDot / BD));
      BDDiag.setLengthDDot(BDDt2);
      C.setAngleDDot(FourBarTools.angleDDotWithCosineLaw(BC, CD, 0.0, 0.0, BD, BDDt, BDDt2));
      if (!C.isConvex())
         C.setAngleDDot(-C.getAngleDDot());

      double angleDt2ABD = FourBarTools.angleDDotWithCosineLaw(AB, BD, BDDt, BDDt2, DA, 0.0, 0.0);
      double angleDt2DBC = FourBarTools.angleDDotWithCosineLaw(BC, BD, BDDt, BDDt2, CD, 0.0, 0.0);
      if (ABEdge.isCrossing())
         angleDt2ABD = -angleDt2ABD;
      if (BCEdge.isCrossing())
         angleDt2DBC = -angleDt2DBC;

      B.setAngleDDot(angleDt2ABD + angleDt2DBC);
      if (!B.isConvex())
         B.setAngleDDot(-B.getAngleDDot());
      D.setAngleDDot(-A.getAngleDDot() - B.getAngleDDot() - C.getAngleDDot());

      double ACDt2 = AB * BC / AC
            * (Math.cos(B.getAngle()) * B.getAngleDot() * B.getAngleDot() + Math.sin(B.getAngle()) * (B.getAngleDDot() - ACDt * B.getAngleDot() / AC));
      ACDiag.setLengthDDot(ACDt2);

      return limit;
   }

   public static void updateLimits(FourBarVertex vertex)
   {
      FourBarVertex A = vertex;
      FourBarEdge ABEdge = A.getNextEdge();
      FourBarEdge BCEdge = ABEdge.getNext();
      FourBarEdge CDEdge = BCEdge.getNext();
      FourBarEdge DAEdge = CDEdge.getNext();

      double AB = ABEdge.getLength();
      double BC = BCEdge.getLength();
      double CD = CDEdge.getLength();
      double DA = DAEdge.getLength();

      double lowerBound, upperBound;

      if (DAEdge.isCrossing())
      { // Assume the four-bar is inverted with DA crossing BC
         if (DA > AB && EuclidGeometryTools.isFormingTriangle(BC, CD, DA - AB))
         {
            /*
                * @formatter:off
                *   A------B      A
                *    \    /        \
                *     \  /          \
                *      \/    =>      B
                *      /\           / \
                *     /  \         /   \
                *    /    \       /     \
                *   C------D     C-------D
                * @formatter:on
                */
            lowerBound = 0.0;
         }
         else
         {
            /*
             * @formatter:off
             *   A------B      A-----B
             *    \    /        \   /
             *     \  /          \ /
             *      \/    =>      D
             *      /\           /
             *     /  \         C
             *    C----D
             * @formatter:on
             */
            lowerBound = FourBarTools.angleWithCosineLaw(AB, DA, BC - CD);
         }

         if (DA > CD && EuclidGeometryTools.isFormingTriangle(AB, DA - CD, BC))
         {
            /*
             * @formatter:off
             *   A------B      A-----B
             *    \    /        \   /
             *     \  /          \ /
             *      \/    =>      C
             *      /\             \
             *     /  \             \
             *    /    \             \
             *   C------D             D
             * @formatter:on
             */
            upperBound = FourBarTools.angleWithCosineLaw(AB, DA - CD, BC);
         }
         else
         {
            /*
             * @formatter:off
             *    A----B           B
             *     \  /           /
             *      \/           A 
             *      /\    =>    / \ 
             *     /  \        /   \
             *    /    \      C-----D
             *   C------D
             * @formatter:on
             */
            upperBound = Math.PI - FourBarTools.angleWithCosineLaw(BC - AB, DA, CD);
         }
      }
      else if (ABEdge.isCrossing())
      { // Assume the four-bar is inverted with DA crossing BC
         if (AB > DA && EuclidGeometryTools.isFormingTriangle(BC, CD, AB - DA))
            lowerBound = 0.0;
         else
            lowerBound = FourBarTools.angleWithCosineLaw(DA, AB, CD - BC);

         if (AB > BC && EuclidGeometryTools.isFormingTriangle(DA, AB - BC, CD))
            upperBound = FourBarTools.angleWithCosineLaw(DA, AB - BC, CD);
         else
            upperBound = Math.PI - FourBarTools.angleWithCosineLaw(AB, CD - DA, BC);
      }
      else
      { // Assume the four-bar is convex.
         double ACMax = Math.min(DA + CD, AB + BC);
         double BDMax = Math.min(BC + CD, DA + AB);

         if (DA + CD == ACMax)
            lowerBound = angleWithCosineLaw(ACMax, AB, BC);
         else
            lowerBound = angleWithCosineLaw(ACMax, DA, CD);

         if (DA + AB == BDMax)
            upperBound = Math.PI;
         else
            upperBound = angleWithCosineLaw(DA, AB, BDMax);
      }

      if (A.isConvex())
      {
         A.setMinAngle(lowerBound);
         A.setMaxAngle(upperBound);
      }
      else
      {
         A.setMinAngle(-upperBound);
         A.setMaxAngle(-lowerBound);
      }
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
}
