package us.ihmc.robotics.kinematics.fourbar;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Bound;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;

public class FourBarTools
{
   public static boolean isFourBarInverted(FourBar fourBar)
   {
      return fourBar.getEdgeAB().isCrossing() && fourBar.getEdgeCD().isCrossing() || fourBar.getEdgeDA().isCrossing() && fourBar.getEdgeBC().isCrossing();
   }

   public static boolean isFourBarInverted(FourBarVertex fourBarVertex)
   {
      FourBarVertex A = fourBarVertex;
      FourBarEdge ABEdge = A.getNextEdge();
      FourBarEdge BCEdge = ABEdge.getNext();
      FourBarEdge CDEdge = BCEdge.getNext();
      FourBarEdge DAEdge = CDEdge.getNext();

      return ABEdge.isCrossing() && CDEdge.isCrossing() || DAEdge.isCrossing() && BCEdge.isCrossing();
   }

   public static void setToMinAngle(FourBarVertex vertex)
   {
      vertex.setToMin();

      FourBarVertex nextVertex = vertex.getNextVertex();
      if (vertex.isConvex() == nextVertex.isConvex())
         nextVertex.setToMax();
      else
         nextVertex.setToMin();

      FourBarVertex oppositeVertex = vertex.getOppositeVertex();
      if (vertex.isConvex() == oppositeVertex.isConvex())
         oppositeVertex.setToMin();
      else
         oppositeVertex.setToMax();

      FourBarVertex previousVertex = vertex.getPreviousVertex();
      if (vertex.isConvex() == previousVertex.isConvex())
         previousVertex.setToMax();
      else
         previousVertex.setToMin();
   }

   public static void setToMaxAngle(FourBarVertex vertex)
   {
      vertex.setToMax();

      FourBarVertex nextVertex = vertex.getNextVertex();
      if (vertex.isConvex() == nextVertex.isConvex())
         nextVertex.setToMin();
      else
         nextVertex.setToMax();

      FourBarVertex oppositeVertex = vertex.getOppositeVertex();
      if (vertex.isConvex() == oppositeVertex.isConvex())
         oppositeVertex.setToMax();
      else
         oppositeVertex.setToMin();

      FourBarVertex previousVertex = vertex.getPreviousVertex();
      if (vertex.isConvex() == previousVertex.isConvex())
         previousVertex.setToMin();
      else
         previousVertex.setToMax();
   }

   public static Bound update(FourBarVertex vertex, double angle)
   {
      return update(vertex, angle, Double.NaN, Double.NaN);
   }

   public static Bound update(FourBarVertex vertex, double angle, double angleDot)
   {
      return update(vertex, angle, angleDot, Double.NaN);
   }

   public static Bound update(FourBarVertex vertex, double angle, double angleDot, double angleDDot)
   {
      Bound limit = null;

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
      double BD = Double.NaN;
      double AC = Double.NaN;

      // -------------------- Compute angles ----------------------- //

      double cosDAB = Double.NaN;
      double cosBCD = Double.NaN;
      double cosDBC = Double.NaN;
      double cosABD = Double.NaN;
      double cosABC = Double.NaN;
      double sinABDSquare = Double.NaN;
      double sinABD = Double.NaN;
      double sinDBCSquare = Double.NaN;
      double sinDBC = Double.NaN;
      double sinBCDSquare = Double.NaN;
      double sinBCD = Double.NaN;

      if (angle <= A.getMinAngle())
      {
         angle = A.getMinAngle();
         setToMinAngle(A);
         limit = Bound.MIN;
      }
      else if (angle >= A.getMaxAngle())
      {
         angle = A.getMaxAngle();
         setToMaxAngle(A);
         limit = Bound.MAX;
      }
      else
      {
         A.setAngle(angle);
         cosDAB = EuclidCoreTools.cos(angle);
         BD = Math.sqrt(AB * AB + DA * DA - 2.0 * AB * DA * cosDAB);
         BDDiag.setLength(BD);
         cosBCD = cosineAngleWithCosineLaw(BC, CD, BD);
         sinBCDSquare = 1.0 - cosBCD * cosBCD;
         sinBCD = Math.sqrt(sinBCDSquare);
         C.setAngle(fastAcos(cosBCD, sinBCD));
         cosDBC = cosineAngleWithCosineLaw(BC, BD, CD);
         cosABD = cosineAngleWithCosineLaw(AB, BD, DA);
         sinABDSquare = 1 - cosABD * cosABD;
         sinABD = Math.sqrt(sinABDSquare);
         sinDBCSquare = 1 - cosDBC * cosDBC;
         sinDBC = Math.sqrt(sinDBCSquare);

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
            cosABC = MathTools.clamp(cosABD * cosDBC + sinABD * sinDBC, 1.0);
            B.setAngle(Math.abs(fastAcos(cosABC)));
            if (!B.isConvex())
               B.setAngle(-B.getAngle());
            D.setAngle(-A.getAngle() - B.getAngle() - C.getAngle());
         }
         else
         { // Assume the four bar is convex.
            cosABC = MathTools.clamp(cosABD * cosDBC - sinABD * sinDBC, 1.0);
            B.setAngle(Math.abs(fastAcos(cosABC)));
            D.setAngle(2.0 * Math.PI - A.getAngle() - B.getAngle() - C.getAngle());
         }

         AC = Math.sqrt(AB * AB + BC * BC - 2.0 * AB * BC * cosABC);
         ACDiag.setLength(AC);
      }

      // -------------------- Compute angle first derivative ----------------------- //
      if (Double.isNaN(angleDot))
         return limit;

      if (limit != null)
      {
         cosABC = EuclidCoreTools.cos(B.getAngle());
         cosDAB = EuclidCoreTools.cos(A.getAngle());
         cosBCD = Math.cos(C.getAngle());
         sinBCDSquare = 1.0 - cosBCD * cosBCD;
         sinBCD = Math.sqrt(sinBCDSquare);
         cosABD = cosineAngleWithCosineLaw(AB, BD, DA);
         cosDBC = cosineAngleWithCosineLaw(BC, BD, CD);
         sinABDSquare = 1 - cosABD * cosABD;
         sinABD = Math.sqrt(sinABDSquare);
         sinDBCSquare = 1 - cosDBC * cosDBC;
         sinDBC = Math.sqrt(sinDBCSquare);

         AC = Math.sqrt(AB * AB + BC * BC - 2.0 * AB * BC * cosABC);
         BD = Math.sqrt(AB * AB + DA * DA - 2.0 * AB * DA * cosDAB);
         ACDiag.setLength(AC);
         BDDiag.setLength(BD);
      }

      double sinDAB = Math.sin(A.getAngle());

      A.setAngleDot(angleDot);
      double BDDt = DA * AB * sinDAB * angleDot / BD;
      BDDiag.setLengthDot(BDDt);
      double cosBCDDot = cosineAngleDotWithCosineLaw(BC, CD, 0.0, BD, BDDt);
      C.setAngleDot(-cosBCDDot / sinBCD);

      if (!C.isConvex())
         C.setAngleDot(-C.getAngleDot());

      double cosABDDot = cosineAngleDotWithCosineLaw(AB, BD, BDDt, DA, 0.0);
      double angleDtABD = -cosABDDot / sinABD;
      double cosDBCDot = cosineAngleDotWithCosineLaw(BC, BD, BDDt, CD, 0.0);
      double angleDtDBC = -cosDBCDot / sinDBC;

      if (ABEdge.isCrossing())
         angleDtABD = -angleDtABD;
      if (BCEdge.isCrossing())
         angleDtDBC = -angleDtDBC;

      B.setAngleDot(angleDtABD + angleDtDBC);
      if (!B.isConvex())
         B.setAngleDot(-B.getAngleDot());

      D.setAngleDot(-A.getAngleDot() - B.getAngleDot() - C.getAngleDot());

      double sinABC = Math.sin(B.getAngle());
      double ACDt = AB * BC * sinABC * B.getAngleDot() / AC;
      ACDiag.setLengthDot(ACDt);

      // -------------------- Compute angle second derivative ----------------------- //
      if (Double.isNaN(angleDDot))
         return limit;

      A.setAngleDDot(angleDDot);
      double BDDt2 = DA * AB / BD * (cosDAB * angleDot * angleDot + sinDAB * (angleDDot - BDDt * angleDot / BD));
      BDDiag.setLengthDDot(BDDt2);

      double cosBCDDt2 = cosineAngleDDotWithCosineLaw(BC, CD, 0.0, 0.0, BD, BDDt, BDDt2);
      C.setAngleDDot(-(cosBCDDt2 * sinBCDSquare + cosBCDDot * cosBCDDot * cosBCD) / (sinBCDSquare * sinBCD));
      if (!C.isConvex())
         C.setAngleDDot(-C.getAngleDDot());

      double cosABDDt2 = cosineAngleDDotWithCosineLaw(AB, BD, BDDt, BDDt2, DA, 0.0, 0.0);
      double angleDt2ABD = -(cosABDDt2 * sinABDSquare + cosABDDot * cosABDDot * cosABD) / (sinABDSquare * sinABD);
      double cosDBCDt2 = cosineAngleDDotWithCosineLaw(BC, BD, BDDt, BDDt2, CD, 0.0, 0.0);
      double angleDt2DBC = -(cosDBCDt2 * sinDBCSquare + cosDBCDot * cosDBCDot * cosDBC) / (sinDBCSquare * sinDBC);
      if (ABEdge.isCrossing())
         angleDt2ABD = -angleDt2ABD;
      if (BCEdge.isCrossing())
         angleDt2DBC = -angleDt2DBC;

      B.setAngleDDot(angleDt2ABD + angleDt2DBC);
      if (!B.isConvex())
         B.setAngleDDot(-B.getAngleDDot());
      D.setAngleDDot(-A.getAngleDDot() - B.getAngleDDot() - C.getAngleDDot());

      double ACDt2 = AB * BC / AC * (cosABC * B.getAngleDot() * B.getAngleDot() + sinABC * (B.getAngleDDot() - ACDt * B.getAngleDot() / AC));
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
            lowerBound = angleWithCosineLaw(AB, DA, BC - CD);
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
            upperBound = angleWithCosineLaw(AB, DA - CD, BC);
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
            upperBound = Math.PI - angleWithCosineLaw(BC - AB, DA, CD);
         }
      }
      else if (ABEdge.isCrossing())
      { // Assume the four-bar is inverted with DA crossing BC
         if (AB > DA && EuclidGeometryTools.isFormingTriangle(BC, CD, AB - DA))
            lowerBound = 0.0;
         else
            lowerBound = angleWithCosineLaw(DA, AB, CD - BC);

         if (AB > BC && EuclidGeometryTools.isFormingTriangle(DA, AB - BC, CD))
            upperBound = angleWithCosineLaw(DA, AB - BC, CD);
         else
            upperBound = Math.PI - angleWithCosineLaw(AB, CD - DA, BC);
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
      return fastAcos(cosineAngleWithCosineLaw(AB, AC, BC));
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
      return -cosineAngleDotWithCosineLaw(AB, AC, ACDot, BC, BCDot) / Math.sqrt(1 - cosAngle * cosAngle);
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
    * Variation of {@link Math#acos(double)} function that relies on the following identity:
    *
    * <pre>
    * acos(cosX) = 2 atan2(&Sqrt;(1 - cosX<sup>2</sup>), 1 + cosX)
    * </pre>
    *
    * @param cosX the value whose arc cosine is to be returned. The value is clamped to be within [-1,
    *             1].
    * @return the arc cosine of the argument.
    */
   public static double fastAcos(double cosX)
   {
      if (cosX == -1.0)
         return Math.PI;
      else
         return 2.0 * Math.atan2(Math.sqrt(1.0 - cosX * cosX), 1.0 + cosX);
   }

   /**
    * Variation of {@link Math#acos(double)} function that relies on the following identity:
    *
    * <pre>
    * acos(cosX) = 2 atan2(&Sqrt;(1 - cosX<sup>2</sup>), 1 + cosX) = 2 atan2(sinX, 1 + cosX)
    * </pre>
    *
    * @param cosX the value whose arc cosine is to be returned. The value is clamped to be within [-1,
    *             1].
    * @param sinX the precomputed value of <tt>&Sqrt;(1 - cosX<sup>2</sup>)</tt>.
    * @return the arc cosine of the argument.
    */
   public static double fastAcos(double cosX, double sinX)
   {
      if (cosX == -1.0)
         return Math.PI;
      else
         return 2.0 * Math.atan2(sinX, 1.0 + cosX);
   }
}
