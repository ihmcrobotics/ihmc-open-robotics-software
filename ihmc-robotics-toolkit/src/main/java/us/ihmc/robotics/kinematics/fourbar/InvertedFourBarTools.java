package us.ihmc.robotics.kinematics.fourbar;

import us.ihmc.euclid.geometry.Bound;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;

public class InvertedFourBarTools
{
   public static void updateLimits(FourBarVertex vertex)
   {
      FourBarVertex A = vertex;
      FourBarEdge AB = A.getNextEdge();
      FourBarEdge BC = AB.getNext();
      FourBarEdge CD = BC.getNext();
      FourBarEdge DA = CD.getNext();

      double limit1, limit2;

      if (DA.isCrossing())
      {
         if (DA.getLength() > AB.getLength() && EuclidGeometryTools.isFormingTriangle(BC.getLength(), CD.getLength(), DA.getLength() - AB.getLength()))
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
            limit1 = 0.0;
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
            limit1 = FourBarTools.angleWithCosineLaw(AB.getLength(), DA.getLength(), BC.getLength() - CD.getLength());
         }

         if (DA.getLength() > CD.getLength() && EuclidGeometryTools.isFormingTriangle(AB.getLength(), DA.getLength() - CD.getLength(), BC.getLength()))
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
            limit2 = FourBarTools.angleWithCosineLaw(AB.getLength(), DA.getLength() - CD.getLength(), BC.getLength());
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
            limit2 = Math.PI - FourBarTools.angleWithCosineLaw(BC.getLength() - AB.getLength(), DA.getLength(), CD.getLength());
         }
      }
      else
      { // AB.isCrossing()
         if (AB.getLength() > DA.getLength() && EuclidGeometryTools.isFormingTriangle(BC.getLength(), CD.getLength(), AB.getLength() - DA.getLength()))
            limit1 = 0.0;
         else
            limit1 = FourBarTools.angleWithCosineLaw(DA.getLength(), AB.getLength(), CD.getLength() - BC.getLength());

         if (AB.getLength() > BC.getLength() && EuclidGeometryTools.isFormingTriangle(DA.getLength(), AB.getLength() - BC.getLength(), CD.getLength()))
            limit2 = FourBarTools.angleWithCosineLaw(DA.getLength(), AB.getLength() - BC.getLength(), CD.getLength());
         else
            limit2 = Math.PI - FourBarTools.angleWithCosineLaw(AB.getLength(), CD.getLength() - DA.getLength(), BC.getLength());
      }

      if (A.isConvex())
      {
         A.setMinAngle(limit1);
         A.setMaxAngle(limit2);
      }
      else
      {
         A.setMinAngle(-limit2);
         A.setMaxAngle(-limit1);
      }
   }

   public static Bound update(FourBarVertex vertex, double angle)
   {
      FourBarVertex A = vertex;
      FourBarVertex B = A.getNextVertex();
      FourBarVertex C = B.getNextVertex();
      FourBarVertex D = C.getNextVertex();

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
      FourBarEdge BCEdge = ABEdge.getNext();
      FourBarEdge CDEdge = BCEdge.getNext();
      FourBarEdge DAEdge = CDEdge.getNext();

      FourBarDiagonal ACDiag = A.getDiagonal();
      FourBarDiagonal BDDiag = ACDiag.getOther();

      double AB = ABEdge.getLength();
      double BC = BCEdge.getLength();
      double CD = CDEdge.getLength();
      double DA = DAEdge.getLength();

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

      A.setAngle(angle);
      double BD = EuclidGeometryTools.unknownTriangleSideLengthByLawOfCosine(AB, DA, angle);
      BDDiag.setLength(BD);
      C.setAngle(FourBarTools.angleWithCosineLaw(BC, CD, BD));
      if (!C.isConvex())
         C.setAngle(-C.getAngle());
      double angleDBC = FourBarTools.angleWithCosineLaw(BC, BD, CD);
      double angleABD = FourBarTools.angleWithCosineLaw(AB, BD, DA);
      B.setAngle(Math.abs(angleABD - angleDBC));
      if (!B.isConvex())
         B.setAngle(-B.getAngle());
      D.setAngle(-A.getAngle() - B.getAngle() - C.getAngle());
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
      FourBarDiagonal BDBiag = B.getDiagonal();

      double AB = ABEdge.getLength();
      double BC = BCEdge.getLength();
      double CD = CDEdge.getLength();
      double DA = DAEdge.getLength();
      double AC = ACDiag.getLength();
      double BD = BDBiag.getLength();

      A.setAngleDot(angleDot);
      double BDDt = DA * AB * Math.sin(angle) * angleDot / BD;
      BDBiag.setLengthDot(BDDt);
      C.setAngleDot(FourBarTools.angleDotWithCosineLaw(BC, CD, 0.0, BD, BDDt));
      if (!C.isConvex())
         C.setAngleDot(-C.getAngleDot());

      double angleDtABD = FourBarTools.angleDotWithCosineLaw(AB, BD, BDDt, DA, 0.0);
      double angleDtDBC = FourBarTools.angleDotWithCosineLaw(BC, BD, BDDt, CD, 0.0);

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
      double BDDt2 = DA * AB / BD * (Math.cos(angle) * angleDot * angleDot + Math.sin(angle) * (angleDDot - BDDt * angleDot / BD));
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

      double ACDt2 = AB * BC / AC * (Math.cos(B.getAngle()) * B.getAngleDot() * B.getAngleDot() + Math.sin(B.getAngle()) * (B.getAngleDDot() - ACDt * B.getAngleDot() / AC));
      ACDiag.setLengthDDot(ACDt2);

      return limit;
   }
}
