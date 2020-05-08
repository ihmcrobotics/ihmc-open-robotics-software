package us.ihmc.robotics.kinematics.fourbar;

import us.ihmc.euclid.geometry.Bound;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class FourBar
{
   /*
    * @formatter:off
    *  Representation of the four bar with name correspondences:
    *        DA
    *    D--------A
    *    |\      /|
    *    | \DB  / |
    *    |  \  /  |
    * CD |   \/   | AB
    *    |   /\   |
    *    |  /  \  |
    *    | /AC  \ |
    *    |/      \|
    *    C--------B
    *        BC
    * Angle name convention:
    * - Inner angle at vertex A: DAB
    * - Inner angle at vertex B: ABC
    * - Inner angle at vertex C: BCD
    * - Inner angle at vertex D: CDA
    * @formatter:on
    */

   private final FourBarVertex A = new FourBarVertex("A");
   private final FourBarVertex B = new FourBarVertex("B");
   private final FourBarVertex C = new FourBarVertex("C");
   private final FourBarVertex D = new FourBarVertex("D");

   private final FourBarEdge AB = new FourBarEdge("AB");
   private final FourBarEdge BC = new FourBarEdge("BC");
   private final FourBarEdge CD = new FourBarEdge("CD");
   private final FourBarEdge DA = new FourBarEdge("DA");

   private final FourBarDiagonal AC = new FourBarDiagonal("AC");
   private final FourBarDiagonal BD = new FourBarDiagonal("AC");

   public enum Angle
   {
      ABC, BCD, CDA, DAB
   };

   public FourBar()
   {
      A.setup(DA, AB, AC);
      B.setup(AB, BC, BD);
      C.setup(BC, CD, AC);
      D.setup(CD, DA, BD);

      AB.setup(A, B, DA, BC);
      BC.setup(B, C, AB, CD);
      CD.setup(C, D, BC, DA);
      DA.setup(D, A, CD, AB);

      AC.setup(A, C, BD);
      BD.setup(B, D, AC);
   }

   public void setup(Point2DReadOnly A, Point2DReadOnly B, Point2DReadOnly C, Point2DReadOnly D)
   {
      setSideLengths(A.distance(B), B.distance(C), C.distance(D), D.distance(A));

      if (!EuclidGeometryTools.isPoint2DOnRightSideOfLine2D(A, D, B))
         throw new UnsupportedOperationException("The four-bar is concave at the vertex A.");
      if (!EuclidGeometryTools.isPoint2DOnRightSideOfLine2D(B, A, C))
         throw new UnsupportedOperationException("The four-bar is concave at the vertex B.");
      if (!EuclidGeometryTools.isPoint2DOnRightSideOfLine2D(C, B, D))
         throw new UnsupportedOperationException("The four-bar is concave at the vertex C.");
      if (!EuclidGeometryTools.isPoint2DOnRightSideOfLine2D(D, C, A))
         throw new UnsupportedOperationException("The four-bar is concave at the vertex D.");
   }

   public void setToNaN()
   {
      A.setToNaN();
      B.setToNaN();
      C.setToNaN();
      D.setToNaN();

      AB.setToNaN();
      BC.setToNaN();
      CD.setToNaN();
      DA.setToNaN();

      AC.setToNaN();
      BD.setToNaN();
   }

   public void setSideLengths(double AB, double BC, double CD, double DA)
   {
      setToNaN();
      this.AB.setLength(AB);
      this.BC.setLength(BC);
      this.CD.setLength(CD);
      this.DA.setLength(DA);
   }

   /**
    * <p>
    * Updates internally: all the angles and diagonal lengths.
    * </p>
    * 
    * @param source
    * @param angle
    * @return
    */
   public Bound update(Angle source, double angle)
   {
      switch (source)
      {
         case DAB:
            return FourBarTools.update(getVertexA(), angle);
         case ABC:
            return FourBarTools.update(getVertexB(), angle);
         case BCD:
            return FourBarTools.update(getVertexC(), angle);
         case CDA:
            return FourBarTools.update(getVertexD(), angle);
         default:
            throw new IllegalStateException();
      }
   }

   public Bound update(Angle source, double angle, double angleDot)
   {
      switch (source)
      {
         case DAB:
            return FourBarTools.update(getVertexA(), angle, angleDot);
         case ABC:
            return FourBarTools.update(getVertexB(), angle, angleDot);
         case BCD:
            return FourBarTools.update(getVertexC(), angle, angleDot);
         case CDA:
            return FourBarTools.update(getVertexD(), angle, angleDot);
         default:
            throw new IllegalStateException();
      }
   }

   public Bound update(Angle source, double angle, double angleDot, double angleDDot)
   {
      switch (source)
      {
         case DAB:
            return FourBarTools.update(getVertexA(), angle, angleDot, angleDDot);
         case ABC:
            return FourBarTools.update(getVertexB(), angle, angleDot, angleDDot);
         case BCD:
            return FourBarTools.update(getVertexC(), angle, angleDot, angleDDot);
         case CDA:
            return FourBarTools.update(getVertexD(), angle, angleDot, angleDDot);
         default:
            throw new IllegalStateException();
      }
   }

   public FourBarVertex getVertexA()
   {
      return A;
   }

   public FourBarVertex getVertexB()
   {
      return B;
   }

   public FourBarVertex getVertexC()
   {
      return C;
   }

   public FourBarVertex getVertexD()
   {
      return D;
   }

   public FourBarEdge getEdgeAB()
   {
      return AB;
   }

   public FourBarEdge getEdgeBC()
   {
      return BC;
   }

   public FourBarEdge getEdgeCD()
   {
      return CD;
   }

   public FourBarEdge getEdgeDA()
   {
      return DA;
   }

   public FourBarDiagonal getDiagonalAC()
   {
      return AC;
   }

   public FourBarDiagonal getDiagonalBD()
   {
      return BD;
   }

   public double getAngleDAB()
   {
      return getVertexA().getAngle();
   }

   public double getAngleABC()
   {
      return getVertexB().getAngle();
   }

   public double getAngleBCD()
   {
      return getVertexC().getAngle();
   }

   public double getAngleCDA()
   {
      return getVertexD().getAngle();
   }

   public double getAngleDtDAB()
   {
      return getVertexA().getAngleDot();
   }

   public double getAngleDtABC()
   {
      return getVertexB().getAngleDot();
   }

   public double getAngleDtBCD()
   {
      return getVertexC().getAngleDot();
   }

   public double getAngleDtCDA()
   {
      return getVertexD().getAngleDot();
   }

   public double getAngleDt2DAB()
   {
      return getVertexA().getAngleDDot();
   }

   public double getAngleDt2ABC()
   {
      return getVertexB().getAngleDDot();
   }

   public double getAngleDt2BCD()
   {
      return getVertexC().getAngleDDot();
   }

   public double getAngleDt2CDA()
   {
      return getVertexD().getAngleDDot();
   }

   public double getMinDAB()
   {
      return getVertexA().getMinAngle();
   }

   public double getMaxDAB()
   {
      return getVertexA().getMaxAngle();
   }

   public double getMinABC()
   {
      return getVertexB().getMinAngle();
   }

   public double getMaxABC()
   {
      return getVertexB().getMaxAngle();
   }

   public double getMinBCD()
   {
      return getVertexC().getMinAngle();
   }

   public double getMaxBCD()
   {
      return getVertexC().getMaxAngle();
   }

   public double getMinCDA()
   {
      return getVertexD().getMinAngle();
   }

   public double getMaxCDA()
   {
      return getVertexD().getMaxAngle();
   }

   public double getAB()
   {
      return getEdgeAB().getLength();
   }

   public double getBC()
   {
      return getEdgeBC().getLength();
   }

   public double getCD()
   {
      return getEdgeCD().getLength();
   }

   public double getDA()
   {
      return getEdgeDA().getLength();
   }

   @Override
   public String toString()
   {
      return String.format("Lengths: AB=%f, BC=%f, CD=%f, DA=%f\nAngles: DAB=%f, ABC=%f, BCD=%f, CDA=%f",
                           AB.getLength(),
                           BC.getLength(),
                           CD.getLength(),
                           DA.getLength(),
                           A.getAngle(),
                           B.getAngle(),
                           C.getAngle(),
                           D.getAngle());
   }
}
