package us.ihmc.robotics.kinematics.fourbar;

import us.ihmc.euclid.geometry.Bound;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

/**
 * Notation is the same used in {@link FourBar}. This implementation is dedicated for inverted
 * four-bar linkage where 2 of the sides are crossing each other:
 * 
 * <pre>
 * +-------+ 
 *  \     /
 *   \   /
 *    \ /
 *     X
 *    / \
 *   /   \
 *  +-----+
 * </pre>
 */
public class InvertedFourBar
{
   private final FourBarVertex A = new InvertedFourBarVertex("A");
   private final FourBarVertex B = new InvertedFourBarVertex("B");
   private final FourBarVertex C = new InvertedFourBarVertex("C");
   private final FourBarVertex D = new InvertedFourBarVertex("D");
   private final FourBarVertex[] vertices = {A, B, C, D};

   private final FourBarEdge AB = new FourBarEdge("AB");
   private final FourBarEdge BC = new FourBarEdge("BC");
   private final FourBarEdge CD = new FourBarEdge("CD");
   private final FourBarEdge DA = new FourBarEdge("DA");

   private final FourBarDiagonal AC = new FourBarDiagonal("AC");
   private final FourBarDiagonal BD = new FourBarDiagonal("AC");

   public InvertedFourBar()
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

      boolean isAConvex = !EuclidGeometryTools.isPoint2DOnRightSideOfLine2D(A, D, B);
      boolean isBConvex = !EuclidGeometryTools.isPoint2DOnRightSideOfLine2D(B, A, C);
      boolean isCConvex = !EuclidGeometryTools.isPoint2DOnRightSideOfLine2D(C, B, D);
      boolean isDConvex = !EuclidGeometryTools.isPoint2DOnRightSideOfLine2D(D, C, A);

      int convexCount = isAConvex ? 1 : 0;
      convexCount += isBConvex ? 1 : 0;
      convexCount += isCConvex ? 1 : 0;
      convexCount += isDConvex ? 1 : 0;

      if (convexCount == 4)
         throw new UnsupportedOperationException("The quadrilateral ABCD is regular, use " + FourBar.class.getSimpleName() + " instead.");
      if (convexCount == 1 || convexCount == 3)
         throw new UnsupportedOperationException("The quadrilateral ABCD is concave at " + (4 - convexCount) + "vertices, only handles 2 concave vertices.");

      boolean isABFlipped = !isAConvex && !isBConvex;
      boolean isBCFlipped = !isBConvex && !isCConvex;
      boolean isCDFlipped = !isCConvex && !isDConvex;
      boolean isDAFlipped = !isDConvex && !isAConvex;

      this.A.setConvex(isAConvex);
      this.B.setConvex(isBConvex);
      this.C.setConvex(isCConvex);
      this.D.setConvex(isDConvex);

      AB.setFlipped(isABFlipped);
      BC.setFlipped(isBCFlipped);
      CD.setFlipped(isCDFlipped);
      DA.setFlipped(isDAFlipped);

      if (AB.isFlipped() || CD.isFlipped())
      {
         DA.setCrossing(true);
         BC.setCrossing(true);
         AB.setCrossing(false);
         CD.setCrossing(false);
      }
      else
      {
         DA.setCrossing(false);
         BC.setCrossing(false);
         AB.setCrossing(true);
         CD.setCrossing(true);
      }
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

   public void setToMin(FourBarAngle source)
   {
      FourBarVertex startVertex = vertices[source.ordinal()];
      startVertex.setToMin();

      FourBarVertex nextVertex = startVertex.getNextVertex();
      if (startVertex.isConvex() == nextVertex.isConvex())
         nextVertex.setToMax();
      else
         nextVertex.setToMin();

      FourBarVertex oppositeVertex = startVertex.getOppositeVertex();
      if (startVertex.isConvex() == oppositeVertex.isConvex())
         oppositeVertex.setToMin();
      else
         oppositeVertex.setToMax();

      FourBarVertex previousVertex = startVertex.getPreviousVertex();
      if (startVertex.isConvex() == previousVertex.isConvex())
         previousVertex.setToMax();
      else
         previousVertex.setToMin();
   }

   public void setToMax(FourBarAngle source)
   {
      FourBarVertex startVertex = vertices[source.ordinal()];
      startVertex.setToMax();

      FourBarVertex nextVertex = startVertex.getNextVertex();
      if (startVertex.isConvex() == nextVertex.isConvex())
         nextVertex.setToMin();
      else
         nextVertex.setToMax();

      FourBarVertex oppositeVertex = startVertex.getOppositeVertex();
      if (startVertex.isConvex() == oppositeVertex.isConvex())
         oppositeVertex.setToMax();
      else
         oppositeVertex.setToMin();

      FourBarVertex previousVertex = startVertex.getPreviousVertex();
      if (startVertex.isConvex() == previousVertex.isConvex())
         previousVertex.setToMin();
      else
         previousVertex.setToMax();
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
   public Bound update(FourBarAngle source, double angle)
   {
      return InvertedFourBarTools.update(vertices[source.ordinal()], angle);
   }

   public Bound update(FourBarAngle source, double angle, double angleDot)
   {
      return FourBarTools.update(vertices[source.ordinal()], angle, angleDot);
   }

   public Bound update(FourBarAngle source, double angle, double angleDot, double angleDDot)
   {
      return FourBarTools.update(vertices[source.ordinal()], angle, angleDDot);
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

   public FourBarVertex getVertex(FourBarAngle angle)
   {
      return vertices[angle.ordinal()];
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
