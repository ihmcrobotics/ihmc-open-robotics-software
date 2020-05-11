package us.ihmc.robotics.kinematics.fourbar;

import us.ihmc.euclid.geometry.Bound;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

/**
 * Notation is the same used in {@link FourBar}. This implementation is dedicated for inverted
 * four-bar linkage where 2 of the sides are crossing each other:
 *
 * <pre>
 *  +A------B+    +D------A+    +C------D+    +B------C+
 *    \    /        \    /        \    /        \    /
 *     \  /          \  /          \  /          \  /
 *      \/     or     \/     or     \/     or     \/
 *      /\            /\            /\            /\
 *     /  \          /  \          /  \          /  \
 *    /    \        /    \        /    \        /    \
 *  -C------D-    -B------C-    -A------B-    -D------A-
 * </pre>
 *
 * Note the symbols "-" and "+" next to each vertex indicating their winding, i.e. "-" for
 * counter-clockwise and "+" for clockwise. A vertex is assumed to be convex when its winding is
 * clockwise.
 */
public class InvertedFourBar implements FourBarInterface
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
   private final FourBarDiagonal BD = new FourBarDiagonal("BD");

   /**
    * Created a new four bar. The new instance needs to be configured.
    *
    * @see #setup(Point2DReadOnly, Point2DReadOnly, Point2DReadOnly, Point2DReadOnly)
    */
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

   /**
    * {@inheritDoc}
    *
    * @throws UnsupportedOperationException if the resulting four bar linkage is an inverted four bar
    *                                       as shown in the header of this class.
    */
   @Override
   public void setup(Point2DReadOnly A, Point2DReadOnly B, Point2DReadOnly C, Point2DReadOnly D)
   {
      setToNaN();
      AB.setLength(A.distance(B));
      BC.setLength(B.distance(C));
      CD.setLength(C.distance(D));
      DA.setLength(D.distance(A));

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

      this.A.setConvex(isAConvex);
      this.B.setConvex(isBConvex);
      this.C.setConvex(isCConvex);
      this.D.setConvex(isDConvex);
   }

   /**
    * Sets up this four bar linkage given its 4 side lengths.
    * <p>
    * The state of this four bar linkage is cleared, i.e. angles, lengths, and their derivatives are
    * cleared and need to be update using one of the update methods.
    * </p>
    *
    * @param AB                 the length of the side joining the vertices A and B.
    * @param BC                 the length of the side joining the vertices B and C.
    * @param CD                 the length of the side joining the vertices C and D.
    * @param DA                 the length of the side joining the vertices D and A.
    * @param firstConvexVertex  indicate the first vertex that is convex, i.e. clockwise winding.
    * @param secondConvexVertex indicate the second vertex that is convex, i.e. clockwise winding.
    * @throws IllegalArgumentException if {@code firstConvexVertex == secondConvexVertex}.
    */
   public void setSideLengths(double AB, double BC, double CD, double DA, FourBarAngle firstConvexVertex, FourBarAngle secondConvexVertex)
   {
      setToNaN();
      this.AB.setLength(AB);
      this.BC.setLength(BC);
      this.CD.setLength(CD);
      this.DA.setLength(DA);

      if (firstConvexVertex == secondConvexVertex)
         throw new IllegalArgumentException("firstConvexVertex and secondConvexVertex cannot be equal.");

      boolean isAConvex = firstConvexVertex == FourBarAngle.DAB || secondConvexVertex == FourBarAngle.DAB;
      boolean isBConvex = firstConvexVertex == FourBarAngle.ABC || secondConvexVertex == FourBarAngle.ABC;
      boolean isCConvex = firstConvexVertex == FourBarAngle.BCD || secondConvexVertex == FourBarAngle.BCD;
      boolean isDConvex = firstConvexVertex == FourBarAngle.CDA || secondConvexVertex == FourBarAngle.CDA;

      int convexCount = isAConvex ? 1 : 0;
      convexCount += isBConvex ? 1 : 0;
      convexCount += isCConvex ? 1 : 0;
      convexCount += isDConvex ? 1 : 0;

      if (convexCount == 4)
         throw new UnsupportedOperationException("The quadrilateral ABCD is regular, use " + FourBar.class.getSimpleName() + " instead.");
      if (convexCount == 1 || convexCount == 3)
         throw new UnsupportedOperationException("The quadrilateral ABCD is concave at " + (4 - convexCount) + "vertices, only handles 2 concave vertices.");

      A.setConvex(isAConvex);
      B.setConvex(isBConvex);
      C.setConvex(isCConvex);
      D.setConvex(isDConvex);
   }

   @Override
   public Bound update(FourBarAngle source, double angle)
   {
      return InvertedFourBarTools.update(vertices[source.ordinal()], angle);
   }

   @Override
   public Bound update(FourBarAngle source, double angle, double angleDot)
   {
      return InvertedFourBarTools.update(vertices[source.ordinal()], angle, angleDot);
   }

   @Override
   public Bound update(FourBarAngle source, double angle, double angleDot, double angleDDot)
   {
      return InvertedFourBarTools.update(vertices[source.ordinal()], angle, angleDot, angleDDot);
   }

   @Override
   public FourBarVertex getVertexA()
   {
      return A;
   }

   @Override
   public FourBarVertex getVertexB()
   {
      return B;
   }

   @Override
   public FourBarVertex getVertexC()
   {
      return C;
   }

   @Override
   public FourBarVertex getVertexD()
   {
      return D;
   }

   @Override
   public FourBarVertex getVertex(FourBarAngle angle)
   {
      return vertices[angle.ordinal()];
   }

   @Override
   public FourBarEdge getEdgeAB()
   {
      return AB;
   }

   @Override
   public FourBarEdge getEdgeBC()
   {
      return BC;
   }

   @Override
   public FourBarEdge getEdgeCD()
   {
      return CD;
   }

   @Override
   public FourBarEdge getEdgeDA()
   {
      return DA;
   }

   @Override
   public FourBarDiagonal getDiagonalAC()
   {
      return AC;
   }

   @Override
   public FourBarDiagonal getDiagonalBD()
   {
      return BD;
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
