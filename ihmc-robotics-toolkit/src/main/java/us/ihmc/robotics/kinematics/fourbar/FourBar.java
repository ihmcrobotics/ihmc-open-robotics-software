package us.ihmc.robotics.kinematics.fourbar;

import us.ihmc.euclid.geometry.Bound;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

/**
 * {@link FourBar} implements the math for working with either a convex or an inverted four bar
 * linkage:
 * <ul>
 * <li>the full state of the linkage can be calculated from the angle, velocity, and acceleration at
 * one of its vertices.
 * <li>this implementation constrains the vertices of the four bar linkage to remain either convex
 * or concave. The limits of the configuration set can be accessed by reading the min and max angles
 * at any vertex.
 * </ul>
 * <p>
 * Representation of the four bar with name correspondences with a convex four bar used as example:
 *
 * <pre>
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
 * </pre>
 *
 * Angle name convention:
 * <ul>
 * <li>Inner angle at vertex A: DAB
 * <li>Inner angle at vertex B: ABC
 * <li>Inner angle at vertex C: BCD
 * <li>Inner angle at vertex D: CDA
 * </ul>
 * </p>
 * <p>
 * There are 4 different configurations for an inverted four bar linkage:
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
 * counter-clockwise and "+" for clockwise. In this example, a vertex is assumed to be convex when
 * its winding is clockwise. Note that the edge with 2 concave vertices is flagged as being flipped.
 * </p>
 * <p>
 * This implementation does not handle the following kind of configuration:
 *
 * <pre>
 *         B
 *        /|
 *       / |
 *      /  |
 *     /   C
 *    /     \
 *   /       \
 *  /         \ 
 * A-----------D
 * </pre>
 *
 * The four bar is only concave at one vertex.
 * </p>
 */
public class FourBar
{
   private final FourBarVertex A = new FourBarVertex("A", FourBarAngle.DAB);
   private final FourBarVertex B = new FourBarVertex("B", FourBarAngle.ABC);
   private final FourBarVertex C = new FourBarVertex("C", FourBarAngle.BCD);
   private final FourBarVertex D = new FourBarVertex("D", FourBarAngle.CDA);
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

      A.checkProperlySetup();
      B.checkProperlySetup();
      C.checkProperlySetup();
      D.checkProperlySetup();

      AB.checkProperlySetup();
      BC.checkProperlySetup();
      CD.checkProperlySetup();
      DA.checkProperlySetup();

      AC.checkProperlySetup();
      BD.checkProperlySetup();
   }

   /**
    * Sets up this four bar linkage for the position of its 4 vertices.
    * <p>
    * The state of this four bar linkage is cleared, i.e. angles, lengths, and their derivatives are
    * cleared and need to be update using one of the update methods.
    * </p>
    * 
    * @param A the position of the vertex A. Not modified.
    * @param B the position of the vertex B. Not modified.
    * @param C the position of the vertex C. Not modified.
    * @param D the position of the vertex D. Not modified.
    * @throws UnsupportedOperationException if the resulting four bar linkage is not supported.
    */
   public void setup(Point2DReadOnly A, Point2DReadOnly B, Point2DReadOnly C, Point2DReadOnly D)
   {
      boolean isAConvex = !EuclidGeometryTools.isPoint2DOnRightSideOfLine2D(A, D, B);
      boolean isBConvex = !EuclidGeometryTools.isPoint2DOnRightSideOfLine2D(B, A, C);
      boolean isCConvex = !EuclidGeometryTools.isPoint2DOnRightSideOfLine2D(C, B, D);
      boolean isDConvex = !EuclidGeometryTools.isPoint2DOnRightSideOfLine2D(D, C, A);

      int convexCount = isAConvex ? 1 : 0;
      convexCount += isBConvex ? 1 : 0;
      convexCount += isCConvex ? 1 : 0;
      convexCount += isDConvex ? 1 : 0;

      if (convexCount <= 1)
      { // The 4 points are ordered in a counter-clockwise scheme.
         isAConvex = !isAConvex;
         isBConvex = !isBConvex;
         isCConvex = !isCConvex;
         isDConvex = !isDConvex;
      }

      setup(A.distance(B), B.distance(C), C.distance(D), D.distance(A), isAConvex, isBConvex, isCConvex, isDConvex);
   }

   /**
    * Sets up this four bar linkage given its 4 side lengths and initialized as a convex four bar.
    * <p>
    * The state of this four bar linkage is cleared, i.e. angles, lengths, and their derivatives are
    * cleared and need to be update using one of the update methods.
    * </p>
    *
    * @param AB the length of the side joining the vertices A and B.
    * @param BC the length of the side joining the vertices B and C.
    * @param CD the length of the side joining the vertices C and D.
    * @param DA the length of the side joining the vertices D and A.
    */
   public void setup(double AB, double BC, double CD, double DA)
   {
      setup(AB, BC, CD, DA, true, true, true, true);
   }

   /**
    * Sets up this four bar linkage given its 4 side lengths.
    * <p>
    * The state of this four bar linkage is cleared, i.e. angles, lengths, and their derivatives are
    * cleared and need to be update using one of the update methods.
    * </p>
    *
    * @param AB        the length of the side joining the vertices A and B.
    * @param BC        the length of the side joining the vertices B and C.
    * @param CD        the length of the side joining the vertices C and D.
    * @param DA        the length of the side joining the vertices D and A.
    * @param isAConvex whether the four bar linkage is convex at the vertex A.
    * @param isBConvex whether the four bar linkage is convex at the vertex B.
    * @param isCConvex whether the four bar linkage is convex at the vertex C.
    * @param isDConvex whether the four bar linkage is convex at the vertex D.
    * @throws if the number of convex vertices is equal to either 1 or 3 which represents the
    *            configurations that this calculator does not support yet.
    */
   public void setup(double AB, double BC, double CD, double DA, boolean isAConvex, boolean isBConvex, boolean isCConvex, boolean isDConvex)
   {
      setToNaN();
      this.AB.setLength(AB);
      this.BC.setLength(BC);
      this.CD.setLength(CD);
      this.DA.setLength(DA);

      int convexCount = isAConvex ? 1 : 0;
      convexCount += isBConvex ? 1 : 0;
      convexCount += isCConvex ? 1 : 0;
      convexCount += isDConvex ? 1 : 0;

      if (convexCount == 1 || convexCount == 3)
         throw new UnsupportedOperationException("The quadrilateral ABCD is concave at " + (4 - convexCount)
               + "vertices, only handles either 0 or 2 concave vertices.");
      if (convexCount == 0)
         throw new IllegalArgumentException("The four bar linkage cannot be concave at all vertices");

      A.setConvex(isAConvex);
      B.setConvex(isBConvex);
      C.setConvex(isCConvex);
      D.setConvex(isDConvex);
   }

   /**
    * Clears the internal data by setting the individual fields to {@link Double#NaN}.
    */
   public void setToNaN()
   {
      getVertexA().setToNaN();
      getVertexB().setToNaN();
      getVertexC().setToNaN();
      getVertexD().setToNaN();

      getEdgeAB().setToNaN();
      getEdgeBC().setToNaN();
      getEdgeCD().setToNaN();
      getEdgeDA().setToNaN();

      getDiagonalAC().setToNaN();
      getDiagonalBD().setToNaN();
   }

   /**
    * Sets the vertex corresponding to {@code source} to its lower limit and update the angles of the
    * other vertices.
    * 
    * @param source indicates the vertex to be moved to its lower limit.
    */
   public void setToMin(FourBarAngle source)
   {
      FourBarTools.setToMinAngle(getVertex(source));
   }

   /**
    * Sets the vertex corresponding to {@code source} to its upper limit and update the angles of the
    * other vertices.
    * 
    * @param source indicates the vertex to be moved to its upper limit.
    */
   public void setToMax(FourBarAngle source)
   {
      FourBarTools.setToMaxAngle(getVertex(source));
   }

   /**
    * Tests if this four bar represents an inverted four bar as follows:
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
    * @return {@code true} if this four bar is inverted, {@code false} otherwise.
    */
   public boolean isInverted()
   {
      return FourBarTools.isFourBarInverted(this);
   }

   /**
    * Calculates and update the state of this four bar linkage, i.e. computes the inner angle for every
    * vertex and the length of the diagonals.
    * 
    * @param source indicates which vertex should be set to {@code angle}.
    * @param angle  the input angle for the chosen vertex.
    * @return {@code null} if the given angle is within limits of this linkage, {@link Bound#MIN} if
    *         the angle was clamped to the lower limit, or {@link Bound#MAX} if the angled was clamped
    *         to the upper limit.
    */
   public Bound update(FourBarAngle source, double angle)
   {
      return FourBarTools.update(vertices[source.ordinal()], angle);
   }

   /**
    * Calculates and update the state of this four bar linkage, i.e. computes the inner angle for every
    * vertex, the length of the diagonals, and calculate their first derivative.
    * 
    * @param source   indicates which vertex should be set.
    * @param angle    the input angle for the chosen vertex.
    * @param angleDot the input velocity for the chosen vertex.
    * @return {@code null} if the given angle is within limits of this linkage, {@link Bound#MIN} if
    *         the angle was clamped to the lower limit, or {@link Bound#MAX} if the angled was clamped
    *         to the upper limit.
    */
   public Bound update(FourBarAngle source, double angle, double angleDot)
   {
      return FourBarTools.update(vertices[source.ordinal()], angle, angleDot);
   }

   /**
    * Calculates and update the state of this four bar linkage, i.e. computes the inner angle for every
    * vertex, the length of the diagonals, and calculate their first and second derivatives.
    * 
    * @param source    indicates which vertex should be set.
    * @param angle     the input angle for the chosen vertex.
    * @param angleDot  the input velocity for the chosen vertex.
    * @param angleDDot the input acceleration for the chosen vertex.
    * @return {@code null} if the given angle is within limits of this linkage, {@link Bound#MIN} if
    *         the angle was clamped to the lower limit, or {@link Bound#MAX} if the angled was clamped
    *         to the upper limit.
    */
   public Bound update(FourBarAngle source, double angle, double angleDot, double angleDDot)
   {
      return FourBarTools.update(vertices[source.ordinal()], angle, angleDot, angleDDot);
   }

   /**
    * Returns the vertex A which can be used to access information such as angle, velocity,
    * acceleration, and limits at this vertex.
    * 
    * @return the vertex A.
    */
   public FourBarVertex getVertexA()
   {
      return A;
   }

   /**
    * Returns the vertex B which can be used to access information such as angle, velocity,
    * acceleration, and limits at this vertex.
    * 
    * @return the vertex B.
    */
   public FourBarVertex getVertexB()
   {
      return B;
   }

   /**
    * Returns the vertex C which can be used to access information such as angle, velocity,
    * acceleration, and limits at this vertex.
    * 
    * @return the vertex C.
    */
   public FourBarVertex getVertexC()
   {
      return C;
   }

   /**
    * Returns the vertex D which can be used to access information such as angle, velocity,
    * acceleration, and limits at this vertex.
    * 
    * @return the vertex D.
    */
   public FourBarVertex getVertexD()
   {
      return D;
   }

   /**
    * Returns the vertex corresponding to {@code angle}.
    * 
    * @param angle indicates the vertex to select.
    * @return the vertex.
    */
   public FourBarVertex getVertex(FourBarAngle angle)
   {
      return vertices[angle.ordinal()];
   }

   /**
    * Returns the edge AB which can be used to access information such as its length.
    * 
    * @return the edge joining the vertex A to the vertex B.
    */
   public FourBarEdge getEdgeAB()
   {
      return AB;
   }

   /**
    * Returns the edge BC which can be used to access information such as its length.
    * 
    * @return the edge joining the vertex B to the vertex C.
    */
   public FourBarEdge getEdgeBC()
   {
      return BC;
   }

   /**
    * Returns the edge CD which can be used to access information such as its length.
    * 
    * @return the edge joining the vertex C to the vertex D.
    */
   public FourBarEdge getEdgeCD()
   {
      return CD;
   }

   /**
    * Returns the edge DA which can be used to access information such as its length.
    * 
    * @return the edge joining the vertex D to the vertex A.
    */
   public FourBarEdge getEdgeDA()
   {
      return DA;
   }

   /**
    * Returns the diagonal AC which can be used to access information such as its length, velocity, and
    * acceleration.
    * 
    * @return the diagonal joining the vertex A to the vertex C.
    */
   public FourBarDiagonal getDiagonalAC()
   {
      return AC;
   }

   /**
    * Returns the diagonal BD which can be used to access information such as its length, velocity, and
    * acceleration.
    * 
    * @return the diagonal joining the vertex B to the vertex D.
    */
   public FourBarDiagonal getDiagonalBD()
   {
      return BD;
   }

   /**
    * Quick access to get the inner angle at the vertex A, equivalent to
    * {@code this.getVertexA().getAngle()}.
    * 
    * @return the inner angle at the vertex A.
    */
   public double getAngleDAB()
   {
      return getVertexA().getAngle();
   }

   /**
    * Quick access to get the inner angle at the vertex B, equivalent to
    * {@code this.getVertexB().getAngle()}.
    * 
    * @return the inner angle at the vertex B.
    */
   public double getAngleABC()
   {
      return getVertexB().getAngle();
   }

   /**
    * Quick access to get the inner angle at the vertex C, equivalent to
    * {@code this.getVertexC().getAngle()}.
    * 
    * @return the inner angle at the vertex C.
    */
   public double getAngleBCD()
   {
      return getVertexC().getAngle();
   }

   /**
    * Quick access to get the inner angle at the vertex D, equivalent to
    * {@code this.getVertexD().getAngle()}.
    * 
    * @return the inner angle at the vertex D.
    */
   public double getAngleCDA()
   {
      return getVertexD().getAngle();
   }

   /**
    * Quick access to get the inner angular velocity at the vertex A, equivalent to
    * {@code this.getVertexA().getAngleDot()}.
    * 
    * @return the inner angular velocity at the vertex A.
    */
   public double getAngleDtDAB()
   {
      return getVertexA().getAngleDot();
   }

   /**
    * Quick access to get the inner angular velocity at the vertex B, equivalent to
    * {@code this.getVertexB().getAngleDot()}.
    * 
    * @return the inner angular velocity at the vertex B.
    */
   public double getAngleDtABC()
   {
      return getVertexB().getAngleDot();
   }

   /**
    * Quick access to get the inner angular velocity at the vertex C, equivalent to
    * {@code this.getVertexC().getAngleDot()}.
    * 
    * @return the inner angular velocity at the vertex C.
    */
   public double getAngleDtBCD()
   {
      return getVertexC().getAngleDot();
   }

   /**
    * Quick access to get the inner angular velocity at the vertex D, equivalent to
    * {@code this.getVertexD().getAngleDot()}.
    * 
    * @return the inner angular velocity at the vertex D.
    */
   public double getAngleDtCDA()
   {
      return getVertexD().getAngleDot();
   }

   /**
    * Quick access to get the inner angular acceleration at the vertex A, equivalent to
    * {@code this.getVertexA().getAngleDDot()}.
    * 
    * @return the inner angular acceleration at the vertex A.
    */
   public double getAngleDt2DAB()
   {
      return getVertexA().getAngleDDot();
   }

   /**
    * Quick access to get the inner angular acceleration at the vertex B, equivalent to
    * {@code this.getVertexB().getAngleDDot()}.
    * 
    * @return the inner angular acceleration at the vertex B.
    */
   public double getAngleDt2ABC()
   {
      return getVertexB().getAngleDDot();
   }

   /**
    * Quick access to get the inner angular acceleration at the vertex C, equivalent to
    * {@code this.getVertexC().getAngleDDot()}.
    * 
    * @return the inner angular acceleration at the vertex C.
    */
   public double getAngleDt2BCD()
   {
      return getVertexC().getAngleDDot();
   }

   /**
    * Quick access to get the inner angular acceleration at the vertex D, equivalent to
    * {@code this.getVertexD().getAngleDDot()}.
    * 
    * @return the inner angular acceleration at the vertex D.
    */
   public double getAngleDt2CDA()
   {
      return getVertexD().getAngleDDot();
   }

   /**
    * Quick access to get the lower limit at the vertex A, equivalent to
    * {@code this.getVertexA().getMinAngle()}.
    * <p>
    * This represents first bound at which the four bar linkage is about to become concave.
    * </p>
    * 
    * @return the lower limit at the vertex A.
    */
   public double getMinDAB()
   {
      return getVertexA().getMinAngle();
   }

   /**
    * Quick access to get the upper limit at the vertex A, equivalent to
    * {@code this.getVertexA().getMaxAngle()}.
    * <p>
    * This represents second bound at which the four bar linkage is about to become concave.
    * </p>
    * 
    * @return the upper limit at the vertex A.
    */
   public double getMaxDAB()
   {
      return getVertexA().getMaxAngle();
   }

   /**
    * Quick access to get the lower limit at the vertex B, equivalent to
    * {@code this.getVertexB().getMinAngle()}.
    * <p>
    * This represents first bound at which the four bar linkage is about to become concave.
    * </p>
    * 
    * @return the lower limit at the vertex B.
    */
   public double getMinABC()
   {
      return getVertexB().getMinAngle();
   }

   /**
    * Quick access to get the upper limit at the vertex B, equivalent to
    * {@code this.getVertexB().getMaxAngle()}.
    * <p>
    * This represents second bound at which the four bar linkage is about to become concave.
    * </p>
    * 
    * @return the upper limit at the vertex B.
    */
   public double getMaxABC()
   {
      return getVertexB().getMaxAngle();
   }

   /**
    * Quick access to get the lower limit at the vertex C, equivalent to
    * {@code this.getVertexC().getMinAngle()}.
    * <p>
    * This represents first bound at which the four bar linkage is about to become concave.
    * </p>
    * 
    * @return the lower limit at the vertex C.
    */
   public double getMinBCD()
   {
      return getVertexC().getMinAngle();
   }

   /**
    * Quick access to get the upper limit at the vertex C, equivalent to
    * {@code this.getVertexC().getMaxAngle()}.
    * <p>
    * This represents second bound at which the four bar linkage is about to become concave.
    * </p>
    * 
    * @return the upper limit at the vertex C.
    */
   public double getMaxBCD()
   {
      return getVertexC().getMaxAngle();
   }

   /**
    * Quick access to get the lower limit at the vertex D, equivalent to
    * {@code this.getVertexD().getMinAngle()}.
    * <p>
    * This represents first bound at which the four bar linkage is about to become concave.
    * </p>
    * 
    * @return the lower limit at the vertex D.
    */
   public double getMinCDA()
   {
      return getVertexD().getMinAngle();
   }

   /**
    * Quick access to get the upper limit at the vertex D, equivalent to
    * {@code this.getVertexD().getMaxAngle()}.
    * <p>
    * This represents second bound at which the four bar linkage is about to become concave.
    * </p>
    * 
    * @return the upper limit at the vertex D.
    */
   public double getMaxCDA()
   {
      return getVertexD().getMaxAngle();
   }

   /**
    * Quick access to get the length for the edge AB, equivalent to
    * {@code this.getEdgeAB().getMaxAngle()}.
    * 
    * @return the length separating the vertices A and B.
    */
   public double getAB()
   {
      return getEdgeAB().getLength();
   }

   /**
    * Quick access to get the length for the edge BC, equivalent to
    * {@code this.getEdgeBC().getMaxAngle()}.
    * 
    * @return the length separating the vertices B and C.
    */
   public double getBC()
   {
      return getEdgeBC().getLength();
   }

   /**
    * Quick access to get the length for the edge CD, equivalent to
    * {@code this.getEdgeCD().getMaxAngle()}.
    * 
    * @return the length separating the vertices C and D.
    */
   public double getCD()
   {
      return getEdgeCD().getLength();
   }

   /**
    * Quick access to get the length for the edge DA, equivalent to
    * {@code this.getEdgeDA().getMaxAngle()}.
    * 
    * @return the length separating the vertices D and A.
    */
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
