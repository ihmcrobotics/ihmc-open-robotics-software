package us.ihmc.robotics.kinematics.fourbar;

import us.ihmc.euclid.geometry.Bound;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

/**
 * {@link FourBar} implements the math for working with a convex four bar linkage:
 * <ul>
 * <li>the full state of the linkage can be calculated from the angle, velocity, and acceleration at
 * one of its vertices.
 * <li>this implementation constrains the four bar linkage to remain convex. The limits of the
 * convex configuration set can be accessed by reading the min and max angles at any vertex.
 * </ul>
 * <p>
 * Representation of the four bar with name correspondences:
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
 */
public class FourBar implements FourBarInterface
{
   private final FourBarVertex A = new FourBarVertex("A");
   private final FourBarVertex B = new FourBarVertex("B");
   private final FourBarVertex C = new FourBarVertex("C");
   private final FourBarVertex D = new FourBarVertex("D");
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
   }

   /**
    * {@inheritDoc}
    * 
    * @throws UnsupportedOperationException if the resulting four bar linkage is not fully convex.
    */
   @Override
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

   /**
    * Sets up this four bar linkage given its 4 side lengths.
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
   public void setSideLengths(double AB, double BC, double CD, double DA)
   {
      setToNaN();
      this.AB.setLength(AB);
      this.BC.setLength(BC);
      this.CD.setLength(CD);
      this.DA.setLength(DA);
   }

   @Override
   public Bound update(FourBarAngle source, double angle)
   {
      return FourBarTools.update(vertices[source.ordinal()], angle);
   }

   @Override
   public Bound update(FourBarAngle source, double angle, double angleDot)
   {
      return FourBarTools.update(vertices[source.ordinal()], angle, angleDot);
   }

   @Override
   public Bound update(FourBarAngle source, double angle, double angleDot, double angleDDot)
   {
      return FourBarTools.update(vertices[source.ordinal()], angle, angleDot, angleDDot);
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
