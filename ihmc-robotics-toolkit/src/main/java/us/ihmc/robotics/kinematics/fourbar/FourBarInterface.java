package us.ihmc.robotics.kinematics.fourbar;

import us.ihmc.euclid.geometry.Bound;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public interface FourBarInterface
{
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
    * @throws UnsupportedOperationException if the resulting four bar linkage is not fully convex.
    */
   void setup(Point2DReadOnly A, Point2DReadOnly B, Point2DReadOnly C, Point2DReadOnly D);

   /**
    * Clears the internal data by setting the individual fields to {@link Double#NaN}.
    */
   default void setToNaN()
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
   default void setToMin(FourBarAngle source)
   {
      FourBarVertex startVertex = getVertex(source);
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

   /**
    * Sets the vertex corresponding to {@code source} to its upper limit and update the angles of the
    * other vertices.
    * 
    * @param source indicates the vertex to be moved to its upper limit.
    */
   default void setToMax(FourBarAngle source)
   {
      FourBarVertex startVertex = getVertex(source);
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
    * Calculates and update the state of this four bar linkage, i.e. computes the inner angle for every
    * vertex and the length of the diagonals.
    * 
    * @param source indicates which vertex should be set to {@code angle}.
    * @param angle  the input angle for the chosen vertex.
    * @return {@code null} if the given angle is within limits of this linkage, {@link Bound#MIN} if
    *         the angle was clamped to the lower limit, or {@link Bound#MAX} if the angled was clamped
    *         to the upper limit.
    */
   Bound update(FourBarAngle source, double angle);

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
   Bound update(FourBarAngle source, double angle, double angleDot);

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
   Bound update(FourBarAngle source, double angle, double angleDot, double angleDDot);

   /**
    * Returns the vertex A which can be used to access information such as angle, velocity,
    * acceleration, and limits at this vertex.
    * 
    * @return the vertex A.
    */
   FourBarVertex getVertexA();

   /**
    * Returns the vertex B which can be used to access information such as angle, velocity,
    * acceleration, and limits at this vertex.
    * 
    * @return the vertex B.
    */
   FourBarVertex getVertexB();

   /**
    * Returns the vertex C which can be used to access information such as angle, velocity,
    * acceleration, and limits at this vertex.
    * 
    * @return the vertex C.
    */
   FourBarVertex getVertexC();

   /**
    * Returns the vertex D which can be used to access information such as angle, velocity,
    * acceleration, and limits at this vertex.
    * 
    * @return the vertex D.
    */
   FourBarVertex getVertexD();

   /**
    * Returns the vertex corresponding to {@code angle}.
    * 
    * @param angle indicates the vertex to select.
    * @return the vertex.
    */
   FourBarVertex getVertex(FourBarAngle angle);

   /**
    * Returns the edge AB which can be used to access information such as its length.
    * 
    * @return the edge joining the vertex A to the vertex B.
    */
   FourBarEdge getEdgeAB();

   /**
    * Returns the edge BC which can be used to access information such as its length.
    * 
    * @return the edge joining the vertex B to the vertex C.
    */
   FourBarEdge getEdgeBC();

   /**
    * Returns the edge CD which can be used to access information such as its length.
    * 
    * @return the edge joining the vertex C to the vertex D.
    */
   FourBarEdge getEdgeCD();

   /**
    * Returns the edge DA which can be used to access information such as its length.
    * 
    * @return the edge joining the vertex D to the vertex A.
    */
   FourBarEdge getEdgeDA();

   /**
    * Returns the diagonal AC which can be used to access information such as its length, velocity, and
    * acceleration.
    * 
    * @return the diagonal joining the vertex A to the vertex C.
    */
   FourBarDiagonal getDiagonalAC();

   /**
    * Returns the diagonal BD which can be used to access information such as its length, velocity, and
    * acceleration.
    * 
    * @return the diagonal joining the vertex B to the vertex D.
    */
   FourBarDiagonal getDiagonalBD();

   /**
    * Quick access to get the inner angle at the vertex A, equivalent to
    * {@code this.getVertexA().getAngle()}.
    * 
    * @return the inner angle at the vertex A.
    */
   default double getAngleDAB()
   {
      return getVertexA().getAngle();
   }

   /**
    * Quick access to get the inner angle at the vertex B, equivalent to
    * {@code this.getVertexB().getAngle()}.
    * 
    * @return the inner angle at the vertex B.
    */
   default double getAngleABC()
   {
      return getVertexB().getAngle();
   }

   /**
    * Quick access to get the inner angle at the vertex C, equivalent to
    * {@code this.getVertexC().getAngle()}.
    * 
    * @return the inner angle at the vertex C.
    */
   default double getAngleBCD()
   {
      return getVertexC().getAngle();
   }

   /**
    * Quick access to get the inner angle at the vertex D, equivalent to
    * {@code this.getVertexD().getAngle()}.
    * 
    * @return the inner angle at the vertex D.
    */
   default double getAngleCDA()
   {
      return getVertexD().getAngle();
   }

   /**
    * Quick access to get the inner angular velocity at the vertex A, equivalent to
    * {@code this.getVertexA().getAngleDot()}.
    * 
    * @return the inner angular velocity at the vertex A.
    */
   default double getAngleDtDAB()
   {
      return getVertexA().getAngleDot();
   }

   /**
    * Quick access to get the inner angular velocity at the vertex B, equivalent to
    * {@code this.getVertexB().getAngleDot()}.
    * 
    * @return the inner angular velocity at the vertex B.
    */
   default double getAngleDtABC()
   {
      return getVertexB().getAngleDot();
   }

   /**
    * Quick access to get the inner angular velocity at the vertex C, equivalent to
    * {@code this.getVertexC().getAngleDot()}.
    * 
    * @return the inner angular velocity at the vertex C.
    */
   default double getAngleDtBCD()
   {
      return getVertexC().getAngleDot();
   }

   /**
    * Quick access to get the inner angular velocity at the vertex D, equivalent to
    * {@code this.getVertexD().getAngleDot()}.
    * 
    * @return the inner angular velocity at the vertex D.
    */
   default double getAngleDtCDA()
   {
      return getVertexD().getAngleDot();
   }

   /**
    * Quick access to get the inner angular acceleration at the vertex A, equivalent to
    * {@code this.getVertexA().getAngleDDot()}.
    * 
    * @return the inner angular acceleration at the vertex A.
    */
   default double getAngleDt2DAB()
   {
      return getVertexA().getAngleDDot();
   }

   /**
    * Quick access to get the inner angular acceleration at the vertex B, equivalent to
    * {@code this.getVertexB().getAngleDDot()}.
    * 
    * @return the inner angular acceleration at the vertex B.
    */
   default double getAngleDt2ABC()
   {
      return getVertexB().getAngleDDot();
   }

   /**
    * Quick access to get the inner angular acceleration at the vertex C, equivalent to
    * {@code this.getVertexC().getAngleDDot()}.
    * 
    * @return the inner angular acceleration at the vertex C.
    */
   default double getAngleDt2BCD()
   {
      return getVertexC().getAngleDDot();
   }

   /**
    * Quick access to get the inner angular acceleration at the vertex D, equivalent to
    * {@code this.getVertexD().getAngleDDot()}.
    * 
    * @return the inner angular acceleration at the vertex D.
    */
   default double getAngleDt2CDA()
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
   default double getMinDAB()
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
   default double getMaxDAB()
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
   default double getMinABC()
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
   default double getMaxABC()
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
   default double getMinBCD()
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
   default double getMaxBCD()
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
   default double getMinCDA()
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
   default double getMaxCDA()
   {
      return getVertexD().getMaxAngle();
   }

   /**
    * Quick access to get the length for the edge AB, equivalent to
    * {@code this.getEdgeAB().getMaxAngle()}.
    * 
    * @return the length separating the vertices A and B.
    */
   default double getAB()
   {
      return getEdgeAB().getLength();
   }

   /**
    * Quick access to get the length for the edge BC, equivalent to
    * {@code this.getEdgeBC().getMaxAngle()}.
    * 
    * @return the length separating the vertices B and C.
    */
   default double getBC()
   {
      return getEdgeBC().getLength();
   }

   /**
    * Quick access to get the length for the edge CD, equivalent to
    * {@code this.getEdgeCD().getMaxAngle()}.
    * 
    * @return the length separating the vertices C and D.
    */
   default double getCD()
   {
      return getEdgeCD().getLength();
   }

   /**
    * Quick access to get the length for the edge DA, equivalent to
    * {@code this.getEdgeDA().getMaxAngle()}.
    * 
    * @return the length separating the vertices D and A.
    */
   default double getDA()
   {
      return getEdgeDA().getLength();
   }
}
