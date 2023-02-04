package us.ihmc.robotics.geometry;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Line3DBasics;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.UnitVector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;
import us.ihmc.log.LogTools;

public class GeometryTools
{
   public static double angleFromXForwardToVector2D(Vector2DReadOnly vector)
   {
      return EuclidGeometryTools.angleFromFirstToSecondVector2D(1.0, 0.0, vector.getX(), vector.getY());
   }

   /**
    * Returns the minimum distance between a 2D point and an infinitely long 2D line defined by two
    * points.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code firstPointOnLine2d.distance(secondPointOnLine2d) < Epsilons.ONE_TRILLIONTH}, this
    * method returns the distance between {@code firstPointOnLine2d} and the given {@code point}.
    * </ul>
    * </p>
    * <p>
    * WARNING: the 3D arguments are projected onto the XY-plane to perform the actual computation in
    * 2D.
    * </p>
    *
    * @param point             the 3D point is projected onto the xy-plane. It's projection is used to
    *                          compute the distance from the line. Not modified.
    * @param firstPointOnLine  the projection of this 3D onto the xy-plane refers to the first point on
    *                          the 2D line. Not modified.
    * @param secondPointOnLine the projection of this 3D onto the xy-plane refers to the second point
    *                          one the 2D line. Not modified.
    * @return the minimum distance between the 2D point and the 2D line.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame.
    */
   public static double distanceFromPointToLine2d(FramePoint3D point, FramePoint3D firstPointOnLine, FramePoint3D secondPointOnLine)
   {
      point.checkReferenceFrameMatch(firstPointOnLine);
      point.checkReferenceFrameMatch(secondPointOnLine);

      double pointOnLineX = firstPointOnLine.getX();
      double pointOnLineY = firstPointOnLine.getY();
      double lineDirectionX = secondPointOnLine.getX() - firstPointOnLine.getX();
      double lineDirectionY = secondPointOnLine.getY() - firstPointOnLine.getY();
      return EuclidGeometryTools.distanceFromPoint2DToLine2D(point.getX(), point.getY(), pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY);
   }

   /**
    * Returns the minimum distance between a point and a given line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < Epsilons.ONE_TRILLIONTH}, this
    * method returns the distance between {@code lineSegmentStart} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point            3D point to compute the distance from the line segment. Not modified.
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   second endpoint of the line segment. Not modified.
    * @return the minimum distance between the 3D point and the 3D line segment.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame.
    */
   public static double distanceFromPointToLineSegment(FramePoint3D point, FramePoint3D lineSegmentStart, FramePoint3D lineSegmentEnd)
   {
      point.checkReferenceFrameMatch(lineSegmentStart);
      point.checkReferenceFrameMatch(lineSegmentEnd);
      return EuclidGeometryTools.distanceFromPoint3DToLineSegment3D(point, lineSegmentStart, lineSegmentEnd);
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left side of a given line. "Left
    * side" is determined based on order of {@code lineStart} and {@code lineEnd}. For instance, given
    * the {@code lineStart} coordinates x = 0, and y = 0, and the {@code lineEnd} coordinates x = 1, y
    * = 0, a point located on the left of this line has a negative y coordinate.
    * <p>
    * This method will return false if the point is on the line.
    * </p>
    * <p>
    * WARNING: the 3D arguments are projected onto the XY-plane to perform the actual computation in
    * 2D.
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param point             the projection onto the XY-plane of this point is used as the 2D query
    *                          point. Not modified.
    * @param firstPointOnLine  the projection onto the XY-plane of this point is used as a first point
    *                          located on the line. Not modified.
    * @param secondPointOnLine the projection onto the XY-plane of this point is used as a second point
    *                          located on the line. Not modified.
    * @return {@code true} if the 2D projection of the point is on the left side of the 2D projection
    *         of the line. {@code false} if the 2D projection of the point is on the right side or
    *         exactly on the 2D projection of the line.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame.
    */
   // FIXME this method is confusing and error prone.
   public static boolean isPointOnLeftSideOfLine(FramePoint3D point, FramePoint3D firstPointOnLine, FramePoint3D secondPointOnLine)
   {
      point.checkReferenceFrameMatch(firstPointOnLine);
      point.checkReferenceFrameMatch(secondPointOnLine);
      Point2DReadOnly lineStartPoint2d = new Point2D(firstPointOnLine.getX(), firstPointOnLine.getY());
      Point2DReadOnly lineEndPoint2d = new Point2D(secondPointOnLine.getX(), secondPointOnLine.getY());
      Point2DReadOnly checkPointPoint2d = new Point2D(point.getX(), point.getY());

      return EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(checkPointPoint2d, lineStartPoint2d, lineEndPoint2d);
   }

   /**
    * Computes the orthogonal projection of a 3D point on a given 3D plane defined by a 3D point and 3D
    * normal.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the plane normal is too small, i.e. less than
    * {@link Epsilons#ONE_TRILLIONTH}, this method fails and returns {@code false}.
    * </ul>
    * </p>
    * 
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param pointOnPlane   a point on the plane. Not modified.
    * @param planeNormal    the normal of the plane. Not modified.
    * @return the projection of the point onto the plane, or {@code null} if the method failed.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame.
    */
   public static FramePoint3D getOrthogonalProjectionOnPlane(FramePoint3D pointToProject, FramePoint3D pointOnPlane, FrameVector3D planeNormal)
   {
      FramePoint3D projection = new FramePoint3D();
      boolean success = getOrthogonalProjectionOnPlane(pointToProject, pointOnPlane, planeNormal, projection);
      if (!success)
         return null;
      else
         return projection;
   }

   /**
    * Computes the orthogonal projection of a 3D point on a given 3D plane defined by a 3D point and 3D
    * normal.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the plane normal is too small, i.e. less than
    * {@link Epsilons#ONE_TRILLIONTH}, this method fails and returns {@code false}.
    * </ul>
    * </p>
    * 
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param pointOnPlane     a point on the plane. Not modified.
    * @param planeNormal      the normal of the plane. Not modified.
    * @param projectionToPack point in which the projection of the point onto the plane is stored.
    *                         Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame, except for {@code projectionToPack}.
    */
   public static boolean getOrthogonalProjectionOnPlane(FramePoint3D pointToProject, FramePoint3D pointOnPlane, FrameVector3D planeNormal,
                                                        FramePoint3D projectionToPack)
   {
      pointToProject.checkReferenceFrameMatch(pointOnPlane);
      pointToProject.checkReferenceFrameMatch(planeNormal);
      projectionToPack.setToZero(pointToProject.getReferenceFrame());
      return EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, pointOnPlane, planeNormal, projectionToPack);
   }

   /**
    * Given two 3D infinitely long lines, this methods computes two points P &in; line1 and Q &in; lin2
    * such that the distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    * 
    * @param pointOnLine1              a 3D point on the first line. Not modified.
    * @param lineDirection1            the 3D direction of the first line. Not modified.
    * @param pointOnLine2              a 3D point on the second line. Not modified.
    * @param lineDirection2            the 3D direction of the second line. Not modified.
    * @param closestPointOnLine1ToPack the 3D coordinates of the point P are packed in this 3D point.
    *                                  Modified.
    * @param closestPointOnLine2ToPack the 3D coordinates of the point Q are packed in this 3D point.
    *                                  Modified.
    * @return the minimum distance between the two lines.
    * @throws ReferenceFrameMismatchException if the input arguments are not expressed in the same
    *                                         reference frame, except for
    *                                         {@code closestPointOnLine1ToPack} and
    *                                         {@code closestPointOnLine2ToPack}.
    */
   public static double getClosestPointsForTwoLines(FramePoint3D pointOnLine1, FrameVector3D lineDirection1, FramePoint3D pointOnLine2,
                                                    FrameVector3D lineDirection2, FramePoint3D closestPointOnLine1ToPack,
                                                    FramePoint3D closestPointOnLine2ToPack)
   {
      pointOnLine1.checkReferenceFrameMatch(lineDirection1);
      pointOnLine2.checkReferenceFrameMatch(lineDirection2);
      pointOnLine1.checkReferenceFrameMatch(pointOnLine2);

      closestPointOnLine1ToPack.setToZero(pointOnLine1.getReferenceFrame());
      closestPointOnLine2ToPack.setToZero(pointOnLine1.getReferenceFrame());

      return EuclidGeometryTools.closestPoint3DsBetweenTwoLine3Ds(pointOnLine1,
                                                                  lineDirection1,
                                                                  pointOnLine2,
                                                                  lineDirection2,
                                                                  closestPointOnLine1ToPack,
                                                                  closestPointOnLine2ToPack);
   }

   /**
    * Computes the coordinates of the intersection between a plane and an infinitely long line. In the
    * case the line is parallel to the plane, this method will return {@code null}.
    * <a href="https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection"> Useful link </a>.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param pointOnPlane  a point located on the plane. Not modified.
    * @param planeNormal   the normal of the plane. Not modified.
    * @param pointOnLine   a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @return the coordinates of the intersection, or {@code null} if the line is parallel to the
    *         plane.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same frame.
    */
   public static FramePoint3D getIntersectionBetweenLineAndPlane(FramePoint3D pointOnPlane, FrameVector3D planeNormal, FramePoint3D pointOnLine,
                                                                 FrameVector3D lineDirection)
   {
      pointOnPlane.checkReferenceFrameMatch(planeNormal);
      pointOnLine.checkReferenceFrameMatch(lineDirection);
      pointOnPlane.checkReferenceFrameMatch(pointOnLine);

      Point3DReadOnly intersection = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(pointOnPlane, planeNormal, pointOnLine, lineDirection);

      if (intersection == null)
         return null;
      else
         return new FramePoint3D(pointOnPlane.getReferenceFrame(), intersection);
   }

   /**
    * Computes the coordinates of the intersection between a plane and a finite length line segment.
    * <p>
    * This method returns null for the following cases:
    * <ul>
    * <li>the line segment is parallel to the plane,
    * <li>the line segment endpoints are on one side of the plane,
    * <li>the line segment length is equal to zero ({@code lineSegmentStart == lineSegmentEnd}),
    * <li>one of the line segment endpoints lies on the plane.
    * </ul>
    * </p>
    * Once the existence of an intersection is verified, this method calls
    * {@link #getIntersectionBetweenLineAndPlane(Point3D, Vector3D, Point3D, Vector3D)} to perform the
    * actual computation.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param pointOnPlane     a point located on the plane. Not modified.
    * @param planeNormal      the normal of the plane. Not modified.
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   second endpoint of the line segment. Not modified.
    * @return the intersection, or {@code null} if there is no intersection.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame.
    */
   public static FramePoint3D getIntersectionBetweenLineSegmentAndPlane(FramePoint3D pointOnPlane, FrameVector3D planeNormal, FramePoint3D lineSegmentStart,
                                                                        FramePoint3D lineSegmentEnd)
   {
      pointOnPlane.checkReferenceFrameMatch(planeNormal);
      lineSegmentStart.checkReferenceFrameMatch(lineSegmentEnd);
      pointOnPlane.checkReferenceFrameMatch(lineSegmentStart);

      Point3D intersection = EuclidGeometryTools.intersectionBetweenLineSegment3DAndPlane3D(pointOnPlane, planeNormal, lineSegmentStart, lineSegmentEnd);

      if (intersection == null)
         return null;
      else
         return new FramePoint3D(pointOnPlane.getReferenceFrame(), intersection);
   }

   /**
    * Test if a given line segment intersects a given plane.
    * <p>
    * Edge cases:
    * <ul>
    * <li>the line segment endpoints are equal, this method returns false whether the endpoints are on
    * the plane or not.
    * <li>one of the line segment endpoints is exactly on the plane, this method returns false.
    * </ul>
    * </p>
    * 
    * @param pointOnPlane     a point located on the plane. Not modified.
    * @param planeNormal      the normal of the plane. Not modified.
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   second endpoint of the line segment. Not modified.
    * @return {@code true} if an intersection line segment - plane exists, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame.
    */
   public static boolean isLineSegmentIntersectingPlane(FramePoint3D pointOnPlane, FrameVector3D planeNormal, FramePoint3D lineSegmentStart,
                                                        FramePoint3D lineSegmentEnd)
   {
      pointOnPlane.checkReferenceFrameMatch(planeNormal);
      lineSegmentStart.checkReferenceFrameMatch(lineSegmentEnd);
      pointOnPlane.checkReferenceFrameMatch(lineSegmentStart);
      return EuclidGeometryTools.doesLineSegment3DIntersectPlane3D(pointOnPlane, planeNormal, lineSegmentStart, lineSegmentEnd);
   }

   /**
    * Computes the minimum distance between a given point and a plane.
    * 
    * @param point        the 3D query. Not modified.
    * @param pointOnPlane a point located on the plane. Not modified.
    * @param planeNormal  the normal of the plane. Not modified.
    * @return the distance between the point and the plane.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame.
    */
   public static double distanceFromPointToPlane(FramePoint3D point, FramePoint3D pointOnPlane, FrameVector3D planeNormal)
   {
      point.checkReferenceFrameMatch(pointOnPlane);
      point.checkReferenceFrameMatch(planeNormal);

      return EuclidGeometryTools.distanceFromPoint3DToPlane3D(point, pointOnPlane, planeNormal);
   }

   /**
    * Test if two line segments intersect each other.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the two line segments are parallel but not collinear, this method returns false.
    * <li>When the two line segments are collinear, this methods returns true only if the two line
    * segments overlap or have at least one common endpoint.
    * <li>When the two line segments have a common endpoint, this method returns true.
    * </ul>
    * </p>
    * 
    * @param lineSegmentStart1 first endpoint of the first line segment. Not modified.
    * @param lineSegmentEnd1   second endpoint of the first line segment. Not modified.
    * @param lineSegmentStart1 first endpoint of the second line segment. Not modified.
    * @param lineSegmentEnd1   second endpoint of the second line segment. Not modified.
    * @return {@code true} if the two line segments intersect, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame.
    */
   public static boolean doLineSegmentsIntersect(FramePoint2D lineSegmentStart1, FramePoint2D lineSegmentEnd1, FramePoint2D lineSegmentStart2,
                                                 FramePoint2D lineSegmentEnd2)
   {
      lineSegmentStart1.checkReferenceFrameMatch(lineSegmentEnd1);
      lineSegmentStart2.checkReferenceFrameMatch(lineSegmentEnd2);
      lineSegmentStart1.checkReferenceFrameMatch(lineSegmentStart2);
      return EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
   }

   private static final ThreadLocal<Point2D> tempIntersection = new ThreadLocal<Point2D>()
   {
      @Override
      public Point2D initialValue()
      {
         return new Point2D();
      }
   };

   /**
    * Computes the intersection between two infinitely long 2D lines each defined by two 2D points.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two lines are parallel but not collinear, the two lines do not intersect.
    * <li>if the two lines are collinear, the two lines are assumed to be intersecting at
    * {@code firstPointOnLine1}.
    * </ul>
    * </p>
    * <p>
    * WARNING: the 3D arguments are projected onto the XY-plane to perform the actual computation in
    * 2D.
    * </p>
    * 
    * @param firstPointOnLine1  the x and y coordinates are used to define a first 2D point on the
    *                           first line. Not modified.
    * @param secondPointOnLine1 the x and y coordinates are used to define a second 2D point on the
    *                           first line. Not modified.
    * @param firstPointOnLine2  the x and y coordinates are used to define a first 2D point on the
    *                           second line. Not modified.
    * @param secondPointOnLine2 the x and y coordinates are used to define a second 2D point on the
    *                           second line. Not modified.
    * @param intersectionToPack the result is stored in the x and y components of this 3D point.
    *                           Modified.
    * @return {@code true} if the two lines intersect, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame, except for {@code intersectionToPack}.
    */
   // FIXME This method is too confusing and error prone.
   public static boolean getIntersectionBetweenTwoLines2d(FramePoint3DReadOnly firstPointOnLine1, FramePoint3DReadOnly secondPointOnLine1,
                                                          FramePoint3DReadOnly firstPointOnLine2, FramePoint3DReadOnly secondPointOnLine2,
                                                          FramePoint3D intersectionToPack)
   {
      firstPointOnLine1.checkReferenceFrameMatch(secondPointOnLine1);
      firstPointOnLine2.checkReferenceFrameMatch(secondPointOnLine2);
      firstPointOnLine1.checkReferenceFrameMatch(firstPointOnLine2);
      intersectionToPack.changeFrame(firstPointOnLine1.getReferenceFrame());

      double pointOnLine1x = firstPointOnLine1.getX();
      double pointOnLine1y = firstPointOnLine1.getY();
      double lineDirection1x = secondPointOnLine1.getX() - firstPointOnLine1.getX();
      double lineDirection1y = secondPointOnLine1.getY() - firstPointOnLine1.getY();
      double pointOnLine2x = firstPointOnLine2.getX();
      double pointOnLine2y = firstPointOnLine2.getY();
      double lineDirection2x = secondPointOnLine2.getX() - firstPointOnLine2.getX();
      double lineDirection2y = secondPointOnLine2.getY() - firstPointOnLine2.getY();

      boolean success = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(pointOnLine1x,
                                                                          pointOnLine1y,
                                                                          lineDirection1x,
                                                                          lineDirection1y,
                                                                          pointOnLine2x,
                                                                          pointOnLine2y,
                                                                          lineDirection2x,
                                                                          lineDirection2y,
                                                                          tempIntersection.get());

      if (!success)
         intersectionToPack.setToNaN();
      else
         intersectionToPack.set(tempIntersection.get().getX(), tempIntersection.get().getY(), intersectionToPack.getZ());
      return success;
   }

   /**
    * Computes the intersection between two infinitely long 2D lines each defined by a 2D point and a
    * 2D direction.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two lines are parallel but not collinear, the two lines do not intersect.
    * <li>if the two lines are collinear, the two lines are assumed to be intersecting at
    * {@code pointOnLine1}.
    * </ul>
    * </p>
    * <p>
    * WARNING: the 3D arguments are projected onto the XY-plane to perform the actual computation in
    * 2D.
    * </p>
    * 
    * @param pointOnLine1       the x and y coordinates are used to define a 2D point on the first
    *                           line. Not modified.
    * @param lineDirection1     the x and y components are used to define the 2D direction of the first
    *                           line. Not modified.
    * @param pointOnLine2       the x and y coordinates are used to define a 2D point on the second
    *                           line. Not modified.
    * @param lineDirection2     the x and y components are used to define the 2D direction of the
    *                           second line. Not modified.
    * @param intersectionToPack the result is stored in the x and y components of this 3D point.
    *                           Modified.
    * @return {@code true} if the two lines intersect, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame, except for {@code intersectionToPack}.
    */
   // FIXME This method is too confusing and error prone.
   public static boolean getIntersectionBetweenTwoLines2d(FramePoint3D pointOnLine1, FrameVector3D lineDirection1, FramePoint3D pointOnLine2,
                                                          FrameVector3D lineDirection2, FramePoint3D intersectionToPack)
   {
      pointOnLine1.checkReferenceFrameMatch(lineDirection1);
      pointOnLine2.checkReferenceFrameMatch(lineDirection2);
      pointOnLine1.checkReferenceFrameMatch(pointOnLine2);
      intersectionToPack.changeFrame(pointOnLine1.getReferenceFrame());

      double pointOnLine1x = pointOnLine1.getX();
      double pointOnLine1y = pointOnLine1.getY();
      double lineDirection1x = lineDirection1.getX();
      double lineDirection1y = lineDirection1.getY();
      double pointOnLine2x = pointOnLine2.getX();
      double pointOnLine2y = pointOnLine2.getY();
      double lineDirection2x = lineDirection2.getX();
      double lineDirection2y = lineDirection2.getY();

      boolean success = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(pointOnLine1x,
                                                                          pointOnLine1y,
                                                                          lineDirection1x,
                                                                          lineDirection1y,
                                                                          pointOnLine2x,
                                                                          pointOnLine2y,
                                                                          lineDirection2x,
                                                                          lineDirection2y,
                                                                          tempIntersection.get());

      if (!success)
         intersectionToPack.setToNaN();
      else
         intersectionToPack.set(tempIntersection.get().getX(), tempIntersection.get().getY(), intersectionToPack.getZ());
      return success;
   }

   /**
    * Get the Line3D intersection of two planes. Uses
    * EuclidGeometryTools#intersectionBetweenTwoPlane3Ds
    *
    * @param plane1
    * @param plane2
    * @return line of intersection, or null if planes are parallel
    */
   public static Line3D getIntersectionBetweenTwoPlanes(Plane3D plane1, Plane3D plane2)
   {
      Line3D intersection = new Line3D();
      boolean parallel = !getIntersectionBetweenTwoPlanes(plane1, plane2, intersection);
      if (parallel)
      {
         return null;
      }
      else
      {
         return intersection;
      }
   }

   /**
    * Get the Line3D intersection of two planes. Uses
    * EuclidGeometryTools#intersectionBetweenTwoPlane3Ds
    *
    * @param plane1
    * @param plane2
    * @param intersectionToPack
    * @return success (not parallel)
    */
   public static boolean getIntersectionBetweenTwoPlanes(Plane3D plane1, Plane3D plane2, Line3DBasics intersectionToPack)
   {
      return EuclidGeometryTools.intersectionBetweenTwoPlane3Ds(plane1.getPoint(),
                                                                plane1.getNormal(),
                                                                plane2.getPoint(),
                                                                plane2.getNormal(),
                                                                1e-8,
                                                                intersectionToPack.getPoint(),
                                                                intersectionToPack.getDirection());
   }

   /**
    * This methods calculates the line of intersection between two planes each defined by a point and a
    * normal. The result is packed in a 3D point located on the intersection line and the 3D direction
    * of the intersection.
    * <p>
    * <a href="http://mathworld.wolfram.com/Plane-PlaneIntersection.html"> Useful link 1</a>,
    * <a href="http://paulbourke.net/geometry/pointlineplane/"> useful link 2</a>.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the length of either the plane normal is below {@link Epsilons#ONE_TRILLIONTH}, this
    * methods fails and returns {@code false}.
    * <li>When the angle between the two planes is below {@code angleThreshold}, this methods fails and
    * returns {@code false}.
    * </ul>
    * </p>
    * 
    * @param pointOnPlane1               a point on the first plane. Not modified.
    * @param planeNormal1                the normal of the first plane. Not modified.
    * @param pointOnPlane2               a point on the second plane. Not modified.
    * @param planeNormal2                the normal of the second plane. Not modified.
    * @param angleThreshold              the minimum angle between the two planes required to do the
    *                                    calculation.
    * @param pointOnIntersectionToPack   a 3D point that is set such that it belongs to the line of
    *                                    intersection between the two planes. Modified.
    * @param intersectionDirectionToPack a 3D vector that is set to the direction of the line of
    *                                    intersection between the two planes. Modified.
    * @return {@code true} if the intersection was calculated properly, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame, except for {@code pointOnIntersectionToPack} and
    *                                         {@code intersectionDirectionToPack}.
    */
   public static boolean getIntersectionBetweenTwoPlanes(FramePoint3D pointOnPlane1, FrameVector3D planeNormal1, FramePoint3D pointOnPlane2,
                                                         FrameVector3D planeNormal2, double angleThreshold, FramePoint3D pointOnIntersectionToPack,
                                                         FrameVector3D intersectionDirectionToPack)
   {
      pointOnPlane1.checkReferenceFrameMatch(planeNormal1);
      pointOnPlane2.checkReferenceFrameMatch(planeNormal2);
      pointOnPlane1.checkReferenceFrameMatch(pointOnPlane2);
      pointOnIntersectionToPack.setToZero(pointOnPlane1.getReferenceFrame());
      intersectionDirectionToPack.setToZero(pointOnPlane1.getReferenceFrame());
      return EuclidGeometryTools.intersectionBetweenTwoPlane3Ds(pointOnPlane1,
                                                                planeNormal1,
                                                                pointOnPlane2,
                                                                planeNormal2,
                                                                angleThreshold,
                                                                pointOnIntersectionToPack,
                                                                intersectionDirectionToPack);
   }

   /**
    * Computes the normal of a plane that is defined by three points.
    * <p>
    * Edge cases:
    * <ul>
    * <li>Returns a {@code null} if the three points are on a line.
    * <li>Returns {@code null} if two or three points are equal.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param firstPointOnPlane  first point on the plane. Not modified.
    * @param secondPointOnPlane second point on the plane. Not modified.
    * @param thirdPointOnPlane  third point on the plane. Not modified.
    * @return the plane normal or {@code null} when the normal could not be determined.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame.
    */
   public static FrameVector3D getPlaneNormalGivenThreePoints(FramePoint3D firstPointOnPlane, FramePoint3D secondPointOnPlane, FramePoint3D thirdPointOnPlane)
   {
      FrameVector3D normal = new FrameVector3D();
      boolean success = getPlaneNormalGivenThreePoints(firstPointOnPlane, secondPointOnPlane, thirdPointOnPlane, normal);
      if (!success)
         return null;
      else
         return normal;
   }

   /**
    * Computes the normal of a plane that is defined by three points.
    * <p>
    * Edge cases:
    * <ul>
    * <li>Fails and returns {@code false} if the three points are on a line.
    * <li>Fails and returns {@code false} if two or three points are equal.
    * </ul>
    * </p>
    *
    * @param firstPointOnPlane  first point on the plane. Not modified.
    * @param secondPointOnPlane second point on the plane. Not modified.
    * @param thirdPointOnPlane  third point on the plane. Not modified.
    * @param normalToPack       the vector in which the result is stored. Modified.
    * @return whether the plane normal is properly determined.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame, except for {@code normalToPack}.
    */
   public static boolean getPlaneNormalGivenThreePoints(FramePoint3D firstPointOnPlane, FramePoint3D secondPointOnPlane, FramePoint3D thirdPointOnPlane,
                                                        FrameVector3D normalToPack)
   {
      firstPointOnPlane.checkReferenceFrameMatch(secondPointOnPlane);
      firstPointOnPlane.checkReferenceFrameMatch(thirdPointOnPlane);
      normalToPack.setToZero(firstPointOnPlane.getReferenceFrame());

      return EuclidGeometryTools.normal3DFromThreePoint3Ds(firstPointOnPlane, secondPointOnPlane, thirdPointOnPlane, normalToPack);
   }

   /**
    * Computes the perpendicular defined by an infinitely long 3D line (defined by two 3D points) and a
    * 3D point. To do so, the orthogonal projection of the {@code point} on line is first computed. The
    * perpendicular vector is computed as follows:
    * {@code perpendicularVector = point - orthogonalProjection}, resulting in a vector going from the
    * computed projection to the given {@code point} with a length equal to the distance between the
    * point and the line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>when the distance between the two points defining the line is below
    * {@value Epsilons#ONE_TRILLIONTH}, the method fails and returns {@code null}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param point                      the 3D point towards which the perpendicular vector should be
    *                                   pointing at. Not modified.
    * @param firstPointOnLine           a first point on the line. Not modified.
    * @param secondPointOnLine          a second point on the line. Not modified.
    * @param orthogonalProjectionToPack a 3D point in which the projection of {@code point} onto the
    *                                   line is stored. Modified. Can be {@code null}.
    * @return the vector perpendicular to the line and pointing to the {@code point}, or {@code null}
    *         when the method fails.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame, except for {@code orthogonalProjectionToPack}.
    */
   public static FrameVector3D getPerpendicularVectorFromLineToPoint(FramePoint3D point, FramePoint3D firstPointOnLine, FramePoint3D secondPointOnLine,
                                                                     FramePoint3D orthogonalProjectionToPack)
   {
      FrameVector3D perpendicularVector = new FrameVector3D();

      boolean success = getPerpendicularVectorFromLineToPoint(point, firstPointOnLine, secondPointOnLine, orthogonalProjectionToPack, perpendicularVector);
      if (!success)
         return null;
      else
         return perpendicularVector;
   }

   /**
    * Computes the perpendicular defined by an infinitely long 3D line (defined by two 3D points) and a
    * 3D point. To do so, the orthogonal projection of the {@code point} on line is first computed. The
    * perpendicular vector is computed as follows:
    * {@code perpendicularVector = point - orthogonalProjection}, resulting in a vector going from the
    * computed projection to the given {@code point} with a length equal to the distance between the
    * point and the line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>when the distance between the two points defining the line is below
    * {@value Epsilons#ONE_TRILLIONTH}, the method fails and returns {@code false}.
    * </ul>
    * </p>
    * 
    * @param point                      the 3D point towards which the perpendicular vector should be
    *                                   pointing at. Not modified.
    * @param firstPointOnLine           a first point on the line. Not modified.
    * @param secondPointOnLine          a second point on the line. Not modified.
    * @param orthogonalProjectionToPack a 3D point in which the projection of {@code point} onto the
    *                                   line is stored. Modified. Can be {@code null}.
    * @param perpendicularVectorToPack  a 3D vector in which the vector perpendicular to the line and
    *                                   pointing to the {@code point} is stored. Modified. Can NOT be
    *                                   {@code null}.
    * @return {@code true} if the method succeeded, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame, except for {@code orthogonalProjectionToPack} and
    *                                         {@code perpendicularVectorToPack}.
    */
   public static boolean getPerpendicularVectorFromLineToPoint(FramePoint3D point, FramePoint3D firstPointOnLine, FramePoint3D secondPointOnLine,
                                                               FramePoint3D orthogonalProjectionToPack, FrameVector3D perpendicularVectorToPack)
   {
      point.checkReferenceFrameMatch(firstPointOnLine);
      point.checkReferenceFrameMatch(secondPointOnLine);
      perpendicularVectorToPack.setToZero(point.getReferenceFrame());

      if (orthogonalProjectionToPack == null)
      {
         return EuclidGeometryTools.perpendicularVector3DFromLine3DToPoint3D(point, firstPointOnLine, secondPointOnLine, null, perpendicularVectorToPack);
      }
      else
      {
         orthogonalProjectionToPack.setToZero(point.getReferenceFrame());
         return EuclidGeometryTools.perpendicularVector3DFromLine3DToPoint3D(point,
                                                                             firstPointOnLine,
                                                                             secondPointOnLine,
                                                                             orthogonalProjectionToPack,
                                                                             perpendicularVectorToPack);
      }
   }

   /**
    * Computes the 2D vector perpendicular to the given 2D {@code vector} such that:
    * <ul>
    * <li>{@code vector2d.dot(perpendicularVector2d) == 0.0}.
    * <li>{@code vector2d.angle(perpendicularVector2d) == Math.PI / 2.0}.
    * </ul>
    * <p>
    * WARNING: the 3D arguments are projected onto the XY-plane to perform the actual computation in
    * 2D.
    * </p>
    * 
    * @param vector                    the vector to compute in the xy-plane the perpendicular of. Not
    *                                  modified.
    * @param perpendicularVectorToPack a vector in which the x and y components of the 2D perpendicular
    *                                  vector are stored. Modified.
    */
   // FIXME this is just bad.
   public static void getPerpendicularVector2d(FrameVector3D vector, FrameVector3D perpendicularVectorToPack)
   {
      perpendicularVectorToPack.set(-vector.getY(), vector.getX(), perpendicularVectorToPack.getZ());
   }

   /**
    * Assuming an isosceles triangle defined by three vertices A, B, and C, with |AB| == |BC|, this
    * methods computes the missing vertex B given the vertices A and C, the normal of the triangle, the
    * angle ABC that is equal to the angle at B from the the leg BA to the leg BC.
    * <a href="https://en.wikipedia.org/wiki/Isosceles_triangle"> Useful link</a>.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param baseVertexA                    the first base vertex of the isosceles triangle ABC. Not
    *                                       modified.
    * @param baseVertexC                    the second base vertex of the isosceles triangle ABC. Not
    *                                       modified.
    * @param trianglePlaneNormal            the normal of the plane on which is lying. Not modified.
    * @param ccwAngleAboutNormalAtTopVertex the angle at B from the the leg BA to the leg BC.
    * @param topVertexBToPack               the missing vertex B. Modified.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame, except for {@code topVertexBToPack}.
    */
   public static void getTopVertexOfIsoscelesTriangle(FramePoint3D baseVertexA, FramePoint3D baseVertexC, FrameVector3D trianglePlaneNormal,
                                                      double ccwAngleAboutNormalAtTopVertex, FramePoint3D topVertexBToPack)
   {
      ReferenceFrame commonFrame = baseVertexA.getReferenceFrame();
      baseVertexC.checkReferenceFrameMatch(commonFrame);
      trianglePlaneNormal.checkReferenceFrameMatch(commonFrame);
      topVertexBToPack.setToZero(commonFrame);

      EuclidGeometryTools.topVertex3DOfIsoscelesTriangle3D(baseVertexA, baseVertexC, trianglePlaneNormal, ccwAngleAboutNormalAtTopVertex, topVertexBToPack);
   }

   /**
    * Clip each component of the given tuple to the axis-aligned bounding box. Each of the bounding box
    * minimum coordinates is defined as follows: {@code minX = x1 < x2 ? x1 : x2}. Each of the bounding
    * box maximum coordinates is defined as follows: {@code maxX = x1 > x2 ? x1 : x2}.
    * 
    * @param tupleToClip the 3D tuple to clip to the bounding box. Modified.
    * @param x1          minimum/maximum x coordinate of the bounding box.
    * @param x2          minimum/maximum x coordinate of the bounding box.
    * @param y1          minimum/maximum y coordinate of the bounding box.
    * @param y2          minimum/maximum y coordinate of the bounding box.
    * @param z1          minimum/maximum z coordinate of the bounding box.
    * @param z2          minimum/maximum z coordinate of the bounding box.
    */
   // FIXME this is rather unsafe, the user should know the difference between the minimum and maximum coordinates of the bounding box.
   public static void clipToBoundingBox(Tuple3DBasics tupleToClip, double x1, double x2, double y1, double y2, double z1, double z2)
   {
      tupleToClip.setX(x1 < x2 ? MathTools.clamp(tupleToClip.getX(), x1, x2) : MathTools.clamp(tupleToClip.getX(), x2, x1));
      tupleToClip.setY(y1 < y2 ? MathTools.clamp(tupleToClip.getY(), y1, y2) : MathTools.clamp(tupleToClip.getY(), y2, y1));
      tupleToClip.setZ(z1 < z2 ? MathTools.clamp(tupleToClip.getZ(), z1, z2) : MathTools.clamp(tupleToClip.getZ(), z2, z1));
   }

   /**
    * Computes the Euclidean distance between the two given n-dimensional points {@code a} and
    * {@code b}: <br>
    * distance = Sqrt{ &sum;<sub>i=1:n</sub> { (a<sub>i</sub> - b<sub>i</sub>)<sup>2</sup>} } <br>
    *
    * @param a array containing the first point coordinates. Not modified.
    * @param b array containing the second point coordinates. Not modified.
    * @return the distance between the two points.
    * @throws IllegalArgumentException if the two vectors have different lengths.
    */
   public static double distanceBetweenPoints(double[] a, double[] b)
   {
      if (a.length != b.length)
      {
         throw new IllegalArgumentException("cannot find distance between points of different dimensions");
      }

      double distance = 0.0;
      for (int i = 0; i < a.length; i++)
      {
         double delta = a[i] - b[i];
         distance += delta * delta;
      }

      distance = Math.sqrt(distance);

      return distance;
   }

   /**
    * Attempts to normalize the given 3D vector. If the vector's length falls below
    * {@value Epsilons#ONE_TRILLIONTH}, the vector is set to (0, 0, 1).
    * 
    * @param vector the 3D vector to normalize. Modified.
    */
   public static void normalizeSafelyZUp(Vector3DBasics vector)
   {
      double distance = vector.length();

      if (distance > Epsilons.ONE_TRILLIONTH)
      {
         vector.scale(1.0 / distance);
      }
      else
      {
         vector.set(0.0, 0.0, 1.0);
      }
   }

   /**
    * Finds the smallest distance between {@code testPoint} and a list of points.
    * 
    * @param testPoint the query. Not modified.
    * @param points    the list of points to search through. Not modified.
    * @return the value of the minimum distance between the query and the point cloud.
    */
   public static double minimumDistance(FramePoint3D testPoint, List<FramePoint3D> points)
   {
      double minimumDistance = Double.POSITIVE_INFINITY;

      for (int i = 0; i < points.size(); i++)
      {
         FramePoint3D point = points.get(i);
         minimumDistance = Math.min(minimumDistance, testPoint.distanceSquared(point));
      }

      return Math.sqrt(minimumDistance);
   }

   /**
    * Change the frame and then project the result onto the XY-plane for each point in the given list
    * {@code points}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param referenceFrame the new reference frame the result will be expressed in.. Not modified.
    * @param points         the list of points to transform. Not modified.
    * @return the result of the transformation.
    */
   public static List<FramePoint2D> changeFrameAndProjectToXYPlane(ReferenceFrame referenceFrame, List<FramePoint3D> points)
   {
      List<FramePoint2D> ret = new ArrayList<>(points.size());

      for (int i = 0; i < points.size(); i++)
      {
         FramePoint3D framePoint = new FramePoint3D(points.get(i));
         framePoint.changeFrame(referenceFrame);

         ret.add(new FramePoint2D(framePoint));
      }

      return ret;
   }

   /**
    * Project each point in the given list {@code points} onto the XY-plane.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param points the list of points to transform. Not modified.
    * @return the result of the transformation.
    */
   public static List<FramePoint2D> projectOntoXYPlane(List<FramePoint3D> points)
   {
      List<FramePoint2D> ret = new ArrayList<>(points.size());

      for (int i = 0; i < points.size(); i++)
      {
         FramePoint3D point3d = points.get(i);
         ret.add(new FramePoint2D(point3d.getReferenceFrame(), point3d.getX(), point3d.getY()));
      }

      return ret;
   }

   /**
    * Assert on a component basis is the {@code tuple} is equal to (0, 0, 0) given the tolerance
    * {@code epsilon}.
    * 
    * @param tuple   the query. Not modified.
    * @param epsilon the tolerance.
    * @return {@code true} if the tuple's component are all equal to zero, {@code false} otherwise.
    */
   public static boolean isZero(Tuple3DReadOnly tuple, double epsilon)
   {
      if (!MathTools.epsilonEquals(tuple.getX(), 0.0, epsilon))
         return false;
      if (!MathTools.epsilonEquals(tuple.getY(), 0.0, epsilon))
         return false;
      if (!MathTools.epsilonEquals(tuple.getZ(), 0.0, epsilon))
         return false;
      return true;
   }

   /**
    * Assert on a component basis is the {@code tuple} is equal to (0, 0) given the tolerance
    * {@code epsilon}.
    * 
    * @param tuple   the query. Not modified.
    * @param epsilon the tolerance.
    * @return {@code true} if the tuple's component are all equal to zero, {@code false} otherwise.
    */
   public static boolean isZero(Tuple2DReadOnly tuple, double epsilon)
   {
      if (!MathTools.epsilonEquals(tuple.getX(), 0.0, epsilon))
         return false;
      if (!MathTools.epsilonEquals(tuple.getY(), 0.0, epsilon))
         return false;
      return true;
   }

   /**
    * Creates a new reference frame such that it is centered at the given {@code point} and with its
    * z-axis aligned with the given {@code zAxis} vector.
    * <p>
    * Note that the parent frame is set to the reference frame the given {@code point} and
    * {@code zAxis} are expressed in.
    * </p>
    *
    * @param frameName the name of the new frame.
    * @param point     location of the reference frame's origin. Not modified.
    * @param zAxis     orientation the reference frame's z-axis. Not modified.
    * @return the new reference frame.
    * @throws ReferenceFrameMismatchException if {@code point} and {@code zAxis} are not expressed in
    *                                         the same reference frame.
    */
   public static ReferenceFrame constructReferenceFrameFromPointAndZAxis(String frameName, FramePoint3D point, FrameVector3D zAxis)
   {
      return constructReferenceFrameFromPointAndAxis(frameName, point, Axis3D.Z, zAxis);
   }

   /**
    * Creates a new reference frame such that it is centered at the given {@code point} and with one of
    * its axes aligned with the given {@code alignAxisWithThis} vector.
    * <p>
    * Note that the parent frame is set to the reference frame the given {@code point} and
    * {@code alignAxisWithThis} are expressed in.
    * </p>
    *
    * @param frameName         the name of the new frame.
    * @param point             location of the reference frame's origin. Not modified.
    * @param axisToAlign       defines which axis of the new reference frame is to be aligned with the
    *                          given {@code alignAxisWithThis} vector.
    * @param alignAxisWithThis the vector to which the reference frame chosen axis should be aligned
    *                          with. Not modified.
    * @return the new reference frame.
    * @throws ReferenceFrameMismatchException if {@code point} and {@code alignAxisWithThis} are not
    *                                         expressed in the same reference frame.
    */
   public static ReferenceFrame constructReferenceFrameFromPointAndAxis(String frameName, FramePoint3DReadOnly point, Axis3D axisToAlign,
                                                                        FrameVector3DReadOnly alignAxisWithThis)
   {
      ReferenceFrame parentFrame = point.getReferenceFrame();
      alignAxisWithThis.checkReferenceFrameMatch(point.getReferenceFrame());

      RigidBodyTransform transformToDesired = new RigidBodyTransform();
      transformToDesired.getTranslation().set(point);
      EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(axisToAlign, alignAxisWithThis, transformToDesired.getRotation());

      return ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(frameName, parentFrame, transformToDesired);
   }

   public static void pitchAboutPoint(FramePoint3DReadOnly pointToTransform, FramePoint3DReadOnly pointToPitchAbout, double pitch, FramePoint3D resultToPack)
   {
      pointToTransform.checkReferenceFrameMatch(pointToPitchAbout);
      double tempX = pointToTransform.getX() - pointToPitchAbout.getX();
      double tempY = pointToTransform.getY() - pointToPitchAbout.getY();
      double tempZ = pointToTransform.getZ() - pointToPitchAbout.getZ();

      double cosAngle = Math.cos(pitch);
      double sinAngle = Math.sin(pitch);

      double x = cosAngle * tempX + sinAngle * tempZ;
      tempZ = -sinAngle * tempX + cosAngle * tempZ;
      tempX = x;

      resultToPack.setIncludingFrame(pointToPitchAbout);
      resultToPack.add(tempX, tempY, tempZ);
   }

   /**
    * yawAboutPoint
    *
    * @param pointToYawAbout FramePoint
    * @param yaw             double
    * @return CartesianPositionFootstep
    */
   public static void yawAboutPoint(FramePoint3DReadOnly pointToTransform, FramePoint3DReadOnly pointToYawAbout, double yaw, FramePoint3D resultToPack)
   {
      pointToTransform.checkReferenceFrameMatch(pointToYawAbout);
      double tempX = pointToTransform.getX() - pointToYawAbout.getX();
      double tempY = pointToTransform.getY() - pointToYawAbout.getY();
      double tempZ = pointToTransform.getZ() - pointToYawAbout.getZ();

      double cosAngle = Math.cos(yaw);
      double sinAngle = Math.sin(yaw);

      double x = cosAngle * tempX + -sinAngle * tempY;
      tempY = sinAngle * tempX + cosAngle * tempY;
      tempX = x;

      resultToPack.setIncludingFrame(pointToYawAbout);
      resultToPack.add(tempX, tempY, tempZ);
   }

   /**
    * yawAboutPoint
    *
    * @param pointToYawAbout FramePoint2d
    * @param yaw             double
    * @return CartesianPositionFootstep
    */
   public static void yawAboutPoint(FramePoint2DReadOnly pointToTransform, FramePoint2DReadOnly pointToYawAbout, double yaw, FramePoint2D resultToPack)
   {
      pointToTransform.checkReferenceFrameMatch(pointToYawAbout);
      double tempX = pointToTransform.getX() - pointToYawAbout.getX();
      double tempY = pointToTransform.getY() - pointToYawAbout.getY();

      double cosAngle = Math.cos(yaw);
      double sinAngle = Math.sin(yaw);

      double x = cosAngle * tempX + -sinAngle * tempY;
      tempY = sinAngle * tempX + cosAngle * tempY;
      tempX = x;

      resultToPack.setIncludingFrame(pointToYawAbout);
      resultToPack.add(tempX, tempY);
   }

   public static void rotatePoseAboutAxis(ReferenceFrame rotationAxisFrame, Axis3D rotationAxis, double angle, FramePose3D framePoseToPack)
   {
      GeometryTools.rotatePoseAboutAxis(rotationAxisFrame, rotationAxis, angle, false, false, framePoseToPack);
   }

   public static void rotatePoseAboutAxis(ReferenceFrame rotationAxisFrame, Axis3D rotationAxis, double angle, boolean lockPosition, boolean lockOrientation,
                                          FramePose3D framePoseToPack)
   {
      ReferenceFrame initialFrame = framePoseToPack.getReferenceFrame();

      framePoseToPack.changeFrame(rotationAxisFrame);

      AxisAngle axisAngle = new AxisAngle(0.0, 0.0, 0.0, angle);
      axisAngle.setElement(rotationAxis.ordinal(), 1.0);

      if (!lockPosition)
      {
         Point3D newPosition = new Point3D(framePoseToPack.getPosition());
         axisAngle.transform(newPosition);
         framePoseToPack.getPosition().set(newPosition);
      }

      if (!lockOrientation)
      {
         Quaternion newOrientation = new Quaternion(framePoseToPack.getOrientation());
         axisAngle.transform(newOrientation);
         framePoseToPack.getOrientation().set(newOrientation);
      }

      framePoseToPack.changeFrame(initialFrame);
   }

   public static void rotatePoseAboutAxis(FrameVector3D rotatationAxis, FramePoint3D rotationAxisOrigin, double angle, FramePose3D framePoseToPack)
   {
      ReferenceFrame frameWhoseZAxisIsRotationAxis = constructReferenceFrameFromPointAndZAxis("rotationAxisFrame", rotationAxisOrigin, rotatationAxis);

      rotatePoseAboutAxis(frameWhoseZAxisIsRotationAxis, Axis3D.Z, angle, framePoseToPack);
   }

   /**
    * Creates a 3D Euclid Box (a Shape) out of a 3D Bounding Box. Allocates a new Box3D.
    *
    * @param boundingBox
    * @return box
    */
   public static Box3D convertBoundingBox3DToBox3D(BoundingBox3DReadOnly boundingBox)
   {
      Point3DReadOnly minPoint = boundingBox.getMinPoint();
      Point3DReadOnly maxPoint = boundingBox.getMaxPoint();

      Point3D boxCenter = new Point3D();
      boxCenter.interpolate(minPoint, maxPoint, 0.5);
      Vector3D size = new Vector3D();
      size.sub(maxPoint, minPoint);

      return new Box3D(boxCenter, new Quaternion(), size.getX(), size.getY(), size.getZ());
   }

   /**
    * Finds the intersection of two bounding boxes defined by a bounding box Allocates a new
    * BoundingBox2D. TODO: Check, Unit test, move where BoundingBox union is, and implement for
    * BoundingBox3D.
    *
    * @param a
    * @param b
    * @return the intersection bounding box, or null if no intersection
    */
   public static BoundingBox2D getIntersectionOfTwoBoundingBoxes(BoundingBox2D a, BoundingBox2D b)
   {
      double maxX = Math.min(a.getMaxX(), b.getMaxX());
      double maxY = Math.min(a.getMaxY(), b.getMaxY());
      double minX = Math.max(a.getMinX(), b.getMinX());
      double minY = Math.max(a.getMinY(), b.getMinY());

      if ((maxX <= minX) || (maxY <= minY))
         return null;

      BoundingBox2D intersection = new BoundingBox2D(minX, minY, maxX, maxY);
      return intersection;
   }

   /**
    * Finds the intersection of two bounding boxes defined by a bounding box Allocates a new boundingBox3D.
    *
    * @param a
    * @param b
    * @return the intersection bounding box, or null if no intersection
    */
   public static BoundingBox3D getIntersectionOfTwoBoundingBoxes(BoundingBox3DReadOnly a, BoundingBox3DReadOnly b)
   {
      double maxX = Math.min(a.getMaxX(), b.getMaxX());
      double maxY = Math.min(a.getMaxY(), b.getMaxY());
      double maxZ = Math.min(a.getMaxZ(), b.getMaxZ());

      double minX = Math.max(a.getMinX(), b.getMinX());
      double minY = Math.max(a.getMinY(), b.getMinY());
      double minZ = Math.max(a.getMinZ(), b.getMinZ());

      if ((maxX <= minX) || (maxY <= minY) || (maxZ <= minZ))
         return null;

      BoundingBox3D intersection = new BoundingBox3D(minX, minY, minZ, maxX, maxY, maxZ);
      return intersection;
   }

   /**
    * Finds the union of two bounding boxes defined by a bounding box Allocates a new boundingBox3D.
    *
    * @param a
    * @param b
    * @return the union bounding box, or null if no intersection
    */
   public static BoundingBox3D getUnionOfTwoBoundingBoxes(BoundingBox3DReadOnly a, BoundingBox3DReadOnly b)
   {
      double maxX = Math.max(a.getMaxX(), b.getMaxX());
      double maxY = Math.max(a.getMaxY(), b.getMaxY());
      double maxZ = Math.max(a.getMaxZ(), b.getMaxZ());

      double minX = Math.min(a.getMinX(), b.getMinX());
      double minY = Math.min(a.getMinY(), b.getMinY());
      double minZ = Math.min(a.getMinZ(), b.getMinZ());

      if ((maxX <= minX) || (maxY <= minY) || (maxZ <= minZ))
         return null;

      BoundingBox3D union = new BoundingBox3D(minX, minY, minZ, maxX, maxY, maxZ);
      return union;
   }

   public static double computeBoundingBoxVolume3D(BoundingBox3DReadOnly boundingBox)
   {
      return Math.abs(boundingBox.getMaxX() - boundingBox.getMinX())
           * Math.abs(boundingBox.getMaxY() - boundingBox.getMinY())
           * Math.abs(boundingBox.getMaxZ() - boundingBox.getMinZ());
   }

   /**
    * Compute Intersection-over-Union (IoU) of two 3D bounding boxes.
    */
   public static double computeIntersectionOverUnionOfTwoBoundingBoxes(BoundingBox3DReadOnly a, BoundingBox3DReadOnly b)
   {
      BoundingBox3D intersection = getIntersectionOfTwoBoundingBoxes(a, b);

      if (intersection == null)
         return 0.0;

      double intersectionVolume = GeometryTools.computeBoundingBoxVolume3D(intersection);
      double volumeA = GeometryTools.computeBoundingBoxVolume3D(a);
      double volumeB = GeometryTools.computeBoundingBoxVolume3D(b);
      double unionVolume = volumeA + volumeB - intersectionVolume;

      return intersectionVolume / unionVolume;
   }

   /**
    * Compute Intersection-over-Union (IoU) of two 3D bounding boxes.
    */
   public static double computeIntersectionOverSmallerOfTwoBoundingBoxes(BoundingBox3DReadOnly a, BoundingBox3DReadOnly b)
   {
      BoundingBox3D intersection = getIntersectionOfTwoBoundingBoxes(a, b);

      if (intersection == null)
         return 0.0;

      double intersectionVolume = GeometryTools.computeBoundingBoxVolume3D(intersection);
      double volumeA = GeometryTools.computeBoundingBoxVolume3D(a);
      double volumeB = GeometryTools.computeBoundingBoxVolume3D(b);
      double smallerVolume = Math.min(volumeA, volumeB);

      return intersectionVolume / smallerVolume;
   }

   /**
    * Returns a boolean value stating whether the two given points, i.e. {@code firstQuery} and
    * {@code secondQuery}, are on the same side of a plane or separated by the plane.
    * <p>
    * The plane is defined by two points that belongs to it and a tangent, i.e. vector orthogonal to
    * the plane's normal.
    * </p>
    * 
    * @param firstQuery         the first point to test. Not modified.
    * @param secondQuery        the second point to test. Not modified.
    * @param firstPointOnPlane  the coordinates of a first point that belongs to the plane. Not
    *                           modified.
    * @param secondPointOnPlane the coordinates of a second point that belongs to the plane. Not
    *                           modified.
    * @param planeTangent       a vector that is orthogonal to the plane's normal. Not modified.
    * @return {@code true} if the two queries are on the same side of the plane,
    *         {@code false otherwise}.
    */
   public static boolean arePoint3DsSameSideOfPlane3D(Point3DReadOnly firstQuery, Point3DReadOnly secondQuery, Point3DReadOnly firstPointOnPlane,
                                                      Point3DReadOnly secondPointOnPlane, Vector3DReadOnly planeTangent)
   {
      double firstQueryX = firstQuery.getX();
      double firstQueryY = firstQuery.getY();
      double firstQueryZ = firstQuery.getZ();

      double secondQueryX = secondQuery.getX();
      double secondQueryY = secondQuery.getY();
      double secondQueryZ = secondQuery.getZ();

      double firstPointOnPlaneX = firstPointOnPlane.getX();
      double firstPointOnPlaneY = firstPointOnPlane.getY();
      double firstPointOnPlaneZ = firstPointOnPlane.getZ();

      double planeFirstTangentX = planeTangent.getX();
      double planeFirstTangentY = planeTangent.getY();
      double planeFirstTangentZ = planeTangent.getZ();

      double planeSecondTangentX = firstPointOnPlane.getX() - secondPointOnPlane.getX();
      double planeSecondTangentY = firstPointOnPlane.getY() - secondPointOnPlane.getY();
      double planeSecondTangentZ = firstPointOnPlane.getZ() - secondPointOnPlane.getZ();

      return arePoint3DsSameSideOfPlane3D(firstQueryX,
                                          firstQueryY,
                                          firstQueryZ,
                                          secondQueryX,
                                          secondQueryY,
                                          secondQueryZ,
                                          firstPointOnPlaneX,
                                          firstPointOnPlaneY,
                                          firstPointOnPlaneZ,
                                          planeFirstTangentX,
                                          planeFirstTangentY,
                                          planeFirstTangentZ,
                                          planeSecondTangentX,
                                          planeSecondTangentY,
                                          planeSecondTangentZ);
   }

   /**
    * Returns a boolean value stating whether the two given points, i.e. {@code firstQuery} and
    * {@code secondQuery}, are on the same side of a plane or separated by the plane.
    * <p>
    * The plane is defined by two points that belongs to it and a tangent, i.e. vector orthogonal to
    * the plane's normal.
    * </p>
    * 
    * @param firstQueryX         the x-coordinate of the first point to test. Not modified.
    * @param firstQueryY         the y-coordinate of the first point to test. Not modified.
    * @param firstQueryZ         the z-coordinate of the first point to test. Not modified.
    * @param secondQueryX        the x-coordinate of the second point to test. Not modified.
    * @param secondQueryY        the y-coordinate of the second point to test. Not modified.
    * @param secondQueryZ        the z-coordinate of the second point to test. Not modified.
    * @param pointOnPlaneX       the x-coordinate of a point that belongs to the plane. Not modified.
    * @param pointOnPlaneY       the y-coordinate of a point that belongs to the plane. Not modified.
    * @param pointOnPlaneZ       the z-coordinate of a point that belongs to the plane. Not modified.
    * @param planeFirstTangentX  the x-component of a first vector that is orthogonal to the plane's
    *                            normal. Not modified.
    * @param planeFirstTangentY  the y-component of a first vector that is orthogonal to the plane's
    *                            normal. Not modified.
    * @param planeFirstTangentZ  the z-component of a first vector that is orthogonal to the plane's
    *                            normal. Not modified.
    * @param planeSecondTangentX the x-component of a second vector that is orthogonal to the plane's
    *                            normal. Not modified.
    * @param planeSecondTangentY the y-component of a second vector that is orthogonal to the plane's
    *                            normal. Not modified.
    * @param planeSecondTangentZ the z-component of a second vector that is orthogonal to the plane's
    *                            normal. Not modified.
    * @return {@code true} if the two queries are on the same side of the plane,
    *         {@code false otherwise}.
    */
   public static boolean arePoint3DsSameSideOfPlane3D(double firstQueryX, double firstQueryY, double firstQueryZ, double secondQueryX, double secondQueryY,
                                                      double secondQueryZ, double pointOnPlaneX, double pointOnPlaneY, double pointOnPlaneZ,
                                                      double planeFirstTangentX, double planeFirstTangentY, double planeFirstTangentZ,
                                                      double planeSecondTangentX, double planeSecondTangentY, double planeSecondTangentZ)
   {

      double planeNormalX = planeFirstTangentY * planeSecondTangentZ - planeFirstTangentZ * planeSecondTangentY;
      double planeNormalY = planeFirstTangentZ * planeSecondTangentX - planeFirstTangentX * planeSecondTangentZ;
      double planeNormalZ = planeFirstTangentX * planeSecondTangentY - planeFirstTangentY * planeSecondTangentX;

      boolean isFirstQueryAbovePlane = EuclidGeometryTools.isPoint3DAboveOrBelowPlane3D(firstQueryX,
                                                                                        firstQueryY,
                                                                                        firstQueryZ,
                                                                                        pointOnPlaneX,
                                                                                        pointOnPlaneY,
                                                                                        pointOnPlaneZ,
                                                                                        planeNormalX,
                                                                                        planeNormalY,
                                                                                        planeNormalZ,
                                                                                        true);
      boolean isSecondQueryAbovePlane = EuclidGeometryTools.isPoint3DAboveOrBelowPlane3D(secondQueryX,
                                                                                         secondQueryY,
                                                                                         secondQueryZ,
                                                                                         pointOnPlaneX,
                                                                                         pointOnPlaneY,
                                                                                         pointOnPlaneZ,
                                                                                         planeNormalX,
                                                                                         planeNormalY,
                                                                                         planeNormalZ,
                                                                                         true);
      return isFirstQueryAbovePlane == isSecondQueryAbovePlane;
   }

   /**
    * Finds the projection of a 3D point onto a 3D plane given in general form.
    * Uses: projectedPoint = point - (normal.dot(point) + planeScalar) * (normal)
    *
    * @param plane Coefficients of the general form of plane equation (ax + by + cz + d = 0) as Vector4D
    * @param point Point to be projected onto the plane as Point3D
    * @return Projected point onto the plane as Point3D
    */
   public static Point3D projectPointOntoPlane(Vector4DReadOnly plane, Point3DReadOnly point)
   {
      UnitVector3D planeNormal = new UnitVector3D(plane.getX(), plane.getY(), plane.getZ());

      Vector3D scaledNormal = new Vector3D(planeNormal);
      scaledNormal.scale(planeNormal.dot(point) + plane.getS());

      Point3D projectedPoint = new Point3D();
      projectedPoint.sub(point, scaledNormal);

      return projectedPoint;
   }
}
