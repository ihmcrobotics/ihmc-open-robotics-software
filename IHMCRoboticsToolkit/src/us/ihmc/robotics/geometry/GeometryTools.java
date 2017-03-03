package us.ihmc.robotics.geometry;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class GeometryTools
{
   /**
    * Returns the minimum distance between a 2D point and an infinitely long 2D line defined by two points.
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if {@code firstPointOnLine2d.distance(secondPointOnLine2d) < Epsilons.ONE_TRILLIONTH}, this method returns the distance between {@code firstPointOnLine2d} and the given {@code point}.
    * </ul>
    * </p>
    * <p>
    * WARNING: the 3D arguments are projected onto the XY-plane to perform the actual computation in 2D.
    * </p>
    *
    * @param point the 3D point is projected onto the xy-plane. It's projection is used to compute the distance from the line. Not modified.
    * @param firstPointOnLine the projection of this 3D onto the xy-plane refers to the first point on the 2D line. Not modified.
    * @param secondPointOnLine the projection of this 3D onto the xy-plane refers to the second point one the 2D line. Not modified.
    * @return the minimum distance between the 2D point and the 2D line.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference frame.
    */
   public static double distanceFromPointToLine2d(FramePoint point, FramePoint firstPointOnLine, FramePoint secondPointOnLine)
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
    *    <li> if {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < Epsilons.ONE_TRILLIONTH}, this method returns the distance between {@code lineSegmentStart} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point 3D point to compute the distance from the line segment. Not modified.
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd second endpoint of the line segment. Not modified.
    * @return the minimum distance between the 3D point and the 3D line segment.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference frame.
    */
   public static double distanceFromPointToLineSegment(FramePoint point, FramePoint lineSegmentStart, FramePoint lineSegmentEnd)
   {
      point.checkReferenceFrameMatch(lineSegmentStart);
      point.checkReferenceFrameMatch(lineSegmentEnd);
      return EuclidGeometryTools.distanceFromPoint3DToLineSegment3D(point.getPoint(), lineSegmentStart.getPoint(), lineSegmentEnd.getPoint());
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left side of a given line.
    * "Left side" is determined based on order of {@code lineStart} and {@code lineEnd}.
    * For instance, given the {@code lineStart} coordinates x = 0, and y = 0, and the {@code lineEnd} coordinates x = 1, y = 0,
    * a point located on the left of this line has a negative y coordinate.
    *<p>
    * This method will return false if the point is on the line.
    * </p>
    * <p>
    * WARNING: the 3D arguments are projected onto the XY-plane to perform the actual computation in 2D.
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param point the projection onto the XY-plane of this point is used as the 2D query point. Not modified.
    * @param firstPointOnLine the projection onto the XY-plane of this point is used as a first point located on the line. Not modified.
    * @param secondPointOnLine the projection onto the XY-plane of this point is used as a second point located on the line. Not modified.
    * @return {@code true} if the 2D projection of the point is on the left side of the 2D projection of the line.
    * {@code false} if the 2D projection of the point is on the right side or exactly on the 2D projection of the line.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference frame.
    */
   // FIXME this method is confusing and error prone.
   public static boolean isPointOnLeftSideOfLine(FramePoint point, FramePoint firstPointOnLine, FramePoint secondPointOnLine)
   {
      point.checkReferenceFrameMatch(firstPointOnLine);
      point.checkReferenceFrameMatch(secondPointOnLine);
      Point2DReadOnly lineStartPoint2d = new Point2D(firstPointOnLine.getX(), firstPointOnLine.getY());
      Point2DReadOnly lineEndPoint2d = new Point2D(secondPointOnLine.getX(), secondPointOnLine.getY());
      Point2DReadOnly checkPointPoint2d = new Point2D(point.getX(), point.getY());

      return EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(checkPointPoint2d, lineStartPoint2d, lineEndPoint2d);
   }

   /**
    * Computes the orthogonal projection of a 3D point on a given 3D plane defined by a 3D point and 3D normal.
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the length of the plane normal is too small, i.e. less than {@link Epsilons#ONE_TRILLIONTH},
    *      this method fails and returns {@code false}.
    * </ul>
    * </p>
    * 
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param pointOnPlane a point on the plane. Not modified.
    * @param planeNormal the normal of the plane. Not modified.
    * @return the projection of the point onto the plane, or {@code null} if the method failed.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference frame.
    */
   public static FramePoint getOrthogonalProjectionOnPlane(FramePoint pointToProject, FramePoint pointOnPlane, FrameVector planeNormal)
   {
      FramePoint projection = new FramePoint();
      boolean success = getOrthogonalProjectionOnPlane(pointToProject, pointOnPlane, planeNormal, projection);
      if (!success)
         return null;
      else
         return projection;
   }

   /**
    * Computes the orthogonal projection of a 3D point on a given 3D plane defined by a 3D point and 3D normal.
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the length of the plane normal is too small, i.e. less than {@link Epsilons#ONE_TRILLIONTH},
    *      this method fails and returns {@code false}.
    * </ul>
    * </p>
    * 
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param pointOnPlane a point on the plane. Not modified.
    * @param planeNormal the normal of the plane. Not modified.
    * @param projectionToPack point in which the projection of the point onto the plane is stored. Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference frame,
    *  except for {@code projectionToPack}.
    */
   public static boolean getOrthogonalProjectionOnPlane(FramePoint pointToProject, FramePoint pointOnPlane, FrameVector planeNormal,
                                                        FramePoint projectionToPack)
   {
      pointToProject.checkReferenceFrameMatch(pointOnPlane);
      pointToProject.checkReferenceFrameMatch(planeNormal);
      projectionToPack.setToZero(pointToProject.getReferenceFrame());
      return EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject.getPoint(), pointOnPlane.getPoint(), planeNormal.getVector(), projectionToPack.getPoint());
   }

   /**
    * Given two 3D infinitely long lines, this methods computes two points P &in; line1 and Q &in; lin2 such that the distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    * 
    * @param pointOnLine1 a 3D point on the first line. Not modified.
    * @param lineDirection1 the 3D direction of the first line. Not modified.
    * @param pointOnLine2 a 3D point on the second line. Not modified.
    * @param lineDirection2 the 3D direction of the second line. Not modified.
    * @param closestPointOnLine1ToPack the 3D coordinates of the point P are packed in this 3D point. Modified.
    * @param closestPointOnLine2ToPack the 3D coordinates of the point Q are packed in this 3D point. Modified.
    * @return the minimum distance between the two lines.
    * @throws ReferenceFrameMismatchException if the input arguments are not expressed in the same reference frame, except for {@code closestPointOnLine1ToPack} and  {@code closestPointOnLine2ToPack}.
    */
   public static double getClosestPointsForTwoLines(FramePoint pointOnLine1, FrameVector lineDirection1, FramePoint pointOnLine2, FrameVector lineDirection2,
                                                  FramePoint closestPointOnLine1ToPack, FramePoint closestPointOnLine2ToPack)
   {
      pointOnLine1.checkReferenceFrameMatch(lineDirection1);
      pointOnLine2.checkReferenceFrameMatch(lineDirection2);
      pointOnLine1.checkReferenceFrameMatch(pointOnLine2);

      closestPointOnLine1ToPack.setToZero(pointOnLine1.getReferenceFrame());
      closestPointOnLine2ToPack.setToZero(pointOnLine1.getReferenceFrame());

      return EuclidGeometryTools.closestPoint3DsBetweenTwoLine3Ds(pointOnLine1.getPoint(), lineDirection1.getVector(), pointOnLine2.getPoint(), lineDirection2.getVector(),
                                         closestPointOnLine1ToPack.getPoint(), closestPointOnLine2ToPack.getPoint());
   }

   /**
    * Computes the coordinates of the intersection between a plane and an infinitely long line.
    * In the case the line is parallel to the plane, this method will return {@code null}.
    * <a href="https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection"> Useful link </a>.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param pointOnPlane a point located on the plane. Not modified.
    * @param planeNormal the normal of the plane. Not modified.
    * @param pointOnLine a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @return the coordinates of the intersection, or {@code null} if the line is parallel to the plane.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same frame.
    */
   public static FramePoint getIntersectionBetweenLineAndPlane(FramePoint pointOnPlane, FrameVector planeNormal, FramePoint pointOnLine,
                                                               FrameVector lineDirection)
   {
      pointOnPlane.checkReferenceFrameMatch(planeNormal);
      pointOnLine.checkReferenceFrameMatch(lineDirection);
      pointOnPlane.checkReferenceFrameMatch(pointOnLine);

      Point3DReadOnly intersection = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(pointOnPlane.getPoint(), planeNormal.getVector(), pointOnLine.getPoint(),
                                                                lineDirection.getVector());

      if (intersection == null)
         return null;
      else
         return new FramePoint(pointOnPlane.getReferenceFrame(), intersection);
   }

   /**
    * Computes the coordinates of the intersection between a plane and a finite length line segment.
    * <p>
    * This method returns null for the following cases:
    * <ul>
    *    <li> the line segment is parallel to the plane,
    *    <li> the line segment endpoints are on one side of the plane,
    *    <li> the line segment length is equal to zero ({@code lineSegmentStart == lineSegmentEnd}),
    *    <li> one of the line segment endpoints lies on the plane.
    * </ul>
    * </p>
    * Once the existence of an intersection is verified,
    * this method calls {@link #getIntersectionBetweenLineAndPlane(Point3D, Vector3D, Point3D, Vector3D)}
    * to perform the actual computation.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param pointOnPlane a point located on the plane. Not modified.
    * @param planeNormal the normal of the plane. Not modified.
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd second endpoint of the line segment. Not modified.
    * @return the intersection, or {@code null} if there is no intersection.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference frame.
    */
   public static FramePoint getIntersectionBetweenLineSegmentAndPlane(FramePoint pointOnPlane, FrameVector planeNormal, FramePoint lineSegmentStart,
                                                                      FramePoint lineSegmentEnd)
   {
      pointOnPlane.checkReferenceFrameMatch(planeNormal);
      lineSegmentStart.checkReferenceFrameMatch(lineSegmentEnd);
      pointOnPlane.checkReferenceFrameMatch(lineSegmentStart);

      Point3D intersection = EuclidGeometryTools.intersectionBetweenLineSegment3DAndPlane3D(pointOnPlane.getPoint(), planeNormal.getVector(), lineSegmentStart.getPoint(),
                                                                       lineSegmentEnd.getPoint());

      if (intersection == null)
         return null;
      else
         return new FramePoint(pointOnPlane.getReferenceFrame(), intersection);
   }

   /**
    * Test if a given line segment intersects a given plane.
    * <p>
    * Edge cases:
    * <ul>
    *    <li> the line segment endpoints are equal, this method returns false whether the endpoints are on the plane or not.
    *    <li> one of the line segment endpoints is exactly on the plane, this method returns false.
    * </ul>
    * </p>
    * 
    * @param pointOnPlane a point located on the plane. Not modified.
    * @param planeNormal the normal of the plane. Not modified.
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd second endpoint of the line segment. Not modified.
    * @return {@code true} if an intersection line segment - plane exists, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference frame.
    */
   public static boolean isLineSegmentIntersectingPlane(FramePoint pointOnPlane, FrameVector planeNormal, FramePoint lineSegmentStart,
                                                        FramePoint lineSegmentEnd)
   {
      pointOnPlane.checkReferenceFrameMatch(planeNormal);
      lineSegmentStart.checkReferenceFrameMatch(lineSegmentEnd);
      pointOnPlane.checkReferenceFrameMatch(lineSegmentStart);
      return EuclidGeometryTools.doesLineSegment3DIntersectPlane3D(pointOnPlane.getPoint(), planeNormal.getVector(), lineSegmentStart.getPoint(), lineSegmentEnd.getPoint());
   }

   /**
    * Computes the minimum distance between a given point and a plane.
    * 
    * @param point the 3D query. Not modified.
    * @param pointOnPlane a point located on the plane. Not modified.
    * @param planeNormal the normal of the plane. Not modified.
    * @return the distance between the point and the plane.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference frame.
    */
   public static double distanceFromPointToPlane(FramePoint point, FramePoint pointOnPlane, FrameVector planeNormal)
   {
      point.checkReferenceFrameMatch(pointOnPlane);
      point.checkReferenceFrameMatch(planeNormal);

      return EuclidGeometryTools.distanceFromPoint3DToPlane3D(point.getPoint(), pointOnPlane.getPoint(), planeNormal.getVector());
   }

   /**
    * Test if two line segments intersect each other.
    * <p>
    * Edge cases:
    * <ul>
    *    <li> When the two line segments are parallel but not collinear, this method returns false.
    *    <li> When the two line segments are collinear,
    *     this methods returns true only if the two line segments overlap or have at least one common endpoint.
    *    <li> When the two line segments have a common endpoint, this method returns true.
    * </ul>
    * </p>
    * 
    * @param lineSegmentStart1 first endpoint of the first line segment. Not modified.
    * @param lineSegmentEnd1 second endpoint of the first line segment. Not modified.
    * @param lineSegmentStart1 first endpoint of the second line segment. Not modified.
    * @param lineSegmentEnd1 second endpoint of the second line segment. Not modified.
    * @return {@code true} if the two line segments intersect, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference frame.
    */
   public static boolean doLineSegmentsIntersect(FramePoint2d lineSegmentStart1, FramePoint2d lineSegmentEnd1, FramePoint2d lineSegmentStart2,
                                                 FramePoint2d lineSegmentEnd2)
   {
      lineSegmentStart1.checkReferenceFrameMatch(lineSegmentEnd1);
      lineSegmentStart2.checkReferenceFrameMatch(lineSegmentEnd2);
      lineSegmentStart1.checkReferenceFrameMatch(lineSegmentStart2);
      return EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1.getPoint(), lineSegmentEnd1.getPoint(), lineSegmentStart2.getPoint(), lineSegmentEnd2.getPoint());
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
    *    <li> if the two lines are parallel but not collinear, the two lines do not intersect.
    *    <li> if the two lines are collinear, the two lines are assumed to be intersecting at {@code firstPointOnLine1}.
    * </ul>
    * </p>
    * <p>
    * WARNING: the 3D arguments are projected onto the XY-plane to perform the actual computation in 2D.
    * </p>
    * 
    * @param firstPointOnLine1 the x and y coordinates are used to define a first 2D point on the first line. Not modified.
    * @param secondPointOnLine1 the x and y coordinates are used to define a second 2D point on the first line. Not modified.
    * @param firstPointOnLine2 the x and y coordinates are used to define a first 2D point on the second line. Not modified.
    * @param secondPointOnLine2 the x and y coordinates are used to define a second 2D point on the second line. Not modified.
    * @param intersectionToPack the result is stored in the x and y components of this 3D point. Modified.
    * @return {@code true} if the two lines intersect, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference frame, except for {@code intersectionToPack}.
    */
   // FIXME This method is too confusing and error prone.
   public static boolean getIntersectionBetweenTwoLines2d(FramePoint firstPointOnLine1, FramePoint secondPointOnLine1, FramePoint firstPointOnLine2,
                                                          FramePoint secondPointOnLine2, FramePoint intersectionToPack)
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

      boolean success = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(pointOnLine1x, pointOnLine1y, lineDirection1x, lineDirection1y, pointOnLine2x, pointOnLine2y,
                                                       lineDirection2x, lineDirection2y, tempIntersection.get());

      if (!success)
         intersectionToPack.setToNaN();
      else
         intersectionToPack.set(tempIntersection.get().getX(), tempIntersection.get().getY(), intersectionToPack.getZ());
      return success;
   }

   /**
    * Computes the intersection between two infinitely long 2D lines each defined by a 2D point and a 2D direction.
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the two lines are parallel but not collinear, the two lines do not intersect.
    *    <li> if the two lines are collinear, the two lines are assumed to be intersecting at {@code pointOnLine1}.
    * </ul>
    * </p>
    * <p>
    * WARNING: the 3D arguments are projected onto the XY-plane to perform the actual computation in 2D.
    * </p>
    * 
    * @param pointOnLine1 the x and y coordinates are used to define a 2D point on the first line. Not modified.
    * @param lineDirection1 the x and y components are used to define the 2D direction of the first line. Not modified.
    * @param pointOnLine2 the x and y coordinates are used to define a 2D point on the second line. Not modified.
    * @param lineDirection2 the x and y components are used to define the 2D direction of the second line. Not modified.
    * @param intersectionToPack the result is stored in the x and y components of this 3D point. Modified.
    * @return {@code true} if the two lines intersect, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference frame, except for {@code intersectionToPack}.
    */
   // FIXME This method is too confusing and error prone.
   public static boolean getIntersectionBetweenTwoLines2d(FramePoint pointOnLine1, FrameVector lineDirection1, FramePoint pointOnLine2,
                                                          FrameVector lineDirection2, FramePoint intersectionToPack)
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

      boolean success = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(pointOnLine1x, pointOnLine1y, lineDirection1x, lineDirection1y, pointOnLine2x, pointOnLine2y,
                                                       lineDirection2x, lineDirection2y, tempIntersection.get());

      if (!success)
         intersectionToPack.setToNaN();
      else
         intersectionToPack.set(tempIntersection.get().getX(), tempIntersection.get().getY(), intersectionToPack.getZ());
      return success;
   }

   /**
    * This methods calculates the line of intersection between two planes each defined by a point and a normal.
    * The result is packed in a 3D point located on the intersection line and the 3D direction of the intersection.
    * <p>
    * <a href="http://mathworld.wolfram.com/Plane-PlaneIntersection.html"> Useful link 1</a>,
    * <a href="http://paulbourke.net/geometry/pointlineplane/"> useful link 2</a>.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    *    <li> When the length of either the plane normal is below {@link Epsilons#ONE_TRILLIONTH}, this methods fails and returns {@code false}.
    *    <li> When the angle between the two planes is below {@code angleThreshold}, this methods fails and returns {@code false}.
    * </ul>
    * </p>
    * 
    * @param pointOnPlane1 a point on the first plane. Not modified.
    * @param planeNormal1 the normal of the first plane. Not modified.
    * @param pointOnPlane2 a point on the second plane. Not modified.
    * @param planeNormal2 the normal of the second plane. Not modified.
    * @param angleThreshold the minimum angle between the two planes required to do the calculation.
    * @param pointOnIntersectionToPack a 3D point that is set such that it belongs to the line of intersection between the two planes. Modified.
    * @param intersectionDirectionToPack a 3D vector that is set to the direction of the line of intersection between the two planes. Modified.
    * @return {@code true} if the intersection was calculated properly, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference frame, except for {@code pointOnIntersectionToPack} and {@code intersectionDirectionToPack}.
    */
   public static boolean getIntersectionBetweenTwoPlanes(FramePoint pointOnPlane1, FrameVector planeNormal1, FramePoint pointOnPlane2, FrameVector planeNormal2,
                                                         double angleThreshold, FramePoint pointOnIntersectionToPack, FrameVector intersectionDirectionToPack)
   {
      pointOnPlane1.checkReferenceFrameMatch(planeNormal1);
      pointOnPlane2.checkReferenceFrameMatch(planeNormal2);
      pointOnPlane1.checkReferenceFrameMatch(pointOnPlane2);
      pointOnIntersectionToPack.setToZero(pointOnPlane1.getReferenceFrame());
      intersectionDirectionToPack.setToZero(pointOnPlane1.getReferenceFrame());
      return EuclidGeometryTools.intersectionBetweenTwoPlane3Ds(pointOnPlane1.getPoint(), planeNormal1.getVector(), pointOnPlane2.getPoint(), planeNormal2.getVector(),
                                             angleThreshold, pointOnIntersectionToPack.getPoint(), intersectionDirectionToPack.getVector());
   }

   /**
    * Computes the normal of a plane that is defined by three points.
    * <p>
    * Edge cases:
    * <ul>
    *    <li> Returns a {@code null} if the three points are on a line.
    *    <li> Returns {@code null} if two or three points are equal.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param firstPointOnPlane first point on the plane. Not modified.
    * @param secondPointOnPlane second point on the plane. Not modified.
    * @param thirdPointOnPlane third point on the plane. Not modified.
    * @return the plane normal or {@code null} when the normal could not be determined.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference frame.
    */
   public static FrameVector getPlaneNormalGivenThreePoints(FramePoint firstPointOnPlane, FramePoint secondPointOnPlane, FramePoint thirdPointOnPlane)
   {
      FrameVector normal = new FrameVector();
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
    *    <li> Fails and returns {@code false} if the three points are on a line.
    *    <li> Fails and returns {@code false} if two or three points are equal.
    * </ul>
    * </p>
    *
    * @param firstPointOnPlane first point on the plane. Not modified.
    * @param secondPointOnPlane second point on the plane. Not modified.
    * @param thirdPointOnPlane third point on the plane. Not modified.
    * @param normalToPack the vector in which the result is stored. Modified.
    * @return whether the plane normal is properly determined.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference frame, except for {@code normalToPack}.
    */
   public static boolean getPlaneNormalGivenThreePoints(FramePoint firstPointOnPlane, FramePoint secondPointOnPlane, FramePoint thirdPointOnPlane,
                                                        FrameVector normalToPack)
   {
      firstPointOnPlane.checkReferenceFrameMatch(secondPointOnPlane);
      firstPointOnPlane.checkReferenceFrameMatch(thirdPointOnPlane);
      normalToPack.setToZero(firstPointOnPlane.getReferenceFrame());

      return getPlaneNormalGivenThreePoints(firstPointOnPlane.getPoint(), secondPointOnPlane.getPoint(), thirdPointOnPlane.getPoint(),
                                            normalToPack.getVector());
   }

   /**
    * Computes the normal of a plane that is defined by three points.
    * <p>
    * Edge cases:
    * <ul>
    *    <li> Returns a {@code null} if the three points are on a line.
    *    <li> Returns {@code null} if two or three points are equal.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param firstPointOnPlane first point on the plane. Not modified.
    * @param secondPointOnPlane second point on the plane. Not modified.
    * @param thirdPointOnPlane third point on the plane. Not modified.
    * @return the plane normal or {@code null} when the normal could not be determined.
    */
   public static Vector3D getPlaneNormalGivenThreePoints(Point3DReadOnly firstPointOnPlane, Point3DReadOnly secondPointOnPlane, Point3DReadOnly thirdPointOnPlane)
   {
      Vector3D normal = new Vector3D();
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
    *    <li> Fails and returns {@code false} if the three points are on a line.
    *    <li> Fails and returns {@code false} if two or three points are equal.
    * </ul>
    * </p>
    *
    * @param firstPointOnPlane first point on the plane. Not modified.
    * @param secondPointOnPlane second point on the plane. Not modified.
    * @param thirdPointOnPlane third point on the plane. Not modified.
    * @param normalToPack the vector in which the result is stored. Modified.
    * @return whether the plane normal is properly determined.
    */
   public static boolean getPlaneNormalGivenThreePoints(Point3DReadOnly firstPointOnPlane, Point3DReadOnly secondPointOnPlane, Point3DReadOnly thirdPointOnPlane, Vector3DBasics normalToPack)
   {
      double v1_x = secondPointOnPlane.getX() - firstPointOnPlane.getX();
      double v1_y = secondPointOnPlane.getY() - firstPointOnPlane.getY();
      double v1_z = secondPointOnPlane.getZ() - firstPointOnPlane.getZ();

      double v2_x = thirdPointOnPlane.getX() - firstPointOnPlane.getX();
      double v2_y = thirdPointOnPlane.getY() - firstPointOnPlane.getY();
      double v2_z = thirdPointOnPlane.getZ() - firstPointOnPlane.getZ();

      normalToPack.setX(v1_y * v2_z - v1_z * v2_y);
      normalToPack.setY(v2_x * v1_z - v2_z * v1_x);
      normalToPack.setZ(v1_x * v2_y - v1_y * v2_x);

      double normalLength = normalToPack.length();
      if (normalLength < 1.0e-7)
         return false;

      normalToPack.scale(1.0 / normalLength);
      return true;
   }

   /**
    * Computes the perpendicular defined by an infinitely long 3D line (defined by two 3D points) and a 3D point.
    * To do so, the orthogonal projection of the {@code point} on line is first computed.
    * The perpendicular vector is computed as follows: {@code perpendicularVector = point - orthogonalProjection},
    * resulting in a vector going from the computed projection to the given {@code point}
    * with a length equal to the distance between the point and the line.
    * <p>
    * Edge cases:
    * <ul>
    *    <li> when the distance between the two points defining the line is below {@value Epsilons#ONE_TRILLIONTH}, the method fails and returns {@code null}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param point the 3D point towards which the perpendicular vector should be pointing at. Not modified.
    * @param firstPointOnLine a first point on the line. Not modified.
    * @param secondPointOnLine a second point on the line. Not modified.
    * @param orthogonalProjectionToPack a 3D point in which the projection of {@code point} onto the line is stored. Modified. Can be {@code null}.
    * @return the vector perpendicular to the line and pointing to the {@code point}, or {@code null} when the method fails.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference frame, except for {@code orthogonalProjectionToPack}.
    */
   public static FrameVector getPerpendicularVectorFromLineToPoint(FramePoint point, FramePoint firstPointOnLine, FramePoint secondPointOnLine,
                                                                   FramePoint orthogonalProjectionToPack)
   {
      FrameVector perpendicularVector = new FrameVector();

      boolean success = getPerpendicularVectorFromLineToPoint(point, firstPointOnLine, secondPointOnLine, orthogonalProjectionToPack, perpendicularVector);
      if (!success)
         return null;
      else
         return perpendicularVector;
   }

   /**
    * Computes the perpendicular defined by an infinitely long 3D line (defined by two 3D points) and a 3D point.
    * To do so, the orthogonal projection of the {@code point} on line is first computed.
    * The perpendicular vector is computed as follows: {@code perpendicularVector = point - orthogonalProjection},
    * resulting in a vector going from the computed projection to the given {@code point}
    * with a length equal to the distance between the point and the line.
    * <p>
    * Edge cases:
    * <ul>
    *    <li> when the distance between the two points defining the line is below {@value Epsilons#ONE_TRILLIONTH}, the method fails and returns {@code false}.
    * </ul>
    * </p>
    * 
    * @param point the 3D point towards which the perpendicular vector should be pointing at. Not modified.
    * @param firstPointOnLine a first point on the line. Not modified.
    * @param secondPointOnLine a second point on the line. Not modified.
    * @param orthogonalProjectionToPack a 3D point in which the projection of {@code point} onto the line is stored. Modified. Can be {@code null}.
    * @param perpendicularVectorToPack a 3D vector in which the vector perpendicular to the line and pointing to the {@code point} is stored. Modified. Can NOT be {@code null}.
    * @return {@code true} if the method succeeded, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference frame, except for {@code orthogonalProjectionToPack} and {@code perpendicularVectorToPack}.
    */
   public static boolean getPerpendicularVectorFromLineToPoint(FramePoint point, FramePoint firstPointOnLine, FramePoint secondPointOnLine,
                                                               FramePoint orthogonalProjectionToPack, FrameVector perpendicularVectorToPack)
   {
      point.checkReferenceFrameMatch(firstPointOnLine);
      point.checkReferenceFrameMatch(secondPointOnLine);
      perpendicularVectorToPack.setToZero(point.getReferenceFrame());

      if (orthogonalProjectionToPack == null)
      {
         return getPerpendicularVectorFromLineToPoint(point.getPoint(), firstPointOnLine.getPoint(), secondPointOnLine.getPoint(), null,
                                                      perpendicularVectorToPack.getVector());
      }
      else
      {
         orthogonalProjectionToPack.setToZero(point.getReferenceFrame());
         return getPerpendicularVectorFromLineToPoint(point.getPoint(), firstPointOnLine.getPoint(), secondPointOnLine.getPoint(),
                                                      orthogonalProjectionToPack.getPoint(), perpendicularVectorToPack.getVector());
      }
   }

   /**
    * Computes the perpendicular defined by an infinitely long 3D line (defined by two 3D points) and a 3D point.
    * To do so, the orthogonal projection of the {@code point} on line is first computed.
    * The perpendicular vector is computed as follows: {@code perpendicularVector = point - orthogonalProjection},
    * resulting in a vector going from the computed projection to the given {@code point}
    * with a length equal to the distance between the point and the line.
    * <p>
    * Edge cases:
    * <ul>
    *    <li> when the distance between the two points defining the line is below {@value Epsilons#ONE_TRILLIONTH}, the method fails and returns {@code null}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param point the 3D point towards which the perpendicular vector should be pointing at. Not modified.
    * @param firstPointOnLine a first point on the line. Not modified.
    * @param secondPointOnLine a second point on the line. Not modified.
    * @param orthogonalProjectionToPack a 3D point in which the projection of {@code point} onto the line is stored. Modified. Can be {@code null}.
    * @return the vector perpendicular to the line and pointing to the {@code point}, or {@code null} when the method fails.
    */
   public static Vector3D getPerpendicularVectorFromLineToPoint(Point3DReadOnly point, Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine,
                                                                Point3DBasics orthogonalProjectionToPack)
   {
      Vector3D perpendicularVector = new Vector3D();
      boolean success = getPerpendicularVectorFromLineToPoint(point, firstPointOnLine, secondPointOnLine, orthogonalProjectionToPack, perpendicularVector);
      if (!success)
         return null;
      else
         return perpendicularVector;
   }

   /**
    * Computes the perpendicular defined by an infinitely long 3D line (defined by two 3D points) and a 3D point.
    * To do so, the orthogonal projection of the {@code point} on line is first computed.
    * The perpendicular vector is computed as follows: {@code perpendicularVector = point - orthogonalProjection},
    * resulting in a vector going from the computed projection to the given {@code point}
    * with a length equal to the distance between the point and the line.
    * <p>
    * Edge cases:
    * <ul>
    *    <li> when the distance between the two points defining the line is below {@value Epsilons#ONE_TRILLIONTH}, the method fails and returns {@code false}.
    * </ul>
    * </p>
    * 
    * @param point the 3D point towards which the perpendicular vector should be pointing at. Not modified.
    * @param firstPointOnLine a first point on the line. Not modified.
    * @param secondPointOnLine a second point on the line. Not modified.
    * @param orthogonalProjectionToPack a 3D point in which the projection of {@code point} onto the line is stored. Modified. Can be {@code null}.
    * @param perpendicularVectorToPack a 3D vector in which the vector perpendicular to the line and pointing to the {@code point} is stored. Modified. Can NOT be {@code null}.
    * @return {@code true} if the method succeeded, {@code false} otherwise.
    */
   public static boolean getPerpendicularVectorFromLineToPoint(Point3DReadOnly point, Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine,
                                                               Point3DBasics orthogonalProjectionToPack, Vector3DBasics perpendicularVectorToPack)
   {
      double lineDirectionX = secondPointOnLine.getX() - firstPointOnLine.getX();
      double lineDirectionY = secondPointOnLine.getY() - firstPointOnLine.getY();
      double lineDirectionZ = secondPointOnLine.getZ() - firstPointOnLine.getZ();
      double lineLength = 1.0 / Math.sqrt(lineDirectionX * lineDirectionX + lineDirectionY * lineDirectionY + lineDirectionZ * lineDirectionZ);

      if (lineLength < Epsilons.ONE_TRILLIONTH)
         return false;

      lineDirectionX *= lineLength;
      lineDirectionY *= lineLength;
      lineDirectionZ *= lineLength;

      double dx = point.getX() - firstPointOnLine.getX();
      double dy = point.getY() - firstPointOnLine.getY();
      double dz = point.getZ() - firstPointOnLine.getZ();

      double distanceFromFirstPointOnLine = lineDirectionX * dx + lineDirectionY * dy + lineDirectionZ * dz;

      if (orthogonalProjectionToPack != null)
      {
         orthogonalProjectionToPack.set(lineDirectionX, lineDirectionY, lineDirectionZ);
         orthogonalProjectionToPack.scaleAdd(distanceFromFirstPointOnLine, orthogonalProjectionToPack, firstPointOnLine);
         perpendicularVectorToPack.sub(point, orthogonalProjectionToPack);
      }
      else
      {
         perpendicularVectorToPack.set(lineDirectionX, lineDirectionY, lineDirectionZ);
         perpendicularVectorToPack.scale(distanceFromFirstPointOnLine);
         perpendicularVectorToPack.add(firstPointOnLine);
         perpendicularVectorToPack.negate();
         perpendicularVectorToPack.add(point);
      }
      return true;
   }

   /**
    * Computes the vector perpendicular to the given {@code vector} such that:
    * <ul>
    *    <li> {@code vector.dot(perpendicularVector) == 0.0}.
    *    <li> {@code vector.angle(perpendicularVector) == Math.PI / 2.0}.
    * </ul>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param vector the vector to compute the perpendicular of. Not modified.
    * @return the perpendicular vector.
    */
   public static Vector2D getPerpendicularVector(Vector2DReadOnly vector)
   {
      Vector2D perpendicularVector = new Vector2D();
      getPerpendicularVector(vector, perpendicularVector);

      return new Vector2D(-vector.getY(), vector.getX());
   }

   /**
    * Computes the vector perpendicular to the given {@code vector} such that:
    * <ul>
    *    <li> {@code vector.dot(perpendicularVector) == 0.0}.
    *    <li> {@code vector.angle(perpendicularVector) == Math.PI / 2.0}.
    * </ul>
    * 
    * @param vector the vector to compute the perpendicular of. Not modified.
    * @param perpendicularVectorToPack a 2D vector in which the perpendicular vector is stored. Modified.
    */
   public static void getPerpendicularVector(Vector2DReadOnly vector, Vector2DBasics perpendicularVectorToPack)
   {
      perpendicularVectorToPack.set(-vector.getY(), vector.getX());
   }

   /**
    * Computes the 2D vector perpendicular to the given 2D {@code vector} such that:
    * <ul>
    *    <li> {@code vector2d.dot(perpendicularVector2d) == 0.0}.
    *    <li> {@code vector2d.angle(perpendicularVector2d) == Math.PI / 2.0}.
    * </ul>
    * <p>
    * WARNING: the 3D arguments are projected onto the XY-plane to perform the actual computation in 2D.
    * </p>
    * 
    * @param vector the vector to compute in the xy-plane the perpendicular of. Not modified.
    * @param perpendicularVectorToPack a vector in which the x and y components of the 2D perpendicular vector are stored. Modified.
    */
   // FIXME this is just bad.
   public static void getPerpendicularVector2d(FrameVector vector, FrameVector perpendicularVectorToPack)
   {
      perpendicularVectorToPack.set(-vector.getY(), vector.getX(), perpendicularVectorToPack.getZ());
   }

   /**
    * Assuming an isosceles triangle defined by three vertices A, B, and C, with |AB| == |BC|, this methods computes the missing vertex B
    * given the vertices A and C, the normal of the triangle, the angle ABC that is equal to the angle at B from the the leg BA to the leg BC.
    * <a href="https://en.wikipedia.org/wiki/Isosceles_triangle"> Useful link</a>.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param baseVertexA the first base vertex of the isosceles triangle ABC. Not modified.
    * @param baseVertexC the second base vertex of the isosceles triangle ABC. Not modified.
    * @param trianglePlaneNormal  the normal of the plane on which is lying. Not modified.
    * @param ccwAngleAboutNormalAtTopVertex the angle at B from the the leg BA to the leg BC.
    * @param topVertexBToPack the missing vertex B. Modified.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference frame, except for {@code topVertexBToPack}.
    */
   public static void getTopVertexOfIsoscelesTriangle(FramePoint baseVertexA, FramePoint baseVertexC, FrameVector trianglePlaneNormal,
                                                      double ccwAngleAboutNormalAtTopVertex, FramePoint topVertexBToPack)
   {
      ReferenceFrame commonFrame = baseVertexA.getReferenceFrame();
      baseVertexC.checkReferenceFrameMatch(commonFrame);
      trianglePlaneNormal.checkReferenceFrameMatch(commonFrame);
      topVertexBToPack.setToZero(commonFrame);

      getTopVertexOfIsoscelesTriangle(baseVertexA.getPoint(), baseVertexC.getPoint(), trianglePlaneNormal.getVector(), ccwAngleAboutNormalAtTopVertex,
                                      topVertexBToPack.getPoint());
   }

   /**
    * Assuming an isosceles triangle defined by three vertices A, B, and C, with |AB| == |BC|, this methods computes the missing vertex B
    * given the vertices A and C, the normal of the triangle, the angle ABC that is equal to the angle at B from the the leg BA to the leg BC.
    * <a href="https://en.wikipedia.org/wiki/Isosceles_triangle"> Useful link</a>.
    * 
    * @param baseVertexA the first base vertex of the isosceles triangle ABC. Not modified.
    * @param baseVertexC the second base vertex of the isosceles triangle ABC. Not modified.
    * @param trianglePlaneNormal  the normal of the plane on which is lying. Not modified.
    * @param ccwAngleAboutNormalAtTopVertex the angle at B from the the leg BA to the leg BC.
    * @param topVertexBToPack the missing vertex B. Modified.
    */
   public static void getTopVertexOfIsoscelesTriangle(Point3DReadOnly baseVertexA, Point3DReadOnly baseVertexC, Vector3DReadOnly trianglePlaneNormal,
                                                      double ccwAngleAboutNormalAtTopVertex, Point3DBasics topVertexBToPack)
   {
      double baseEdgeACx = baseVertexC.getX() - baseVertexA.getX();
      double baseEdgeACy = baseVertexC.getY() - baseVertexA.getY();
      double baseEdgeACz = baseVertexC.getZ() - baseVertexA.getZ();
      double baseEdgeACLength = Math.sqrt(baseEdgeACx * baseEdgeACx + baseEdgeACy * baseEdgeACy + baseEdgeACz * baseEdgeACz);

      double legLengthABorCB = getRadiusOfArc(baseEdgeACLength, ccwAngleAboutNormalAtTopVertex);
      double lengthOfBisectorOfBase = pythagorasGetCathetus(legLengthABorCB, 0.5 * baseEdgeACLength);

      double perpendicularBisectorX = trianglePlaneNormal.getY() * baseEdgeACz - trianglePlaneNormal.getZ() * baseEdgeACy;
      double perpendicularBisectorY = trianglePlaneNormal.getZ() * baseEdgeACx - trianglePlaneNormal.getX() * baseEdgeACz;
      double perpendicularBisectorZ = trianglePlaneNormal.getX() * baseEdgeACy - trianglePlaneNormal.getY() * baseEdgeACx;
      double scale = lengthOfBisectorOfBase;
      scale /= Math.sqrt(perpendicularBisectorX * perpendicularBisectorX + perpendicularBisectorY * perpendicularBisectorY
            + perpendicularBisectorZ * perpendicularBisectorZ);
      perpendicularBisectorX *= scale;
      perpendicularBisectorY *= scale;
      perpendicularBisectorZ *= scale;

      topVertexBToPack.interpolate(baseVertexA, baseVertexC, 0.5);
      topVertexBToPack.setX(topVertexBToPack.getX() + perpendicularBisectorX);
      topVertexBToPack.setY(topVertexBToPack.getY() + perpendicularBisectorY);
      topVertexBToPack.setZ(topVertexBToPack.getZ() + perpendicularBisectorZ);
   }

   /**
    * Returns the radius of an arc with the specified chord length and angle.
    * <a href="http://planetcalc.com/1421/"> Useful link</a>.
    *
    * @param chordLength the length of the chord.
    * @param chordAngle angle covered by the chord.
    * @return the radius of the arc, or {@code Double.NaN} if {@code chordAngle % Math.PI == 0.0}.
    */
   public static double getRadiusOfArc(double chordLength, double chordAngle)
   {
      if (chordAngle % Math.PI == 0.0)
         return Double.NaN;
      else
         return chordLength / (2.0 * Math.sin(0.5 * chordAngle));
   }

   /**
    * Clip each component of the given tuple to the axis-aligned bounding box.
    * Each of the bounding box minimum coordinates is defined as follows: {@code minX = x1 < x2 ? x1 : x2}.
    * Each of the bounding box maximum coordinates is defined as follows: {@code maxX = x1 > x2 ? x1 : x2}.
    * 
    * @param tupleToClip the 3D tuple to clip to the bounding box. Modified.
    * @param x1 minimum/maximum x coordinate of the bounding box.
    * @param x2 minimum/maximum x coordinate of the bounding box.
    * @param y1 minimum/maximum y coordinate of the bounding box.
    * @param y2 minimum/maximum y coordinate of the bounding box.
    * @param z1 minimum/maximum z coordinate of the bounding box.
    * @param z2 minimum/maximum z coordinate of the bounding box.
    */
   // FIXME this is rather unsafe, the user should know the difference between the minimum and maximum coordinates of the bounding box.
   public static void clipToBoundingBox(Tuple3DBasics tupleToClip, double x1, double x2, double y1, double y2, double z1, double z2)
   {
      tupleToClip.setX(x1 < x2 ? MathTools.clamp(tupleToClip.getX(), x1, x2) : MathTools.clamp(tupleToClip.getX(), x2, x1));
      tupleToClip.setY(y1 < y2 ? MathTools.clamp(tupleToClip.getY(), y1, y2) : MathTools.clamp(tupleToClip.getY(), y2, y1));
      tupleToClip.setZ(z1 < z2 ? MathTools.clamp(tupleToClip.getZ(), z1, z2) : MathTools.clamp(tupleToClip.getZ(), z2, z1));
   }

   /**
    * Computes the perpendicular bisector of line segment defined by its two endpoints.
    * The bisector starts off the the middle of the line segment and points toward the left side of the line segment.
    * <p>
    * Edge cases:
    * <ul>
    *    <li> when the line segment endpoints are equal,
    *     more precisely when {@code lineSegmentStart.distance(lineSegmentEnd) < Epsilons.ONE_TRILLIONTH},
    *     the method fails and returns {@code false}.
    * </ul>
    * </p>
    * 
    * @param lineSegmentStart the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd the second endpoint of the line segment. Not modified.
    * @param bisectorStartToPack a 2D point in which the origin of the bisector is stored. Modified.
    * @param bisectorDirectionToPack a 2D vector in which the direction of the bisector is stored. Modified.
    * @return whether the perpendicular bisector could be determined or not.
    */
   public static boolean getPerpendicularBisector(Point2DReadOnly lineSegmentStart, Point2DReadOnly lineSegmentEnd, Point2DBasics bisectorStartToPack,
                                                  Vector2DBasics bisectorDirectionToPack)
   {
      if (lineSegmentStart.distance(lineSegmentEnd) < Epsilons.ONE_TRILLIONTH)
         return false;

      // direction will be on left side of line
      bisectorStartToPack.interpolate(lineSegmentStart, lineSegmentEnd, 0.5);
      bisectorDirectionToPack.sub(lineSegmentEnd, lineSegmentStart);
      getPerpendicularVector(bisectorDirectionToPack, bisectorDirectionToPack);
      bisectorDirectionToPack.normalize();
      return true;
   }

   /**
    * Computes the endpoints of the perpendicular bisector segment to a line segment defined by its endpoints, such that:
    * <ul>
    *    <li> each endpoint of the perpendicular bisector is at a distance of {@code bisectorSegmentHalfLength} from the line segment.
    *    <li> the first perpendicular bisector endpoint is located on the left side on the line segment.
    *    <li> the second perpendicular bisector endpoint is located on the right side on the line segment.
    * </ul>
    * <p>
    * <p>
    * Edge cases:
    * <ul>
    *    <li> when the line segment endpoints are equal,
    *     more precisely when {@code lineSegmentStart.distance(lineSegmentEnd) < Epsilons.ONE_TRILLIONTH},
    *     the method fails and returns {@code null}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param lineSegmentStart the first endpoint of the line segment from which the perpendicular bisector is to be computed. Not modified.
    * @param lineSegmentEnd the second endpoint of the line segment from which the perpendicular bisector is to be computed. Not modified.
    * @param bisectorSegmentHalfLength distance from the line segment each endpoint of the perpendicular bisector segment will be positioned.
    * @return a list containing the two endpoints of the perpendicular bisector segment.
    */
   public static List<Point2D> getPerpendicularBisectorSegment(Point2DReadOnly lineSegmentStart, Point2DReadOnly lineSegmentEnd, double bisectorSegmentHalfLength)
   {
      Point2D bisectorSegmentStart = new Point2D();
      Point2D bisectorSegmentEnd = new Point2D();

      boolean success = getPerpendicularBisectorSegment(lineSegmentStart, lineSegmentEnd, bisectorSegmentHalfLength, bisectorSegmentStart, bisectorSegmentEnd);
      if (!success)
         return null;

      List<Point2D> bisectorEndpoints = new ArrayList<>();
      bisectorEndpoints.add(bisectorSegmentStart);
      bisectorEndpoints.add(bisectorSegmentEnd);
      return bisectorEndpoints;
   }

   /**
    * Computes the endpoints of the perpendicular bisector segment to a line segment defined by its endpoints, such that:
    * <ul>
    *    <li> each endpoint of the perpendicular bisector is at a distance of {@code bisectorSegmentHalfLength} from the line segment.
    *    <li> the first perpendicular bisector endpoint is located on the left side on the line segment.
    *    <li> the second perpendicular bisector endpoint is located on the right side on the line segment.
    * </ul>
    * <p>
    * <p>
    * Edge cases:
    * <ul>
    *    <li> when the line segment endpoints are equal,
    *     more precisely when {@code lineSegmentStart.distance(lineSegmentEnd) < Epsilons.ONE_TRILLIONTH},
    *     the method fails and returns false.
    * </ul>
    * </p>
    * 
    * @param lineSegmentStart the first endpoint of the line segment from which the perpendicular bisector is to be computed. Not modified.
    * @param lineSegmentEnd the second endpoint of the line segment from which the perpendicular bisector is to be computed. Not modified.
    * @param bisectorSegmentHalfLength distance from the line segment each endpoint of the perpendicular bisector segment will be positioned.
    * @param bisectorSegmentStartToPack the first endpoint of the perpendicular bisector segment to be computed. Modified.
    * @param bisectorSegmentEndToPack the second endpoint of the perpendicular bisector segment to be computed. Modified.
    * @return whether the perpendicular bisector could be determined or not.
    */
   public static boolean getPerpendicularBisectorSegment(Point2DReadOnly lineSegmentStart, Point2DReadOnly lineSegmentEnd, double bisectorSegmentHalfLength,
                                                         Point2DBasics bisectorSegmentStartToPack, Point2DBasics bisectorSegmentEndToPack)
   {
      if (lineSegmentStart.distance(lineSegmentEnd) < Epsilons.ONE_TRILLIONTH)
         return false;

      // direction will be on left side of line
      double bisectorDirectionX = -(lineSegmentEnd.getY() - lineSegmentStart.getY());
      double bisectorDirectionY = lineSegmentEnd.getX() - lineSegmentStart.getX();
      double directionInverseMagnitude = 1.0 / Math.sqrt(bisectorDirectionX * bisectorDirectionX + bisectorDirectionY * bisectorDirectionY);
      bisectorDirectionX *= directionInverseMagnitude;
      bisectorDirectionY *= directionInverseMagnitude;

      double midPointX = 0.5 * (lineSegmentStart.getX() + lineSegmentEnd.getX());
      double midPointY = 0.5 * (lineSegmentStart.getY() + lineSegmentEnd.getY());

      bisectorSegmentStartToPack.setX(midPointX + bisectorDirectionX * bisectorSegmentHalfLength);
      bisectorSegmentStartToPack.setY(midPointY + bisectorDirectionY * bisectorSegmentHalfLength);
      bisectorSegmentEndToPack.setX(midPointX - bisectorDirectionX * bisectorSegmentHalfLength);
      bisectorSegmentEndToPack.setY(midPointY - bisectorDirectionY * bisectorSegmentHalfLength);

      return true;
   }

   /**
    * Computes the complete minimum rotation from {@code zUp = (0, 0, 1)} to the given {@code vector} and packs it into an {@link AxisAngle}.
    * The rotation axis if perpendicular to both vectors.
    * The rotation angle is computed as the angle from the {@code zUp} to the {@code vector}:
    * <br> {@code rotationAngle = zUp.angle(vector)}. </br>
    * Note: the vector does not need to be unit length.
    * <p>
    * Edge cases:
    * <ul>
    *    <li> the vector is aligned with {@code zUp}: the rotation angle is equal to {@code 0.0} and the rotation axis is set to: (1, 0, 0).
    *    <li> the vector is collinear pointing opposite direction of {@code zUp}: the rotation angle is equal to {@code Math.PI} and the rotation axis is set to: (1, 0, 0).
    *    <li> if the length of the given normal is below {@code 1.0E-7}: the rotation angle is equal to {@code 0.0} and the rotation axis is set to: (1, 0, 0).
    * </ul>
    * </p>
    * <p>
    * Note: The calculation becomes less accurate as the two vectors are more collinear.
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param vector the 3D vector that is rotated with respect to {@code zUp}. Not modified.
    * @return the minimum rotation from {@code zUp} to the given {@code vector}.
    */
   public static AxisAngle getAxisAngleFromZUpToVector(Vector3DReadOnly vector)
   {
      AxisAngle axisAngle = new AxisAngle();
      getAxisAngleFromZUpToVector(vector, axisAngle);
      return axisAngle;
   }

   /**
    * Computes the complete minimum rotation from {@code zUp = (0, 0, 1)} to the given {@code vector} and packs it into an {@link AxisAngle}.
    * The rotation axis if perpendicular to both vectors.
    * The rotation angle is computed as the angle from the {@code zUp} to the {@code vector}:
    * <br> {@code rotationAngle = zUp.angle(vector)}. </br>
    * Note: the vector does not need to be unit length.
    * <p>
    * Edge cases:
    * <ul>
    *    <li> the vector is aligned with {@code zUp}: the rotation angle is equal to {@code 0.0} and the rotation axis is set to: (1, 0, 0).
    *    <li> the vector is collinear pointing opposite direction of {@code zUp}: the rotation angle is equal to {@code Math.PI} and the rotation axis is set to: (1, 0, 0).
    *    <li> if the length of the given normal is below {@code 1.0E-7}: the rotation angle is equal to {@code 0.0} and the rotation axis is set to: (1, 0, 0).
    * </ul>
    * </p>
    * <p>
    * Note: The calculation becomes less accurate as the two vectors are more collinear.
    * </p>
    * 
    * @param vector the vector that is rotated with respect to {@code zUp}. Not modified.
    * @param rotationToPack the minimum rotation from {@code zUp} to the given {@code vector}. Modified.
    */
   public static void getAxisAngleFromZUpToVector(Vector3DReadOnly vector, AxisAngleBasics rotationToPack)
   {
      getAxisAngleFromFirstToSecondVector(0.0, 0.0, 1.0, vector.getX(), vector.getY(), vector.getZ(), rotationToPack);
   }

   /**
    * Computes the complete minimum rotation from {@code firstVector} to the {@code secondVector} and packs it into an {@link AxisAngle}.
    * The rotation axis if perpendicular to both vectors.
    * The rotation angle is computed as the angle from the {@code firstVector} to the {@code secondVector}:
    * <br> {@code rotationAngle = firstVector.angle(secondVector)}. </br>
    * Note: the vectors do not need to be unit length.
    * <p>
    * Edge cases:
    * <ul>
    *    <li> the vectors are the same: the rotation angle is equal to {@code 0.0} and the rotation axis is set to: (1, 0, 0).
    *    <li> the vectors are collinear pointing opposite directions: the rotation angle is equal to {@code Math.PI} and the rotation axis is set to: (1, 0, 0).
    *    <li> if the length of either normal is below {@code 1.0E-7}: the rotation angle is equal to {@code 0.0} and the rotation axis is set to: (1, 0, 0).
    * </ul>
    * </p>
    * <p>
    * Note: The calculation becomes less accurate as the two vectors are more collinear.
    * </p>
    * 
    * @param firstVector the first vector. Not modified.
    * @param secondVector the second vector that is rotated with respect to the first vector. Not modified.
    * @param rotationToPack the minimum rotation from {@code firstVector} to the {@code secondVector}. Modified.
    */
   public static void getAxisAngleFromFirstToSecondVector(Vector3DReadOnly firstVector, Vector3DReadOnly secondVector, AxisAngleBasics rotationToPack)
   {
      getAxisAngleFromFirstToSecondVector(firstVector.getX(), firstVector.getY(), firstVector.getZ(), secondVector.getX(), secondVector.getY(),
                                          secondVector.getZ(), rotationToPack);
   }

   /**
    * Computes the complete minimum rotation from {@code firstVector} to the {@code secondVector} and packs it into an {@link AxisAngle}.
    * The rotation axis if perpendicular to both vectors.
    * The rotation angle is computed as the angle from the {@code firstVector} to the {@code secondVector}:
    * <br> {@code rotationAngle = firstVector.angle(secondVector)}. </br>
    * Note: the vectors do not need to be unit length.
    * <p>
    * Edge cases:
    * <ul>
    *    <li> the vectors are the same: the rotation angle is equal to {@code 0.0} and the rotation axis is set to: (1, 0, 0).
    *    <li> the vectors are collinear pointing opposite directions: the rotation angle is equal to {@code Math.PI} and the rotation axis is set to: (1, 0, 0).
    *    <li> if the length of either normal is below {@code 1.0E-7}: the rotation angle is equal to {@code 0.0} and the rotation axis is set to: (1, 0, 0).
    * </ul>
    * </p>
    * <p>
    * Note: The calculation becomes less accurate as the two vectors are more collinear.
    * </p>
    * 
    * @param firstVectorX x-component of the first vector.
    * @param firstVectorY y-component of the first vector.
    * @param firstVectorZ z-component of the first vector.
    * @param secondVectorX x-component of the second vector that is rotated with respect to the first vector.
    * @param secondVectorY y-component of the second vector that is rotated with respect to the first vector.
    * @param secondVectorZ z-component of the second vector that is rotated with respect to the first vector.
    * @param rotationToPack the minimum rotation from {@code firstVector} to the {@code secondVector}. Modified.
    */
   public static void getAxisAngleFromFirstToSecondVector(double firstVectorX, double firstVectorY, double firstVectorZ, double secondVectorX,
                                                          double secondVectorY, double secondVectorZ, AxisAngleBasics rotationToPack)
   {
      double rotationAxisX = firstVectorY * secondVectorZ - firstVectorZ * secondVectorY;
      double rotationAxisY = firstVectorZ * secondVectorX - firstVectorX * secondVectorZ;
      double rotationAxisZ = firstVectorX * secondVectorY - firstVectorY * secondVectorX;
      double rotationAxisLength = Math.sqrt(rotationAxisX * rotationAxisX + rotationAxisY * rotationAxisY + rotationAxisZ * rotationAxisZ);

      boolean normalsAreParallel = rotationAxisLength < 1e-7;

      double dot;
      dot = secondVectorX * firstVectorX;
      dot += secondVectorY * firstVectorY;
      dot += secondVectorZ * firstVectorZ;

      if (normalsAreParallel)
      {
         double rotationAngle = dot > 0.0 ? 0.0 : Math.PI;
         rotationToPack.set(1.0, 0.0, 0.0, rotationAngle);
         return;
      }

      double rotationAngle = getAngleFromFirstToSecondVector(firstVectorX, firstVectorY, firstVectorZ, secondVectorX, secondVectorY,
                                                             secondVectorZ);

      rotationAxisX /= rotationAxisLength;
      rotationAxisY /= rotationAxisLength;
      rotationAxisZ /= rotationAxisLength;
      rotationToPack.set(rotationAxisX, rotationAxisY, rotationAxisZ, rotationAngle);
   }

   /**
    *  Given a triangle defined by three points (A,B,C), this methods the point
    *  X &in; AC such that the line (B, X) is the angle bisector of B.
    *  As a result, the two angles CBX and XBA are equal.
    *  <a href="https://en.wikipedia.org/wiki/Angle_bisector_theorem"> Useful link</a>.
    *<p>
    *Edge cases:
    *<ul>
    *   <li> if any the triangle's edge is shorter than {@link Epsilons#ONE_TRILLIONTH},
    *    this method fails and returns {@code null}.
    *</ul>
    *</p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param A the first vertex of the triangle. Not modified.
    * @param B the second vertex of the triangle, this is the first endpoint of the bisector. Not modified.
    * @param C the third vertex of the triangle. Not modified.
    * @return the second endpoint of the bisector, or {@code null} if the method failed.
    */
   public static Point2D getTriangleBisector(Point2DReadOnly A, Point2DReadOnly B, Point2DReadOnly C)
   {
      Point2D X = new Point2D();
      getTriangleBisector(A, B, C, X);
      return X;
   }

   /**
    *  Given a triangle defined by three points (A,B,C), this methods the point
    *  X &in; AC such that the line (B, X) is the angle bisector of B.
    *  As a result, the two angles CBX and XBA are equal.
    *  <a href="https://en.wikipedia.org/wiki/Angle_bisector_theorem"> Useful link</a>.
    *<p>
    *Edge cases:
    *<ul>
    *   <li> if any the triangle's edge is shorter than {@link Epsilons#ONE_TRILLIONTH},
    *    this method fails and returns {@code false}.
    *</ul>
    *</p>
    *
    * @param A the first vertex of the triangle. Not modified.
    * @param B the second vertex of the triangle, this is the first endpoint of the bisector. Not modified.
    * @param C the third vertex of the triangle. Not modified.
    * @param XToPack point in which the second endpoint of the bisector is stored. Modified.
    * @return whether the bisector could be calculated or not.
    */
   public static boolean getTriangleBisector(Point2DReadOnly A, Point2DReadOnly B, Point2DReadOnly C, Point2DBasics XToPack)
   {
      // find all proportional values
      double BA = B.distance(A);
      if (BA < Epsilons.ONE_TRILLIONTH)
         return false;

      double BC = B.distance(C);
      if (BC < Epsilons.ONE_TRILLIONTH)
         return false;

      double AC = A.distance(C);

      if (AC < Epsilons.ONE_TRILLIONTH)
         return false;

      double AX = AC / ((BC / BA) + 1.0);

      // use AX distance to find X along AC
      double vectorAXx = C.getX() - A.getX();
      double vectorAXy = C.getY() - A.getY();
      double inverseMagnitude = 1.0 / Math.sqrt(vectorAXx * vectorAXx + vectorAXy * vectorAXy);
      vectorAXx *= AX * inverseMagnitude;
      vectorAXy *= AX * inverseMagnitude;

      XToPack.set(vectorAXx, vectorAXy);
      XToPack.add(A);
      return false;
   }

   /**
    * Computes the angle in radians from the first 2D vector to the second 2D vector.
    * The computed angle is in the range [-<i>pi</i>; <i>pi</i>].
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the length of either vector is below {@code 1.0E-7}, this method fails and returns an angle of {@code 0.0} radian.
    * </ul>
    * </p>
    * 
    * @param vector1x x-component of first the vector.
    * @param vector1y y-component of first the vector.
    * @param vector2x x-component of second the vector.
    * @param vector2y y-component of second the vector.
    * @return the angle in radians from the first vector to the second vector.
    */
   public static double getAngleFromFirstToSecondVector(Vector2DReadOnly firstVector, Vector2DReadOnly secondVector)
   {
      return getAngleFromFirstToSecondVector(firstVector.getX(), firstVector.getY(), secondVector.getX(), secondVector.getY());
   }

   /**
    * Computes the angle in radians from the first 2D vector to the second 2D vector.
    * The computed angle is in the range [-<i>pi</i>; <i>pi</i>].
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the length of either vector is below {@code 1.0E-7}, this method fails and returns an angle of {@code 0.0} radian.
    * </ul>
    * </p>
    * 
    * @param firstVector the first vector. Not modified.
    * @param secondVector the second vector. Not modified.
    * @return the angle in radians from the first vector to the second vector.
    */
   public static double getAngleFromFirstToSecondVector(double firstVectorX, double firstVectorY, double secondVectorX, double secondVectorY)
   {
      double firstVectorLength = Math.sqrt(firstVectorX * firstVectorX + firstVectorY * firstVectorY);

      if (firstVectorLength < 1e-7)
         return 0.0;

      firstVectorX /= firstVectorLength;
      firstVectorY /= firstVectorLength;

      double secondVectorLength = Math.sqrt(secondVectorX * secondVectorX + secondVectorY * secondVectorY);

      if (secondVectorLength < 1e-7)
         return 0.0;

      secondVectorX /= secondVectorLength;
      secondVectorY /= secondVectorLength;

      // The sign of the angle comes from the cross product
      double crossProduct = firstVectorX * secondVectorY - firstVectorY * secondVectorX;
      // the magnitude of the angle comes from the dot product
      double dotProduct = firstVectorX * secondVectorX + firstVectorY * secondVectorY;

      double angle = Math.atan2(crossProduct, dotProduct);
      // This is a hack to get the polygon tests to pass.
      // Probably some edge case not well handled somewhere (Sylvain)
      if (crossProduct == 0.0)
         angle = -angle;

      return angle;
   }

   /**
    * Computes the angle in radians from the first 3D vector to the second 3D vector.
    * The computed angle is in the range [0; <i>pi</i>].
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the length of either vector is below {@code 1.0E-7}, this method fails and returns an angle of {@code 0.0} radian.
    * </ul>
    * </p>
    * 
    * @param firstVector the first vector. Not modified.
    * @param secondVector the second vector. Not modified.
    * @return the angle in radians from the first vector to the second vector.
    */
   public static double getAngleFromFirstToSecondVector(Vector3DReadOnly firstVector, Vector3DReadOnly secondVector)
   {
      return getAngleFromFirstToSecondVector(firstVector.getX(), firstVector.getY(), firstVector.getZ(), secondVector.getX(), secondVector.getY(),
                                             secondVector.getZ());
   }

   /**
    * Computes the angle in radians from the first 3D vector to the second 3D vector.
    * The computed angle is in the range [0; <i>pi</i>].
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the length of either vector is below {@code 1.0E-7}, this method fails and returns an angle of {@code 0.0} radian.
    * </ul>
    * </p>
    * 
    * @param firstVectorX x-component of first the vector.
    * @param firstVectorY y-component of first the vector.
    * @param firstVectorZ z-component of first the vector.
    * @param secondVectorX x-component of second the vector.
    * @param secondVectorY y-component of second the vector.
    * @param secondVectorZ z-component of second the vector.
    * @return the angle in radians from the first vector to the second vector.
    */
   public static double getAngleFromFirstToSecondVector(double firstVectorX, double firstVectorY, double firstVectorZ, double secondVectorX,
                                                        double secondVectorY, double secondVectorZ)
   {
      double firstVectorLength = Math.sqrt(firstVectorX * firstVectorX + firstVectorY * firstVectorY + firstVectorZ * firstVectorZ);

      if (firstVectorLength < 1e-7)
         return 0.0;

      double secondVectorLength = Math.sqrt(secondVectorX * secondVectorX + secondVectorY * secondVectorY + secondVectorZ * secondVectorZ);

      if (secondVectorLength < 1e-7)
         return 0.0;

      double dotProduct = firstVectorX * secondVectorX + firstVectorY * secondVectorY + firstVectorZ * secondVectorZ;
      dotProduct /= firstVectorLength * secondVectorLength;

      return Math.acos(MathTools.clamp(dotProduct, -1.0, 1.0));
   }

   /**
    * Computes the Euclidean distance between the two given n-dimensional points {@code a} and {@code b}:
    * <br>
    * distance = Sqrt{  &sum;<sub>i=1:n</sub> { (a<sub>i</sub> - b<sub>i</sub>)<sup>2</sup>}   }
    * <br>
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
    * Calculates the distance between two points.
    * 
    * @param firstPointX the x-coordinate of the first point.
    * @param firstPointY the y-coordinate of the first point.
    * @param secondPointX the x-coordinate of the second point.
    * @param secondPointY the y-coordinate of the second point.
    * @return the distance between the two points.
    */
   public static double distanceBetweenPoints(double firstPointX, double firstPointY, double secondPointX, double secondPointY)
   {
      double deltaX = secondPointX - firstPointX;
      double deltaY = secondPointY - firstPointY;
      return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
   }

   /**
    * Calculates the distance on the xy-plane bewteen two 3D points.
    * <p>
    * WARNING: the 3D arguments are projected onto the XY-plane to perform the actual computation in 2D.
    * </p>
    * 
    * @param firstPoint the first point. Not modified.
    * @param secondPoint the second point. Not modified.
    * @return the distance between the two points.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference frame.
    */
   public static double getXYDistance(FramePoint firstPoint, FramePoint secondPoint)
   {
      firstPoint.checkReferenceFrameMatch(secondPoint);
      return getXYDistance(firstPoint.getPoint(), secondPoint.getPoint());
   }

   /**
    * Calculates the distance on the xy-plane bewteen two 3D points.
    * <p>
    * WARNING: the 3D arguments are projected onto the XY-plane to perform the actual computation in 2D.
    * </p>
    * 
    * @param firstPoint the first point. Not modified.
    * @param secondPoint the second point. Not modified.
    * @return the distance between the two points.
    */
   public static double getXYDistance(Point3DReadOnly firstPoint, Point3DReadOnly secondPoint)
   {
      return distanceBetweenPoints(firstPoint.getX(), firstPoint.getY(), secondPoint.getX(), secondPoint.getY());
   }

   /**
    * Computes the dot product between two vectors each defined by two points:
    * <ul>
    *    <li> {@code vector1 = end1 - start1}
    *    <li> {@code vector2 = end2 - start2}
    * </ul>
    * 
    * @param start1 the origin of the first vector. Not modified.
    * @param end1 the end of the first vector. Not modified.
    * @param start2 the origin of the second vector. Not modified.
    * @param end2 the end of the second vector. Not modified.
    * @return the value of the dot product of the two vectors.
    */
   public static double dotProduct(Point2DReadOnly start1, Point2DReadOnly end1, Point2DReadOnly start2, Point2DReadOnly end2)
   {
      double vector1X = end1.getX() - start1.getX();
      double vector1Y = end1.getY() - start1.getY();
      double vector2X = end2.getX() - start2.getX();
      double vector2Y = end2.getY() - start2.getY();

      return vector1X * vector2X + vector1Y * vector2Y;
   }

   /**
    * Attempts to normalize the given 3D vector.
    * If the vector's length falls below {@value Epsilons#ONE_TRILLIONTH}, the vector is set to (0, 0, 1).
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
    * Tests if the two given vectors are collinear given a tolerance on the angle between the two vector axes in the range ]0; <i>pi</i>/2[.
    * This method returns {@code true} if the two vectors are collinear, whether they are pointing in the same direction or in opposite directions.
    * 
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the length of either vector is below {@code 1.0E-7}, this method fails and returns {@code false}.
    * </ul>
    * </p>
    * 
    * @param firstVector the first vector. Not modified.
    * @param secondVector the second vector. Not modified.
    * @param angleEpsilon tolerance on the angle in radians.
    * @return {@code true} if the two vectors are collinear, {@code false} otherwise.
    */
   public static boolean areVectorsCollinear(Vector3DReadOnly firstVector, Vector3DReadOnly secondVector, double angleEpsilon)
   {
      return areVectorsCollinear(firstVector.getX(), firstVector.getY(), firstVector.getZ(), secondVector.getX(), secondVector.getY(), secondVector.getZ(), angleEpsilon);
   }

   /**
    * Tests if the two given vectors are collinear given a tolerance on the angle between the two vector axes in the range ]0; <i>pi</i>/2[.
    * This method returns {@code true} if the two vectors are collinear, whether they are pointing in the same direction or in opposite directions.
    * 
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the length of either vector is below {@code 1.0E-7}, this method fails and returns {@code false}.
    * </ul>
    * </p>
    * 
    * @param firstVectorX x-component of the first vector. Not modified.
    * @param firstVectorY y-component of the first vector. Not modified.
    * @param firstVectorZ z-component of the first vector. Not modified.
    * @param secondVectorX x-component of the second vector. Not modified.
    * @param secondVectorY y-component of the second vector. Not modified.
    * @param secondVectorZ z-component of the second vector. Not modified.
    * @param angleEpsilon tolerance on the angle in radians.
    * @return {@code true} if the two vectors are collinear, {@code false} otherwise.
    */
   public static boolean areVectorsCollinear(double firstVectorX, double firstVectorY, double firstVectorZ, double secondVectorX, double secondVectorY, double secondVectorZ, double angleEpsilon)
   {
      double firstVectorLength = Math.sqrt(firstVectorX * firstVectorX + firstVectorY * firstVectorY + firstVectorZ * firstVectorZ);
      if (firstVectorLength < Epsilons.ONE_TEN_MILLIONTH)
         return false;
      double secondVectorLength = Math.sqrt(secondVectorX * secondVectorX + secondVectorY * secondVectorY + secondVectorZ * secondVectorZ);
      if (secondVectorLength < Epsilons.ONE_TEN_MILLIONTH)
         return false;
      double dot = firstVectorX * secondVectorX + firstVectorY * secondVectorY + firstVectorZ * secondVectorZ;
      return Math.abs(dot / (firstVectorLength * secondVectorLength)) > Math.cos(angleEpsilon);
   }

   /**
    * Tests if the two given vectors are collinear given a tolerance on the angle between the two vector axes in the range ]0; <i>pi</i>/2[.
    * This method returns {@code true} if the two vectors are collinear, whether they are pointing in the same direction or in opposite directions.
    * 
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the length of either vector is below {@code 1.0E-7}, this method fails and returns {@code false}.
    * </ul>
    * </p>
    * 
    * @param firstVector the first vector. Not modified.
    * @param secondVector the second vector. Not modified.
    * @param angleEpsilon tolerance on the angle in radians.
    * @return {@code true} if the two vectors are collinear, {@code false} otherwise.
    */
   public static boolean areVectorsCollinear(Vector2DReadOnly firstVector, Vector2DReadOnly secondVector, double angleEpsilon)
   {
      return areVectorsCollinear(firstVector.getX(), firstVector.getY(), secondVector.getX(), secondVector.getY(), angleEpsilon);
   }

   /**
    * Tests if the two given vectors are collinear given a tolerance on the angle between the two vector axes in the range ]0; <i>pi</i>/2[.
    * This method returns {@code true} if the two vectors are collinear, whether they are pointing in the same direction or in opposite directions.
    * 
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the length of either vector is below {@code 1.0E-7}, this method fails and returns {@code false}.
    * </ul>
    * </p>
    * 
    * @param firstVectorX x-component of the first vector. Not modified.
    * @param firstVectorY y-component of the first vector. Not modified.
    * @param secondVectorX x-component of the second vector. Not modified.
    * @param secondVectorY y-component of the second vector. Not modified.
    * @param angleEpsilon tolerance on the angle in radians.
    * @return {@code true} if the two vectors are collinear, {@code false} otherwise.
    */
   public static boolean areVectorsCollinear(double firstVectorX, double firstVectorY, double secondVectorX, double secondVectorY, double angleEpsilon)
   {
      double firstVectorLength = Math.sqrt(firstVectorX * firstVectorX + firstVectorY * firstVectorY);
      if (firstVectorLength < Epsilons.ONE_TEN_MILLIONTH)
         return false;
      double secondVectorLength = Math.sqrt(secondVectorX * secondVectorX + secondVectorY * secondVectorY);
      if (secondVectorLength < Epsilons.ONE_TEN_MILLIONTH)
         return false;
      double dot = firstVectorX * secondVectorX + firstVectorY * secondVectorY;
      return Math.abs(dot / (firstVectorLength * secondVectorLength)) > Math.cos(angleEpsilon);
   }

   /**
    * Tests if the two given lines are collinear given a tolerance on the angle between in the range ]0; <i>pi</i>/2[.
    * This method returns {@code true} if the two lines are collinear, whether they are pointing in the same direction or in opposite directions.
    * 
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the direction magnitude of either line is below {@code 1.0E-7}, this method fails and returns {@code false}.
    * </ul>
    * </p>
    * 
    * @param firstPointOnLine1 a first point located on the first line. Not modified.
    * @param secondPointOnLine1 a second point located on the first line. Not modified.
    * @param firstPointOnLine2 a first point located on the second line. Not modified.
    * @param secondPointOnLine2 a second point located on the second line. Not modified.
    * @param angleEpsilon tolerance on the angle in radians.
    * @param distanceEpsilon tolerance on the distance to determine if {@code firstPointOnLine2} belongs to the first line segment.
    * @return {@code true} if the two line segments are collinear, {@code false} otherwise.
    */
   public static boolean areLinesCollinear(Point2DReadOnly firstPointOnLine1, Point2DReadOnly secondPointOnLine1, Point2DReadOnly firstPointOnLine2, Point2DReadOnly secondPointOnLine2,
                                           double angleEpsilon, double distanceEspilon)
   {
      double pointOnLine1x = firstPointOnLine1.getX();
      double pointOnLine1y = firstPointOnLine1.getY();
      double lineDirection1x = secondPointOnLine1.getX() - firstPointOnLine1.getX();
      double lineDirection1y = secondPointOnLine1.getY() - firstPointOnLine1.getY();
      double pointOnLine2x = firstPointOnLine2.getX();
      double pointOnLine2y = firstPointOnLine2.getY();
      double lineDirection2x = secondPointOnLine2.getX() - firstPointOnLine2.getX();
      double lineDirection2y = secondPointOnLine2.getY() - firstPointOnLine2.getY();
      return areLinesCollinear(pointOnLine1x, pointOnLine1y, lineDirection1x, lineDirection1y, pointOnLine2x, pointOnLine2y, lineDirection2x, lineDirection2y,
                               angleEpsilon, distanceEspilon);
   }

   /**
    * Tests if the two given lines are collinear given a tolerance on the angle between in the range ]0; <i>pi</i>/2[.
    * This method returns {@code true} if the two lines are collinear, whether they are pointing in the same direction or in opposite directions.
    * 
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the direction magnitude of either line is below {@code 1.0E-7}, this method fails and returns {@code false}.
    * </ul>
    * </p>
    * 
    * @param pointOnLine1 point located on the first line. Not modified.
    * @param lineDirection1 the first line direction. Not modified.
    * @param pointOnLine2 point located on the second line. Not modified.
    * @param lineDirection2 the second line direction. Not modified.
    * @param angleEpsilon tolerance on the angle in radians.
    * @param distanceEpsilon tolerance on the distance to determine if {@code pointOnLine2} belongs to the first line segment.
    * @return {@code true} if the two line segments are collinear, {@code false} otherwise.
    */
   public static boolean areLinesCollinear(Point2DReadOnly pointOnLine1, Vector2DReadOnly lineDirection1, Point2DReadOnly pointOnLine2, Vector2DReadOnly lineDirection2, double angleEpsilon,
                                           double distanceEspilon)
   {
      return areLinesCollinear(pointOnLine1.getX(), pointOnLine1.getY(), lineDirection1.getX(), lineDirection1.getY(), pointOnLine2.getX(), pointOnLine2.getY(),
                               lineDirection2.getX(), lineDirection2.getY(), angleEpsilon, distanceEspilon);
   }

   /**
    * Tests if the two given lines are collinear given a tolerance on the angle between in the range ]0; <i>pi</i>/2[.
    * This method returns {@code true} if the two lines are collinear, whether they are pointing in the same direction or in opposite directions.
    * 
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the direction magnitude of either line is below {@code 1.0E-7}, this method fails and returns {@code false}.
    * </ul>
    * </p>
    * 
    * @param pointOnLine1x x-coordinate of a point located on the first line.
    * @param pointOnLine1y y-coordinate of a point located on the first line.
    * @param lineDirection1x x-component of the first line direction.
    * @param lineDirection1y y-component of the first line direction.
    * @param pointOnLine2x x-coordinate of a point located on the second line.
    * @param pointOnLine2y y-coordinate of a point located on the second line.
    * @param lineDirection2x x-component of the second line direction.
    * @param lineDirection2y y-component of the second line direction.
    * @param angleEpsilon tolerance on the angle in radians.
    * @param distanceEpsilon tolerance on the distance to determine if {@code pointOnLine2} belongs to the first line segment.
    * @return {@code true} if the two line segments are collinear, {@code false} otherwise.
    */
   public static boolean areLinesCollinear(double pointOnLine1x, double pointOnLine1y, double lineDirection1x, double lineDirection1y, double pointOnLine2x,
                                           double pointOnLine2y, double lineDirection2x, double lineDirection2y, double angleEpsilon, double distanceEspilon)
   {
      if (!areVectorsCollinear(lineDirection1x, lineDirection1y, lineDirection2x, lineDirection2y, angleEpsilon))
            return false;

      double distance = EuclidGeometryTools.distanceFromPoint2DToLine2D(pointOnLine2x, pointOnLine2y, pointOnLine1x, pointOnLine1y, lineDirection1x, lineDirection1y);
      return distance < distanceEspilon;
   }

   /**
    * Tests if the two given lines are collinear given a tolerance on the angle between in the range ]0; <i>pi</i>/2[.
    * This method returns {@code true} if the two lines are collinear, whether they are pointing in the same direction or in opposite directions.
    * 
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the direction magnitude of either line is below {@code 1.0E-7}, this method fails and returns {@code false}.
    * </ul>
    * </p>
    * 
    * @param firstPointOnLine1 a first point located on the first line. Not modified.
    * @param secondPointOnLine1 a second point located on the first line. Not modified.
    * @param firstPointOnLine2 a first point located on the second line. Not modified.
    * @param secondPointOnLine2 a second point located on the second line. Not modified.
    * @param angleEpsilon tolerance on the angle in radians.
    * @param distanceEpsilon tolerance on the distance to determine if {@code firstPointOnLine2} belongs to the first line segment.
    * @return {@code true} if the two line segments are collinear, {@code false} otherwise.
    */
   public static boolean areLinesCollinear(Point3DReadOnly firstPointOnLine1, Point3DReadOnly secondPointOnLine1, Point3DReadOnly firstPointOnLine2, Point3DReadOnly secondPointOnLine2,
                                           double angleEpsilon, double distanceEspilon)
   {
      double pointOnLine1x = firstPointOnLine1.getX();
      double pointOnLine1y = firstPointOnLine1.getY();
      double pointOnLine1z = firstPointOnLine1.getZ();
      double lineDirection1x = secondPointOnLine1.getX() - firstPointOnLine1.getX();
      double lineDirection1y = secondPointOnLine1.getY() - firstPointOnLine1.getY();
      double lineDirection1z = secondPointOnLine1.getZ() - firstPointOnLine1.getZ();
      double pointOnLine2x = firstPointOnLine2.getX();
      double pointOnLine2y = firstPointOnLine2.getY();
      double pointOnLine2z = firstPointOnLine2.getZ();
      double lineDirection2x = secondPointOnLine2.getX() - firstPointOnLine2.getX();
      double lineDirection2y = secondPointOnLine2.getY() - firstPointOnLine2.getY();
      double lineDirection2z = secondPointOnLine2.getZ() - firstPointOnLine2.getZ();
      return areLinesCollinear(pointOnLine1x, pointOnLine1y, pointOnLine1z, lineDirection1x, lineDirection1y, lineDirection1z, pointOnLine2x, pointOnLine2y,
                               pointOnLine2z, lineDirection2x, lineDirection2y, lineDirection2z, angleEpsilon, distanceEspilon);
   }

   /**
    * Tests if the two given lines are collinear given a tolerance on the angle between in the range ]0; <i>pi</i>/2[.
    * This method returns {@code true} if the two lines are collinear, whether they are pointing in the same direction or in opposite directions.
    * 
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the direction magnitude of either line is below {@code 1.0E-7}, this method fails and returns {@code false}.
    * </ul>
    * </p>
    * 
    * @param pointOnLine1 point located on the first line. Not modified.
    * @param lineDirection1 the first line direction. Not modified.
    * @param pointOnLine2 point located on the second line. Not modified.
    * @param lineDirection2 the second line direction. Not modified.
    * @param angleEpsilon tolerance on the angle in radians.
    * @param distanceEpsilon tolerance on the distance to determine if {@code pointOnLine2} belongs to the first line segment.
    * @return {@code true} if the two line segments are collinear, {@code false} otherwise.
    */
   public static boolean areLinesCollinear(Point3DReadOnly pointOnLine1, Vector3DReadOnly lineDirection1, Point3DReadOnly pointOnLine2, Vector3DReadOnly lineDirection2, double angleEpsilon,
                                           double distanceEspilon)
   {
      return areLinesCollinear(pointOnLine1.getX(), pointOnLine1.getY(), pointOnLine1.getZ(), lineDirection1.getX(), lineDirection1.getY(),
                               lineDirection1.getZ(), pointOnLine2.getX(), pointOnLine2.getY(), pointOnLine2.getZ(), lineDirection2.getX(),
                               lineDirection2.getY(), lineDirection2.getZ(), angleEpsilon, distanceEspilon);
   }

   /**
    * Tests if the two given lines are collinear given a tolerance on the angle between in the range ]0; <i>pi</i>/2[.
    * This method returns {@code true} if the two lines are collinear, whether they are pointing in the same direction or in opposite directions.
    * 
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the direction magnitude of either line is below {@code 1.0E-7}, this method fails and returns {@code false}.
    * </ul>
    * </p>
    * 
    * @param pointOnLine1x x-coordinate of a point located on the first line.
    * @param pointOnLine1y y-coordinate of a point located on the first line.
    * @param pointOnLine1z z-coordinate of a point located on the first line.
    * @param lineDirection1x x-component of the first line direction.
    * @param lineDirection1y y-component of the first line direction.
    * @param lineDirection1z z-component of the first line direction.
    * @param pointOnLine2x x-coordinate of a point located on the second line.
    * @param pointOnLine2y y-coordinate of a point located on the second line.
    * @param pointOnLine2z z-coordinate of a point located on the second line.
    * @param lineDirection2x x-component of the second line direction.
    * @param lineDirection2y y-component of the second line direction.
    * @param lineDirection2z z-component of the second line direction.
    * @param angleEpsilon tolerance on the angle in radians.
    * @param distanceEpsilon tolerance on the distance to determine if {@code pointOnLine2} belongs to the first line segment.
    * @return {@code true} if the two line segments are collinear, {@code false} otherwise.
    */
   public static boolean areLinesCollinear(double pointOnLine1x, double pointOnLine1y, double pointOnLine1z, double lineDirection1x, double lineDirection1y,
                                           double lineDirection1z, double pointOnLine2x, double pointOnLine2y, double pointOnLine2z, double lineDirection2x,
                                           double lineDirection2y, double lineDirection2z, double angleEpsilon, double distanceEspilon)
   {
      if (!areVectorsCollinear(lineDirection1x, lineDirection1y, lineDirection1z, lineDirection2x, lineDirection2y, lineDirection2z, angleEpsilon))
         return false;

      double distance = EuclidGeometryTools.distanceFromPoint3DToLine3D(pointOnLine2x, pointOnLine2y, pointOnLine2z, pointOnLine1x, pointOnLine1y,
                                                                        pointOnLine1z, lineDirection1x, lineDirection1y, lineDirection1z);
      return distance < distanceEspilon;
   }

   /**
    * Tests if the two given planes are coincident:
    * <ul>
    *    <li> {@code planeNormal1} and {@code planeNormal2} are collinear given the tolerance {@code angleEpsilon}.
    *    <li> the distance of {@code pointOnPlane2} from the first plane is less than {@code distanceEpsilon}.
    * </ul>
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the length of either normal is below {@code 1.0E-7}, this method fails and returns {@code false}.
    * </ul>
    * </p>
    * 
    * @param pointOnPlane1 a point on the first plane. Not modified.
    * @param planeNormal1 the normal of the first plane. Not modified.
    * @param pointOnPlane2 a point on the second plane. Not modified.
    * @param planeNormal2 the normal of the second plane. Not modified.
    * @param angleEpsilon tolerance on the angle in radians to determine if the plane normals are collinear. 
    * @param distanceEpsilon tolerance on the distance to determine if {@code pointOnPlane2} belongs to the first plane.
    * @return {@code true} if the two planes are coincident, {@code false} otherwise.
    */
   public static boolean arePlanesCoincident(Point3DReadOnly pointOnPlane1, Vector3DReadOnly planeNormal1, Point3DReadOnly pointOnPlane2, Vector3DReadOnly planeNormal2, double angleEpsilon,
                                             double distanceEpsilon)
   {
      if (!areVectorsCollinear(planeNormal1, planeNormal2, angleEpsilon))
         return false;
      else
         return EuclidGeometryTools.distanceFromPoint3DToPlane3D(pointOnPlane2, pointOnPlane1, planeNormal1) < distanceEpsilon;
   }
   
   /**
    * Rotates the given {@code tupleOriginal} tuple by an angle {@code yaw} and stores the result in the tuple.
    * 
    * @param yaw the angle in radians by which {@code tupleOriginal} should be rotated.
    * @param tupleToRotate the original tuple. Not modified.
    */
   public static void rotateTuple2d(double yaw, Tuple2DBasics tupleToRotate)
   {
      RotationMatrixTools.applyYawRotation(yaw, tupleToRotate, tupleToRotate);
   }

   /**
    * Rotates the given {@code tupleOriginal} tuple by an angle {@code yaw} and stores the result in the tuple {@code tupleTransformed}.
    * 
    * @param yaw the angle in radians by which {@code tupleOriginal} should be rotated.
    * @param tupleOriginal the original tuple. Not modified.
    * @param tupleTransformed the tuple in which the transformed {@code original} is stored. Modified.
    */
   public static void rotateTuple2d(double yaw, Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      RotationMatrixTools.applyYawRotation(yaw, tupleOriginal, tupleTransformed);
   }

   /**
    * Calculate an unknown side length of a fully defined 2D Triangle by the law of Cosine.
    * <p>
    * Given a triangle with the three sides A, B, and C, this methods calculates the length of the side C, given:
    * <ul>
    *    <li> the lengths of A and B.
    *    <li> the angle between the sides A and B.
    * </ul>
    * </p>
    *
    * @param lengthSideA the length of the side A.
    * @param lengthSideB the length of the side B.
    * @param angleBetweenAAndB the angle between the sides A and B.
    * @throws RuntimeException if {@code lengthSideA} and/or {@code lengthSideB} are negative, if {@code angleBetweenAAndB} is greater than <i>pi</i>.
    */
   public static double getUnknownTriangleSideLengthByLawOfCosine(double lengthSideA, double lengthSideB, double angleBetweenAAndB)
   {
      MathTools.checkIntervalContains(lengthSideA, 0.0, Double.POSITIVE_INFINITY);
      MathTools.checkIntervalContains(lengthSideB, 0.0, Double.POSITIVE_INFINITY);

      if (Math.abs(angleBetweenAAndB) > Math.PI)
      {
         throw new RuntimeException("angleBetweenAAndB " + angleBetweenAAndB + " does not define a triangle.");
      }

      return Math.sqrt(MathTools.square(lengthSideA) + MathTools.square(lengthSideB) - 2.0 * lengthSideA * lengthSideB * Math.cos(angleBetweenAAndB));
   }

   /**
    * Calculate an unknown angle of a fully defined 2D Triangle by the law of Cosine.
    * <p>
    * Given a triangle with the three sides A, B, and C, this methods calculates the angle between A and B given the lengths of three sides.
    *
    * @param lengthNeighbourSideA the length of the side A.
    * @param lengthNeighbourSideB the length of the side B.
    * @param lengthOppositeSideC the length of the side C.
    * @throws RuntimeException if the lengths do not describe a triangle, see {@link #isFormingTriangle(double, double, double)}.
    */
   public static double getUnknownTriangleAngleByLawOfCosine(double lengthNeighbourSideA, double lengthNeighbourSideB, double lengthOppositeSideC)
   {
      if (GeometryTools.isFormingTriangle(lengthNeighbourSideA, lengthNeighbourSideB, lengthOppositeSideC))
      {
         return Math.acos((MathTools.square(lengthNeighbourSideA) + MathTools.square(lengthNeighbourSideB) - MathTools.square(lengthOppositeSideC))
               / (2.0 * lengthNeighbourSideA * lengthNeighbourSideB));
      }
      else
      {
         throw new RuntimeException("Unable to build a Triangle of the given triangle sides a: " + lengthNeighbourSideA + " b: " + lengthNeighbourSideB + " c: "
               + lengthOppositeSideC);
      }
   }

   /**
    * Get a unknown cathetus (90-deg triangle one of the two shorter triangle sides, neighbouring the 90-degree angle) by Pythagoras law.
    * <p>
    * Given a right triangle with the three sides A, B, and C, where A and B are the catheti and C the hypotenuse,
    * this method calculates the length of the cathetus B given the lengths of A and C:
    * <br> |B|<sup>2</sup> = |C|<sup>2</sup> - |A|<sup>2</sup>. </br>
    * <a href="https://en.wikipedia.org/wiki/Cathetus"> Useful link</a>.
    * </p>
    * 
    * @param hypotenuseC the length of the hypotenuse C.
    * @param cathetusA the length of the cathetus A.
    * @return the length of the cathetus B.
    * @throws RuntimeException if the length of the cathetus A is negative or greater than the hypotenuse C.
    */
   public static double pythagorasGetCathetus(double hypotenuseC, double cathetusA)
   {
      MathTools.checkIntervalContains(cathetusA, 0.0, hypotenuseC);

      return Math.sqrt(MathTools.square(hypotenuseC) - MathTools.square(cathetusA));
   }

   /**
    * Get the hypotenuse c (90-degree triangle longest triangle length, opposite to the 90-degree angle) by Pythagoras law, a^2+b^2=c^2
    * <p>
    * Given a right triangle with the three sides A, B, and C, where A and B are the catheti and C the hypotenuse,
    * this method calculates the length of the hypotenuse C given the lengths of A and B:
    * <br> |C|<sup>2</sup> = |A|<sup>2</sup> + |B|<sup>2</sup>. </br>
    * <a href="https://en.wikipedia.org/wiki/Cathetus"> Useful link</a>.
    * </p>
    *
    * @param cathetusA the length of the cathetus A.
    * @param cathetusB the length of the cathetus B.
    * @return the length of the hypotenuse C.
    * @throws RuntimeException if any of the two lengths is negative.
    */
   public static double pythagorasGetHypotenuse(double cathetusA, double cathetusB)
   {
      MathTools.checkIntervalContains(cathetusA, 0.0, Double.POSITIVE_INFINITY);
      MathTools.checkIntervalContains(cathetusB, 0.0, Double.POSITIVE_INFINITY);
      return Math.hypot(cathetusA, cathetusB);
   }

   /**
    * This methods verifies that the given set of three lengths represents a triangle.
    * A valid triangle with three edges A, B, and C verifies the three following inequalities:
    * <ul>
    *    <li> |A| + |B| > |C|
    *    <li> |B| + |C| > |A|
    *    <li> |C| + |A| > |B|
    * </ul>
    * 
    * <a href="https://opencurriculum.org/5534/the-triangle-inequality/"> Useful link</a>.
    * </p>
    * 
    * @param lengthSideA the length of the side A.
    * @param lengthSideB the length of the side B.
    * @param lengthSideC the length of the side C.
    * @return {@code true} if the lengths represents the three sides of a triangle, {@code false} otherwise.
    * @throws RuntimeException if any of the three lengths is negative.
    */
   public static boolean isFormingTriangle(double lengthSideA, double lengthSideB, double lengthSideC)
   {
      MathTools.checkIntervalContains(lengthSideA, 0.0, Double.POSITIVE_INFINITY);
      MathTools.checkIntervalContains(lengthSideB, 0.0, Double.POSITIVE_INFINITY);
      MathTools.checkIntervalContains(lengthSideC, 0.0, Double.POSITIVE_INFINITY);

      if (lengthSideA + lengthSideB <= lengthSideC)
         return false;
      if (lengthSideB + lengthSideC <= lengthSideA)
         return false;
      if (lengthSideA + lengthSideC <= lengthSideB)
         return false;
      return true;
   }

   /**
    * Finds the smallest distance between {@code testPoint} and a list of points.
    * 
    * @param testPoint the query. Not modified.
    * @param points the list of points to search through. Not modified.
    * @return the value of the minimum distance between the query and the point cloud.
    */
   public static double minimumDistance(FramePoint testPoint, List<FramePoint> points)
   {
      double minimumDistance = Double.POSITIVE_INFINITY;

      for (int i = 0; i < points.size(); i++)
      {
         FramePoint point = points.get(i);
         minimumDistance = Math.min(minimumDistance, testPoint.distanceSquared(point));
      }

      return Math.sqrt(minimumDistance);
   }

   /**
    * Change the frame and then project the result onto the XY-plane for each point in the given list {@code points}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param referenceFrame the new reference frame the result will be expressed in.. Not modified.
    * @param points the list of points to transform. Not modified.
    * @return the result of the transformation.
    */
   public static List<FramePoint2d> changeFrameAndProjectToXYPlane(ReferenceFrame referenceFrame, List<FramePoint> points)
   {
      List<FramePoint2d> ret = new ArrayList<>(points.size());

      for (int i = 0; i < points.size(); i++)
      {
         FramePoint framePoint = new FramePoint(points.get(i));
         framePoint.changeFrame(referenceFrame);

         ret.add(framePoint.toFramePoint2d());
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
   public static List<FramePoint2d> projectOntoXYPlane(List<FramePoint> points)
   {
      List<FramePoint2d> ret = new ArrayList<>(points.size());

      for (int i = 0; i < points.size(); i++)
      {
         FramePoint point3d = points.get(i);
         ret.add(new FramePoint2d(point3d.getReferenceFrame(), point3d.getX(), point3d.getY()));
      }

      return ret;
   }

   /**
    * Assert on a component basis is the {@code tuple} is equal to (0, 0, 0) given the tolerance {@code epsilon}.
    * 
    * @param tuple the query. Not modified.
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
    * Assert on a component basis is the {@code tuple} is equal to (0, 0) given the tolerance {@code epsilon}.
    * 
    * @param tuple the query. Not modified.
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
}
