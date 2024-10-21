package us.ihmc.euclid.referenceFrame.tools;

import us.ihmc.euclid.geometry.interfaces.Line3DBasics;
import us.ihmc.euclid.geometry.interfaces.Plane3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class EuclidFrameShapeMissingTools
{
   /**
    * Returns the minimum XY distance between a 3D point and an infinitely long 3D line defined by two
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
   public static double distanceXYFromPoint3DToLine3D(FramePoint3DReadOnly point, FramePoint3DReadOnly firstPointOnLine, FramePoint3DReadOnly secondPointOnLine)
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
   public static boolean isLineSegmentIntersectingPlane(FramePoint3DReadOnly pointOnPlane,
                                                        FrameVector3DReadOnly planeNormal,
                                                        FramePoint3DReadOnly lineSegmentStart,
                                                        FramePoint3DReadOnly lineSegmentEnd)
   {
      pointOnPlane.checkReferenceFrameMatch(planeNormal, lineSegmentStart, lineSegmentEnd);

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
   public static double distanceFromPointToPlane(FramePoint3DReadOnly point, FramePoint3DReadOnly pointOnPlane, FrameVector3DReadOnly planeNormal)
   {
      point.checkReferenceFrameMatch(pointOnPlane, planeNormal);

      return EuclidGeometryTools.distanceFromPoint3DToPlane3D(point, pointOnPlane, planeNormal);
   }

   /**
    * This methods calculates the line of intersection between two planes each defined by a point and a normal. The result is packed in a 3D point located on the intersection line and the 3D direction of the intersection.
    * Useful link 1  , useful link 2  .
    * Edge cases:
    * When the length of either the plane normal is below ONE_TRILLIONTH, this methods fails and returns false.
    * When the angle between the two planes is below ONE_MILLIONTH, this methods fails and returns false.
    * When there is no intersection, this method returns false and pointOnIntersectionToPack and intersectionDirectionToPack are set to Double. NaN.
    *
    * Uses
    * {@link EuclidGeometryTools#intersectionBetweenTwoPlane3Ds(Point3DReadOnly, Vector3DReadOnly, Point3DReadOnly, Vector3DReadOnly, Point3DBasics, Vector3DBasics)}
    *
    * @param plane1 first plane of which to compute the intersection
    * @param plane2 second plane of which to compute the inersection
    * @param intersectionToPack line of intersection between the two planes
    * @return success (not parallel)
    */
   public static boolean getIntersectionBetweenTwoPlanes(Plane3DReadOnly plane1, Plane3DReadOnly plane2, Line3DBasics intersectionToPack)
   {
      return EuclidGeometryTools.intersectionBetweenTwoPlane3Ds(plane1.getPoint(),
                                                                plane1.getNormal(),
                                                                plane2.getPoint(),
                                                                plane2.getNormal(),
                                                                intersectionToPack.getPoint(),
                                                                intersectionToPack.getDirection());
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
   public static FrameVector3D getPlaneNormalGivenThreePoints(FramePoint3DReadOnly firstPointOnPlane,
                                                              FramePoint3DReadOnly secondPointOnPlane,
                                                              FramePoint3DReadOnly thirdPointOnPlane)
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
   public static boolean getPlaneNormalGivenThreePoints(FramePoint3DReadOnly firstPointOnPlane,
                                                        FramePoint3DReadOnly secondPointOnPlane,
                                                        FramePoint3DReadOnly thirdPointOnPlane,
                                                        FrameVector3DBasics normalToPack)
   {
      firstPointOnPlane.checkReferenceFrameMatch(secondPointOnPlane, thirdPointOnPlane);

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
   public static FrameVector3D getPerpendicularVectorFromLineToPoint(FramePoint3DReadOnly point,
                                                                     FramePoint3DReadOnly firstPointOnLine,
                                                                     FramePoint3DReadOnly secondPointOnLine,
                                                                     FramePoint3DBasics orthogonalProjectionToPack)
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
   public static boolean getPerpendicularVectorFromLineToPoint(FramePoint3DReadOnly point,
                                                               FramePoint3DReadOnly firstPointOnLine,
                                                               FramePoint3DReadOnly secondPointOnLine,
                                                               FramePoint3DBasics orthogonalProjectionToPack,
                                                               FrameVector3DBasics perpendicularVectorToPack)
   {
      point.checkReferenceFrameMatch(firstPointOnLine, secondPointOnLine);
      perpendicularVectorToPack.setReferenceFrame(point.getReferenceFrame());

      if (orthogonalProjectionToPack == null)
      {
         return EuclidGeometryTools.perpendicularVector3DFromLine3DToPoint3D(point, firstPointOnLine, secondPointOnLine, null, perpendicularVectorToPack);
      }
      else
      {
         orthogonalProjectionToPack.setReferenceFrame(point.getReferenceFrame());
         return EuclidGeometryTools.perpendicularVector3DFromLine3DToPoint3D(point,
                                                                             firstPointOnLine,
                                                                             secondPointOnLine,
                                                                             orthogonalProjectionToPack,
                                                                             perpendicularVectorToPack);
      }
   }
}
