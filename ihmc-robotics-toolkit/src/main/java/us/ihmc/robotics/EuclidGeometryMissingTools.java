package us.ihmc.robotics;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class EuclidGeometryMissingTools
{
   public static double getZOnPlane(Point3DReadOnly pointOnPlane, Vector3DReadOnly planeNormal, double pointX, double pointY)
   {
      // The three components of the plane origin
      double x0 = pointOnPlane.getX();
      double y0 = pointOnPlane.getY();
      double z0 = pointOnPlane.getZ();
      // The three components of the plane normal
      double a = planeNormal.getX();
      double b = planeNormal.getY();
      double c = planeNormal.getZ();

      // Given the plane equation: a*x + b*y + c*z + d = 0, with d = -(a*x0 + b*y0 + c*z0), we find z:
      double z = a / c * (x0 - pointX) + b / c * (y0 - pointY) + z0;
      return z;
   }

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
    * Finds the intersection of two bounding boxes defined by a bounding box Allocates a new
    * BoundingBox2D. TODO: Check, Unit test, move where BoundingBox union is
    *
    * @param a
    * @param b
    * @return the intersection bounding box, or null if no intersection
    */
   public static BoundingBox2D computeIntersectionOfTwoBoundingBoxes(BoundingBox2DReadOnly a, BoundingBox2DReadOnly b)
   {
      double maxX = Math.min(a.getMaxX(), b.getMaxX());
      double maxY = Math.min(a.getMaxY(), b.getMaxY());
      double minX = Math.max(a.getMinX(), b.getMinX());
      double minY = Math.max(a.getMinY(), b.getMinY());

      if ((maxX <= minX) || (maxY <= minY))
         return null;

      return new BoundingBox2D(minX, minY, maxX, maxY);
   }

   /**
    * Finds the intersection of two bounding boxes defined by a bounding box Allocates a new boundingBox3D.
    *
    * @param a
    * @param b
    * @return the intersection bounding box, or null if no intersection
    */
   public static BoundingBox3D computeIntersectionOfTwoBoundingBoxes(BoundingBox3DReadOnly a, BoundingBox3DReadOnly b)
   {
      double maxX = Math.min(a.getMaxX(), b.getMaxX());
      double maxY = Math.min(a.getMaxY(), b.getMaxY());
      double maxZ = Math.min(a.getMaxZ(), b.getMaxZ());

      double minX = Math.max(a.getMinX(), b.getMinX());
      double minY = Math.max(a.getMinY(), b.getMinY());
      double minZ = Math.max(a.getMinZ(), b.getMinZ());

      if ((maxX <= minX) || (maxY <= minY) || (maxZ <= minZ))
         return null;

      return new BoundingBox3D(minX, minY, minZ, maxX, maxY, maxZ);
   }

   public static double computeBoundingBoxVolume3D(BoundingBox3DReadOnly boundingBox)
   {
      return Math.abs(boundingBox.getMaxX() - boundingBox.getMinX())
           * Math.abs(boundingBox.getMaxY() - boundingBox.getMinY())
           * Math.abs(boundingBox.getMaxZ() - boundingBox.getMinZ());
   }
}
