package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion;

public class PolygonizerTools
{
   public static List<Point2D> toPointsInPlane(List<Point3D> pointsToTransform, Point3D planeOrigin, Vector3D planeNormal)
   {
      return toPointsInPlane(pointsToTransform, planeOrigin, getQuaternionFromZUpToVector(planeNormal));
   }

   public static List<Point2D> toPointsInPlane(List<Point3D> pointsToTransform, Point3D planeOrigin, Quaternion planeOrientation)
   {
      return pointsToTransform.stream().map(point -> toPointInPlane(point, planeOrigin, planeOrientation)).collect(Collectors.toList());
   }

   public static Point2D toPointInPlane(Point3D pointToTransform, Point3D planeOrigin, Quaternion planeOrientation)
   {
      return toPointInPlane(pointToTransform.getX(), pointToTransform.getY(), pointToTransform.getZ(), planeOrigin, planeOrientation);
   }

   public static Point2D toPointInPlane(Point3D32 pointToTransform, Point3D planeOrigin, Quaternion planeOrientation)
   {
      return toPointInPlane(pointToTransform.getX(), pointToTransform.getY(), pointToTransform.getZ(), planeOrigin, planeOrientation);
   }

   public static List<LineSegment2D> toLineSegmentsInPlane(List<LineSegment3D> lineSegmentsToTransform, Point3D planeOrigin, Vector3D planeNormal)
   {
      return lineSegmentsToTransform.stream().map(lineSegment -> toLineSegmentInPlane(lineSegment, planeOrigin, planeNormal)).collect(Collectors.toList());
   }

   public static List<LineSegment2D> toLineSegmentsInPlane(List<LineSegment3D> lineSegmentsToTransform, Point3D planeOrigin, Quaternion planeOrientation)
   {
      return lineSegmentsToTransform.stream().map(lineSegment -> toLineSegmentInPlane(lineSegment, planeOrigin, planeOrientation)).collect(Collectors.toList());
   }

   public static LineSegment2D toLineSegmentInPlane(LineSegment3D lineSegmentToTransform, Point3D planeOrigin, Vector3D planeNormal)
   {
      return toLineSegmentInPlane(lineSegmentToTransform, planeOrigin, getQuaternionFromZUpToVector(planeNormal));
   }

   public static LineSegment2D toLineSegmentInPlane(LineSegment3D lineSegmentToTransform, Point3D planeOrigin, Quaternion planeOrientation)
   {
      Point2D lineSemgentStart = toPointInPlane(lineSegmentToTransform.getFirstEndpoint(), planeOrigin, planeOrientation);
      Point2D lineSemgentEnd = toPointInPlane(lineSegmentToTransform.getSecondEndpoint(), planeOrigin, planeOrientation);
      return new LineSegment2D(lineSemgentStart, lineSemgentEnd);
   }

   public static Point2D toPointInPlane(double xToTransform, double yToTransform, double zToTransform, Point3D planeOrigin, Quaternion planeOrientation)
   {
      Point2D pointInPlane = new Point2D();

      double qx = -planeOrientation.getX();
      double qy = -planeOrientation.getY();
      double qz = -planeOrientation.getZ();
      double qs = planeOrientation.getS();

      // t = 2.0 * cross(q.xyz, v);
      // v' = v + q.s * t + cross(q.xyz, t);
      double x = xToTransform - planeOrigin.getX();
      double y = yToTransform - planeOrigin.getY();
      double z = zToTransform - planeOrigin.getZ();

      double crossX = 2.0 * (qy * z - qz * y);
      double crossY = 2.0 * (qz * x - qx * z);
      double crossZ = 2.0 * (qx * y - qy * x);

      double crossCrossX = qy * crossZ - qz * crossY;
      double crossCrossY = qz * crossX - qx * crossZ;

      pointInPlane.setX(x + qs * crossX + crossCrossX);
      pointInPlane.setY(y + qs * crossY + crossCrossY);

      return pointInPlane;
   }

   public static List<Point3D> toPointsInWorld(List<Point2D> pointsInPlane, Point3D planeOrigin, Vector3D planeNormal)
   {
      return toPointsInWorld(pointsInPlane, planeOrigin, getQuaternionFromZUpToVector(planeNormal));
   }

   public static List<Point3D> toPointsInWorld(List<Point2D> pointsInPlane, Point3D planeOrigin, Quaternion planeOrientation)
   {
      return pointsInPlane.stream().map(point -> toPointInWorld(point, planeOrigin, planeOrientation)).collect(Collectors.toList());
   }

   public static Point3D toPointInWorld(Point2DReadOnly point2dReadOnly, Point3D planeOrigin, Quaternion planeOrientation)
   {
      return toPointInWorld(point2dReadOnly.getX(), point2dReadOnly.getY(), planeOrigin, planeOrientation);
   }

   public static List<LineSegment3D> toLineSegmentsInWorld(List<LineSegment2D> lineSegmentsToTransform, Point3D planeOrigin, Vector3D planeNormal)
   {
      return lineSegmentsToTransform.stream().map(lineSegment -> toLineSegmentInWorld(lineSegment, planeOrigin, planeNormal)).collect(Collectors.toList());
   }

   public static List<LineSegment3D> toLineSegmentsInWorld(List<LineSegment2D> lineSegmentsToTransform, Point3D planeOrigin, Quaternion planeOrientation)
   {
      return lineSegmentsToTransform.stream().map(lineSegment -> toLineSegmentInWorld(lineSegment, planeOrigin, planeOrientation)).collect(Collectors.toList());
   }

   public static LineSegment3D toLineSegmentInWorld(LineSegment2D lineSegmentToTransform, Point3D planeOrigin, Vector3D planeNormal)
   {
      return toLineSegmentInWorld(lineSegmentToTransform, planeOrigin, getQuaternionFromZUpToVector(planeNormal));
   }

   public static LineSegment3D toLineSegmentInWorld(LineSegment2D lineSegmentToTransform, Point3D planeOrigin, Quaternion planeOrientation)
   {
      Point3D lineSemgentStart = toPointInWorld(lineSegmentToTransform.getFirstEndpoint(), planeOrigin, planeOrientation);
      Point3D lineSemgentEnd = toPointInWorld(lineSegmentToTransform.getSecondEndpoint(), planeOrigin, planeOrientation);
      return new LineSegment3D(lineSemgentStart, lineSemgentEnd);
   }

   public static Point3D toPointInWorld(double xToTransform, double yToTransform, Point3D planeOrigin, Quaternion planeOrientation)
   {
      Point3D pointInWorld = new Point3D();

      double qx = planeOrientation.getX();
      double qy = planeOrientation.getY();
      double qz = planeOrientation.getZ();
      double qs = planeOrientation.getS();

      // t = 2.0 * cross(q.xyz, v);
      // v' = v + q.s * t + cross(q.xyz, t);
      double x = xToTransform;
      double y = yToTransform;
      double z = 0.0;

      double crossX = 2.0 * (qy * z - qz * y);
      double crossY = 2.0 * (qz * x - qx * z);
      double crossZ = 2.0 * (qx * y - qy * x);

      double crossCrossX = qy * crossZ - qz * crossY;
      double crossCrossY = qz * crossX - qx * crossZ;
      double crossCrossZ = qx * crossY - qy * crossX;

      pointInWorld.setX(x + qs * crossX + crossCrossX);
      pointInWorld.setY(y + qs * crossY + crossCrossY);
      pointInWorld.setZ(z + qs * crossZ + crossCrossZ);
      pointInWorld.add(planeOrigin);

      return pointInWorld;
   }

   public static Quaternion getRotationBasedOnNormal(Vector3D32 normal)
   {
      return getQuaternionFromZUpToVector(new Vector3D(normal));
   }

   public static Quaternion getQuaternionFromZUpToVector(Vector3D normal)
   {
      Quaternion orientation = new Quaternion();
      orientation.set(EuclidGeometryTools.axisAngleFromZUpToVector3D(normal));
      return orientation;
   }

   public static double computeEllipsoidVolume(Vector3D radii)
   {
      return computeEllipsoidVolume(radii.getX(), radii.getY(), radii.getZ());
   }

   public static double computeEllipsoidVolume(double xRadius, double yRadius, double zRadius)
   {
      return 4.0 / 3.0 * Math.PI * xRadius * yRadius * zRadius;
   }
}
