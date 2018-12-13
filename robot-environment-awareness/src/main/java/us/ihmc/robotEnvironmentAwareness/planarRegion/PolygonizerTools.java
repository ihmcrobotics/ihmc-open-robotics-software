package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;

public class PolygonizerTools
{
   public static List<Point2D> toPointsInPlane(List<? extends Point3DReadOnly> pointsToTransform, Point3DReadOnly planeOrigin, Vector3DReadOnly planeNormal)
   {
      return toPointsInPlane(pointsToTransform, planeOrigin, getQuaternionFromZUpToVector(planeNormal));
   }

   public static List<Point2D> toPointsInPlane(List<? extends Point3DReadOnly> pointsToTransform, Point3DReadOnly planeOrigin,
                                               Orientation3DReadOnly planeOrientation)
   {
      return pointsToTransform.stream().map(point -> toPointInPlane(point, planeOrigin, planeOrientation)).collect(Collectors.toList());
   }

   public static Point2D toPointInPlane(Point3DReadOnly pointToTransform, Point3DReadOnly planeOrigin, Orientation3DReadOnly planeOrientation)
   {
      return toPointInPlane(pointToTransform.getX(), pointToTransform.getY(), pointToTransform.getZ(), planeOrigin, planeOrientation);
   }

   public static List<LineSegment2D> toLineSegmentsInPlane(List<? extends LineSegment3DReadOnly> lineSegmentsToTransform, Point3DReadOnly planeOrigin,
                                                           Vector3DReadOnly planeNormal)
   {
      return lineSegmentsToTransform.stream().map(lineSegment -> toLineSegmentInPlane(lineSegment, planeOrigin, planeNormal)).collect(Collectors.toList());
   }

   public static List<LineSegment2D> toLineSegmentsInPlane(List<? extends LineSegment3DReadOnly> lineSegmentsToTransform, Point3DReadOnly planeOrigin,
                                                           Orientation3DReadOnly planeOrientation)
   {
      return lineSegmentsToTransform.stream().map(lineSegment -> toLineSegmentInPlane(lineSegment, planeOrigin, planeOrientation)).collect(Collectors.toList());
   }

   public static LineSegment2D toLineSegmentInPlane(LineSegment3DReadOnly lineSegmentToTransform, Point3DReadOnly planeOrigin, Vector3DReadOnly planeNormal)
   {
      return toLineSegmentInPlane(lineSegmentToTransform, planeOrigin, getQuaternionFromZUpToVector(planeNormal));
   }

   public static LineSegment2D toLineSegmentInPlane(LineSegment3DReadOnly lineSegmentToTransform, Point3DReadOnly planeOrigin,
                                                    Orientation3DReadOnly planeOrientation)
   {
      Point2D lineSemgentStart = toPointInPlane(lineSegmentToTransform.getFirstEndpoint(), planeOrigin, planeOrientation);
      Point2D lineSemgentEnd = toPointInPlane(lineSegmentToTransform.getSecondEndpoint(), planeOrigin, planeOrientation);
      return new LineSegment2D(lineSemgentStart, lineSemgentEnd);
   }

   public static Point2D toPointInPlane(double xToTransform, double yToTransform, double zToTransform, Point3DReadOnly planeOrigin,
                                        Orientation3DReadOnly planeOrientation)
   {
      Point3D point3DInPlane = new Point3D(xToTransform, yToTransform, zToTransform);
      point3DInPlane.sub(planeOrigin);
      planeOrientation.inverseTransform(point3DInPlane);
      return new Point2D(point3DInPlane);
   }

   public static List<Point3D> toPointsInWorld(List<? extends Point2DReadOnly> pointsInPlane, Point3DReadOnly planeOrigin, Vector3DReadOnly planeNormal)
   {
      return toPointsInWorld(pointsInPlane, planeOrigin, getQuaternionFromZUpToVector(planeNormal));
   }

   public static List<Point3D> toPointsInWorld(List<? extends Point2DReadOnly> pointsInPlane, Point3DReadOnly planeOrigin,
                                               Orientation3DReadOnly planeOrientation)
   {
      return pointsInPlane.stream().map(point -> toPointInWorld(point, planeOrigin, planeOrientation)).collect(Collectors.toList());
   }

   public static Point3D toPointInWorld(Point2DReadOnly point2dReadOnly, Point3DReadOnly planeOrigin, Orientation3DReadOnly planeOrientation)
   {
      return toPointInWorld(point2dReadOnly.getX(), point2dReadOnly.getY(), planeOrigin, planeOrientation);
   }

   public static List<LineSegment3D> toLineSegmentsInWorld(List<? extends LineSegment2DReadOnly> lineSegmentsToTransform, Point3DReadOnly planeOrigin,
                                                           Vector3DReadOnly planeNormal)
   {
      return lineSegmentsToTransform.stream().map(lineSegment -> toLineSegmentInWorld(lineSegment, planeOrigin, planeNormal)).collect(Collectors.toList());
   }

   public static List<LineSegment3D> toLineSegmentsInWorld(List<? extends LineSegment2DReadOnly> lineSegmentsToTransform, Point3DReadOnly planeOrigin,
                                                           Orientation3DReadOnly planeOrientation)
   {
      return lineSegmentsToTransform.stream().map(lineSegment -> toLineSegmentInWorld(lineSegment, planeOrigin, planeOrientation)).collect(Collectors.toList());
   }

   public static LineSegment3D toLineSegmentInWorld(LineSegment2DReadOnly lineSegmentToTransform, Point3DReadOnly planeOrigin, Vector3DReadOnly planeNormal)
   {
      return toLineSegmentInWorld(lineSegmentToTransform, planeOrigin, getQuaternionFromZUpToVector(planeNormal));
   }

   public static LineSegment3D toLineSegmentInWorld(LineSegment2DReadOnly lineSegmentToTransform, Point3DReadOnly planeOrigin,
                                                    Orientation3DReadOnly planeOrientation)
   {
      Point3D lineSemgentStart = toPointInWorld(lineSegmentToTransform.getFirstEndpoint(), planeOrigin, planeOrientation);
      Point3D lineSemgentEnd = toPointInWorld(lineSegmentToTransform.getSecondEndpoint(), planeOrigin, planeOrientation);
      return new LineSegment3D(lineSemgentStart, lineSemgentEnd);
   }

   public static Point3D toPointInWorld(double xToTransform, double yToTransform, Point3DReadOnly planeOrigin, Orientation3DReadOnly planeOrientation)
   {
      Point3D pointInWorld = new Point3D(xToTransform, yToTransform, 0.0);
      planeOrientation.transform(pointInWorld);
      pointInWorld.add(planeOrigin);

      return pointInWorld;
   }

   public static Quaternion getQuaternionFromZUpToVector(Vector3DReadOnly normal)
   {
      return new Quaternion(EuclidGeometryTools.axisAngleFromZUpToVector3D(normal));
   }

   public static double computeEllipsoidVolume(Vector3DReadOnly radii)
   {
      return computeEllipsoidVolume(radii.getX(), radii.getY(), radii.getZ());
   }

   public static double computeEllipsoidVolume(double xRadius, double yRadius, double zRadius)
   {
      return 4.0 / 3.0 * Math.PI * xRadius * yRadius * zRadius;
   }
}
