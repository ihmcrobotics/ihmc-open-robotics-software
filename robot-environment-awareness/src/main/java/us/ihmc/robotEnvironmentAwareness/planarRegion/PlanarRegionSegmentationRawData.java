package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.robotEnvironmentAwareness.communication.packets.PlanarRegionSegmentationMessage;

public class PlanarRegionSegmentationRawData
{
   private final int regionId;
   private final Vector3D normal;
   private final Point3D origin;
   private final List<Point3D> pointCloud;
   private final Quaternion orientation;
   private final List<LineSegment2D> intersections;

   public PlanarRegionSegmentationRawData(int regionId, Vector3D32 normal, Point3D32 origin)
   {
      this(regionId, normal, origin, Collections.emptyList());
   }

   public PlanarRegionSegmentationRawData(int regionId, Vector3D normal, Point3D origin)
   {
      this(regionId, normal, origin, Collections.emptyList());
   }

   public PlanarRegionSegmentationRawData(PlanarRegionSegmentationNodeData nodeData)
   {
      this(nodeData.getId(), nodeData.getNormal(), nodeData.getOrigin(), nodeData.nodeStream(), null);
   }

   public PlanarRegionSegmentationRawData(PlanarRegionSegmentationMessage message)
   {
      this(message.getRegionId(), message.getNormal(), message.getOrigin(), Arrays.stream(message.getHitLocations()), null);
   }

   public PlanarRegionSegmentationRawData(int regionId, Vector3D32 normal, Point3D32 origin, List<Point3D32> pointCloud)
   {
      this(regionId, normal, origin, pointCloud.stream(), null);
   }

   public PlanarRegionSegmentationRawData(int regionId, Vector3D normal, Point3D origin, List<Point3D> pointCloud)
   {
      this(regionId, normal, origin, pointCloud.stream(), null);
   }

   private <T> PlanarRegionSegmentationRawData(int regionId, Vector3D32 normal, Point3D32 origin, Stream<T> streamToConvert, List<LineSegment2D> intersections)
   {
      this(regionId, new Vector3D(normal), new Point3D(origin), streamToConvert, intersections);
   }

   private <T> PlanarRegionSegmentationRawData(int regionId, Vector3D normal, Point3D origin, Stream<T> streamToConvert, List<LineSegment2D> intersections)
   {
      this.regionId = regionId;
      this.normal = new Vector3D(normal);
      this.origin = new Point3D(origin);
      this.pointCloud = toListOfPoint3d(streamToConvert);
      orientation = PolygonizerTools.getQuaternionFromZUpToVector(normal);
      if (intersections == null)
         this.intersections = new ArrayList<>();
      else
         this.intersections = intersections.stream().map(LineSegment2D::new).collect(Collectors.toList());
   }

   private static <T> List<Point3D> toListOfPoint3d(Stream<T> inputStream)
   {
      if (inputStream == null)
         return Collections.emptyList();
      else
         return inputStream.map(PlanarRegionSegmentationRawData::convertToPoint3d).collect(Collectors.toList());
   }

   private static <T> Point3D convertToPoint3d(T input)
   {
      if (input instanceof Point3D)
         return new Point3D((Point3D) input);
      
      if (input instanceof Point3D32)
         return new Point3D((Point3D32) input);

      if (input instanceof NormalOcTreeNode)
         return new Point3D(((NormalOcTreeNode) input).getHitLocationCopy());

      throw new RuntimeException("Unhandled type: " + input.getClass().getSimpleName());
   }

   public int getRegionId()
   {
      return regionId;
   }

   public int size()
   {
      return pointCloud.size();
   }

   public List<Point2D> getPointCloudInPlane()
   {
      return pointCloud.stream()
                       .map(this::toPointInPlane)
                       .collect(Collectors.toList());
   }

   private Point2D toPointInPlane(Point3D point3d)
   {
      return PolygonizerTools.toPointInPlane(point3d, origin, orientation);
   }

   public List<Point3D> getPointCloudInWorld()
   {
      return pointCloud;
   }

   public void getPoint(int index, Point3D pointToPack)
   {
      pointToPack.set(pointCloud.get(index));
   }

   public Point3D getOrigin()
   {
      return origin;
   }

   public Vector3D getNormal()
   {
      return normal;
   }

   public Quaternion getOrientation()
   {
      return orientation;
   }

   public Stream<Point3D> stream()
   {
      return pointCloud.stream();
   }

   public Stream<Point3D> parallelStream()
   {
      return pointCloud.parallelStream();
   }

   public RigidBodyTransform getTransformFromLocalToWorld()
   {
      return new RigidBodyTransform(orientation, origin);
   }

   public boolean hasIntersections()
   {
      return intersections != null;
   }

   public void addIntersections(List<LineSegment2D> intersectionsToAdd)
   {
      intersectionsToAdd.forEach(this::addIntersection);
   }

   public void addIntersection(LineSegment2D intersectionToAdd)
   {
      intersections.add(intersectionToAdd);
   }

   public List<LineSegment2D> getIntersections()
   {
      return intersections;
   }

   public PlanarRegionSegmentationMessage toMessage()
   {
      return new PlanarRegionSegmentationMessage(regionId, origin, normal, null, pointCloud);
   }

   public static PlanarRegionSegmentationMessage[] toMessageArray(List<PlanarRegionSegmentationRawData> rawData)
   {
      return rawData.stream()
                    .map(PlanarRegionSegmentationRawData::toMessage)
                    .toArray(PlanarRegionSegmentationMessage[]::new);
   }
}
