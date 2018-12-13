package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.robotEnvironmentAwareness.communication.converters.REAPlanarRegionsConverter;
import us.ihmc.robotEnvironmentAwareness.communication.packets.PlanarRegionSegmentationMessage;

public class PlanarRegionSegmentationRawData
{
   private final int regionId;
   private final Vector3D normal;
   private final Point3D origin;
   private final Quaternion orientation;
   private final List<Point3D> pointCloud;
   private final List<LineSegment3D> intersections = new ArrayList<>();
   private final BoundingBox3D boundingBoxWorld = new BoundingBox3D();

   public PlanarRegionSegmentationRawData(int regionId, Vector3DReadOnly normal, Point3DReadOnly origin)
   {
      this(regionId, normal, origin, Collections.emptyList());
   }

   public PlanarRegionSegmentationRawData(PlanarRegionSegmentationNodeData nodeData)
   {
      this(nodeData.getId(), nodeData.getNormal(), nodeData.getOrigin(), nodeData.nodeStream().map(NormalOcTreeNode::getHitLocationCopy), null);
   }

   public PlanarRegionSegmentationRawData(PlanarRegionSegmentationMessage message)
   {
      this(message.getRegionId(), message.getNormal(), message.getOrigin(), Arrays.stream(message.getHitLocations()), null);
   }

   public PlanarRegionSegmentationRawData(int regionId, Vector3DReadOnly normal, Point3DReadOnly origin, List<? extends Point3DReadOnly> pointCloud)
   {
      this(regionId, normal, origin, pointCloud.stream(), null);
   }

   private PlanarRegionSegmentationRawData(int regionId, Vector3DReadOnly normal, Point3DReadOnly origin, Stream<? extends Point3DReadOnly> streamToConvert,
                                           List<? extends LineSegment3DReadOnly> intersections)
   {
      this.regionId = regionId;
      this.normal = new Vector3D(normal);
      this.origin = new Point3D(origin);
      this.pointCloud = streamToConvert.map(Point3D::new).collect(Collectors.toList());
      orientation = PolygonizerTools.getQuaternionFromZUpToVector(normal);
      if (intersections != null)
         intersections.forEach(this::addIntersection);

      getPointCloudInWorld().forEach(boundingBoxWorld::updateToIncludePoint);
   }

   public void addIntersections(List<? extends LineSegment3DReadOnly> intersectionsToAdd)
   {
      intersectionsToAdd.forEach(this::addIntersection);
   }

   public void addIntersection(LineSegment3DReadOnly intersectionToAdd)
   {
      intersections.add(new LineSegment3D(intersectionToAdd));
      boundingBoxWorld.updateToIncludePoint(intersectionToAdd.getFirstEndpoint());
      boundingBoxWorld.updateToIncludePoint(intersectionToAdd.getSecondEndpoint());
   }

   public void clearIntersections()
   {
      intersections.clear();
      boundingBoxWorld.setToNaN();
      pointCloud.forEach(boundingBoxWorld::updateToIncludePoint);
   }

   public void translate(Tuple3DReadOnly translation)
   {
      origin.add(translation);
      pointCloud.stream().forEach(point -> point.add(translation));
      intersections.stream().forEach(segment -> segment.translate(translation));
      Point3D newMin = new Point3D(boundingBoxWorld.getMinPoint());
      Point3D newMax = new Point3D(boundingBoxWorld.getMaxPoint());
      newMin.add(translation);
      newMax.add(translation);
      boundingBoxWorld.set(newMin, newMax);
   }

   public int getRegionId()
   {
      return regionId;
   }

   public int size()
   {
      return pointCloud.size();
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

   public RigidBodyTransform getTransformFromLocalToWorld()
   {
      return new RigidBodyTransform(orientation, origin);
   }

   public BoundingBox3D getBoundingBoxInWorld()
   {
      return boundingBoxWorld;
   }

   public List<Point3D> getPointCloudInWorld()
   {
      return pointCloud;
   }

   public List<Point2D> getPointCloudInPlane()
   {
      return pointCloud.stream().map(this::toPointInPlane).collect(Collectors.toList());
   }

   private Point2D toPointInPlane(Point3D point3d)
   {
      return PolygonizerTools.toPointInPlane(point3d, origin, orientation);
   }

   public void getPoint(int index, Point3D pointToPack)
   {
      pointToPack.set(pointCloud.get(index));
   }

   public Stream<Point3D> stream()
   {
      return pointCloud.stream();
   }

   public Stream<Point3D> parallelStream()
   {
      return pointCloud.parallelStream();
   }

   public boolean hasIntersections()
   {
      return intersections != null;
   }

   public List<LineSegment3D> getIntersectionsInWorld()
   {
      return intersections;
   }

   public List<LineSegment2D> getIntersectionsInPlane()
   {
      return intersections.stream().map(this::toLineSegmentInPlane).collect(Collectors.toList());
   }

   private LineSegment2D toLineSegmentInPlane(LineSegment3D lineSegmentInWorld)
   {
      return PolygonizerTools.toLineSegmentInPlane(lineSegmentInWorld, origin, orientation);
   }

   public PlanarRegionSegmentationMessage toMessage()
   {
      return REAPlanarRegionsConverter.createPlanarRegionSegmentationMessage(regionId, origin, normal, null, pointCloud);
   }

   public static PlanarRegionSegmentationMessage[] toMessageArray(List<PlanarRegionSegmentationRawData> rawData)
   {
      return rawData.stream().map(PlanarRegionSegmentationRawData::toMessage).toArray(PlanarRegionSegmentationMessage[]::new);
   }
}
