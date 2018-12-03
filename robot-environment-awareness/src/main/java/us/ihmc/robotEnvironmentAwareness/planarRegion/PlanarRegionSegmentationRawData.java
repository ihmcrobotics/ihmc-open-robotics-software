package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
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
   private final List<Point3D> pointCloud;
   private final Quaternion orientation;
   private final List<LineSegment2D> intersections = new ArrayList<>();
   private final BoundingBox2D boundingBoxLocal = new BoundingBox2D();
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
                                           List<? extends LineSegment2DReadOnly> intersections)
   {
      this.regionId = regionId;
      this.normal = new Vector3D(normal);
      this.origin = new Point3D(origin);
      this.pointCloud = streamToConvert.map(Point3D::new).collect(Collectors.toList());
      orientation = PolygonizerTools.getQuaternionFromZUpToVector(normal);
      intersections.forEach(this::addIntersection);

      getPointCloudInPlane().forEach(boundingBoxLocal::updateToIncludePoint);
      getPointCloudInWorld().forEach(boundingBoxWorld::updateToIncludePoint);
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
      return pointCloud.stream().map(this::toPointInPlane).collect(Collectors.toList());
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

   public void addIntersections(List<? extends LineSegment2DReadOnly> intersectionsToAdd)
   {
      intersectionsToAdd.forEach(this::addIntersection);
   }

   public void addIntersection(LineSegment2DReadOnly intersectionToAdd)
   {
      intersections.add(new LineSegment2D(intersectionToAdd));

      boundingBoxLocal.updateToIncludePoint(intersectionToAdd.getFirstEndpoint());
      boundingBoxLocal.updateToIncludePoint(intersectionToAdd.getSecondEndpoint());

      LineSegment3D intersectionWorld = new LineSegment3D();
      intersectionWorld.getFirstEndpoint().set(PolygonizerTools.toPointInWorld(intersectionToAdd.getFirstEndpoint(), origin, orientation));
      intersectionWorld.getSecondEndpoint().set(PolygonizerTools.toPointInWorld(intersectionToAdd.getSecondEndpoint(), origin, orientation));

      boundingBoxWorld.updateToIncludePoint(intersectionWorld.getFirstEndpoint());
      boundingBoxWorld.updateToIncludePoint(intersectionWorld.getSecondEndpoint());
   }

   public List<LineSegment2D> getIntersections()
   {
      return intersections;
   }

   public BoundingBox3D getBoundingBoxInWorld()
   {
      return boundingBoxWorld;
   }

   public BoundingBox2D getBoundingBoxInPlane()
   {
      return boundingBoxLocal;
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
