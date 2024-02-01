package us.ihmc.perception.steppableRegions;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionNormal;
import us.ihmc.robotics.geometry.PlanarRegionOrigin;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2D;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DReadOnly;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.List;

public class SteppableRegion
{
   private int regionId = -1;

   private final double footYaw;

   private final ConcavePolygon2D concaveHullInRegionFrame = new ConcavePolygon2D();
   private final ConvexPolygon2D convexHullInRegionFrame = new ConvexPolygon2D();

   private final RigidBodyTransform transformFromWorldToRegion = new RigidBodyTransform();
   private final RigidBodyTransform transformFromRegionToWorld = new RigidBodyTransform();
   private final PlanarRegionOrigin origin = new PlanarRegionOrigin(transformFromRegionToWorld);
   private final PlanarRegionNormal normal = new PlanarRegionNormal(transformFromRegionToWorld);

   private HeightMapData localHeightMap;

   public SteppableRegion(RigidBodyTransformReadOnly transformFromRegionToWorld, List<? extends Point2DReadOnly> concaveHullVertices, double footYaw)
   {
      this.footYaw = footYaw;

      this.transformFromRegionToWorld.set(transformFromRegionToWorld);
      transformFromWorldToRegion.setAndInvert(transformFromRegionToWorld);

      concaveHullInRegionFrame.addVertices(Vertex2DSupplier.asVertex2DSupplier(concaveHullVertices));
      convexHullInRegionFrame.addVertices(Vertex2DSupplier.asVertex2DSupplier(concaveHullVertices));
      concaveHullInRegionFrame.update();
      convexHullInRegionFrame.update();
   }

   public ConvexPolygon2DReadOnly getConvexHullInRegionFrame()
   {
      return convexHullInRegionFrame;
   }

   public ConcavePolygon2DReadOnly getConcaveHullInRegionFrame()
   {
      return concaveHullInRegionFrame;
   }

   public void setLocalHeightMap(HeightMapData heightMapData)
   {
      this.localHeightMap = heightMapData;
   }

   public HeightMapData getLocalHeightMap()
   {
      return localHeightMap;
   }

   public Point3DReadOnly getRegionOrigin()
   {
      return origin;
   }

   public Vector3DReadOnly getRegionNormal()
   {
      return normal;
   }

   public Orientation3DReadOnly getRegionOrientation()
   {
      return transformFromWorldToRegion.getRotation();
   }

   public void setRegionId(int regionId)
   {
      this.regionId = regionId;
   }

   public int getRegionId()
   {
      return regionId;
   }

   public double getFootYaw()
   {
      return footYaw;
   }

   public void toPlanarRegion(PlanarRegion planarRegionToPack)
   {
      toPlanarRegion(this, planarRegionToPack);
   }

   public static void toPlanarRegion(SteppableRegion steppableRegion, PlanarRegion planarRegionToPack)
   {
      planarRegionToPack.set(steppableRegion.transformFromRegionToWorld, steppableRegion.getConcaveHullInRegionFrame());
      planarRegionToPack.setRegionId(steppableRegion.getRegionId());
   }
}

