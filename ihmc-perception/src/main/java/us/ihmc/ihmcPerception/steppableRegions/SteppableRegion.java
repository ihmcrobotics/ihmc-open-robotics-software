package us.ihmc.ihmcPerception.steppableRegions;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2D;

import java.util.List;

public class SteppableRegion
{
   private final ConcavePolygon2D concaveHullInRegionFrame = new ConcavePolygon2D();
   private final ConvexPolygon2D convexHullInRegionFrame = new ConvexPolygon2D();

   private final RigidBodyTransform transformFromWorldToRegion = new RigidBodyTransform();

   public SteppableRegion(Point3DReadOnly origin,
                          Orientation3DReadOnly orientation,
                          List<? extends Point2DReadOnly> concaveHullVertices)
   {
      transformFromWorldToRegion.set(orientation, origin);
      concaveHullInRegionFrame.addVertices(Vertex2DSupplier.asVertex2DSupplier(concaveHullVertices));
      convexHullInRegionFrame.addVertices(Vertex2DSupplier.asVertex2DSupplier(concaveHullVertices));
      concaveHullInRegionFrame.update();
      convexHullInRegionFrame.update();
   }
}
