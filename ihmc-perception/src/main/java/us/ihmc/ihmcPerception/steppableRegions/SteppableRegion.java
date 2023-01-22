package us.ihmc.ihmcPerception.steppableRegions;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2D;

public class SteppableRegion
{
   private final ConcavePolygon2D concaveHullInRegionFrame = new ConcavePolygon2D();
   private final ConvexPolygon2D convexHullInRegionFrame = new ConvexPolygon2D();
   private final Point2D centroid = new Point2D();

   private final RigidBodyTransform transformFromWorldToRegion = new RigidBodyTransform();
}
