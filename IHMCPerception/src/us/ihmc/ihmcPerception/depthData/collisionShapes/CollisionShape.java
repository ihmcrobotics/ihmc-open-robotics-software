package us.ihmc.ihmcPerception.depthData.collisionShapes;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

public abstract class CollisionShape
{
   private final RigidBodyTransform pose;

   protected CollisionShape(RigidBodyTransform pose)
   {
      this.pose = pose;
   }

   public final RigidBodyTransform getPose()
   {
      return new RigidBodyTransform(pose);
   }

   public abstract boolean contains(Point3D point);
}
