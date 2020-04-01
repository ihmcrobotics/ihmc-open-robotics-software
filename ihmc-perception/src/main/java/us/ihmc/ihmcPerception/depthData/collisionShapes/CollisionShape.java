package us.ihmc.ihmcPerception.depthData.collisionShapes;

import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

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

   public abstract boolean contains(Point3DReadOnly point);

   public abstract Shape3DReadOnly getOrCreateShape3D();
}
