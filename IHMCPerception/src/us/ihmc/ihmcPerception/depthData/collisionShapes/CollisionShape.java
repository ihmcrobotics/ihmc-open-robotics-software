package us.ihmc.ihmcPerception.depthData.collisionShapes;

import javax.vecmath.Point3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;

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

   public abstract boolean contains(Point3d point);
}
