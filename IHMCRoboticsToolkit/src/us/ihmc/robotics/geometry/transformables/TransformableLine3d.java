package us.ihmc.robotics.geometry.transformables;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public class TransformableLine3d implements TransformableDataObject
{
   private final TransformablePoint3d origin = new TransformablePoint3d();
   private final TransformableVector3d direction = new TransformableVector3d();

   @Override
   public void transform(RigidBodyTransform transform)
   {
      origin.transform(transform);
      direction.transform(transform);
   }

   public TransformablePoint3d getOrigin()
   {
      return origin;
   }

   public TransformableVector3d getDirection()
   {
      return direction;
   }

}
