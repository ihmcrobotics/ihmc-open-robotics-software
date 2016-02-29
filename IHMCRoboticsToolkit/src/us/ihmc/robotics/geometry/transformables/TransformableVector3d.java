package us.ihmc.robotics.geometry.transformables;

import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public class TransformableVector3d extends Vector3d implements TransformableDataObject
{
   private static final long serialVersionUID = 3215925974643446454L;

   public TransformableVector3d(Tuple3d tuple)
   {
      super(tuple);
   }

   public TransformableVector3d()
   {
      super();
   }

   public TransformableVector3d(double x, double y, double z)
   {
      super(x, y, z);
   }

   public TransformableVector3d(double[] position)
   {
      super(position);
   }

   @Override
   public void transform(RigidBodyTransform transform)
   {
      transform.transform(this);
   }
}
