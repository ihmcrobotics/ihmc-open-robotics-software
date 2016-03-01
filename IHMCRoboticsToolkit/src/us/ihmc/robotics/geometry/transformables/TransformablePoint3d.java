package us.ihmc.robotics.geometry.transformables;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public class TransformablePoint3d extends Point3d implements TransformableDataObject
{
   private static final long serialVersionUID = 3215925974643446454L;

   public TransformablePoint3d(Tuple3d tuple)
   {
      super(tuple);
   }

   public TransformablePoint3d()
   {
      super();
   }

   public TransformablePoint3d(double x, double y, double z)
   {
      super(x, y, z);
   }

   public TransformablePoint3d(double[] position)
   {
      super(position);
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      transform.transform(this);
   }
}
