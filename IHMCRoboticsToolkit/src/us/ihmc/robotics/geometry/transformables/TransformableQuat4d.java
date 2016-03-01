package us.ihmc.robotics.geometry.transformables;

import javax.vecmath.Quat4d;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public class TransformableQuat4d extends Quat4d implements TransformableDataObject
{
   private static final long serialVersionUID = -3751421971526302255L;
   private final Quat4d tempQuaternionForTransform = new Quat4d();

   public TransformableQuat4d(Quat4d tuple)
   {
      super(tuple);
   }

   public TransformableQuat4d()
   {
      super();
   }

   public TransformableQuat4d(double[] quaternion)
   {
      super(quaternion);
   }

   @Override
   public void transform(RigidBodyTransform transform3D)
   {
      transform3D.get(tempQuaternionForTransform);
      this.mul(tempQuaternionForTransform, this);

      normalizeAndLimitToPiMinusPi();
   }

   /**
    * Normalize the quaternion and also limits the described angle magnitude in [-Pi, Pi].
    * The latter prevents some controllers to poop their pants.
    */
   public void normalizeAndLimitToPiMinusPi()
   {
      if (normSquared() < 1.0e-7)
         setToZero();
      else
      {
         this.normalize();
         if (this.getW() < 0.0)
            this.negate();
      }
   }

   public double normSquared()
   {
      return this.x * this.x + this.y * this.y + this.z * this.z + this.w * this.w;
   }

   public void setToZero()
   {
      this.set(0.0, 0.0, 0.0, 1.0);
   }

}
