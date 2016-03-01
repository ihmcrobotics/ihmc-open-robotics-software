package us.ihmc.robotics.geometry.transformables;

import javax.vecmath.Quat4d;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public class TransformableQuat4d extends Quat4d implements TransformableDataObject<TransformableQuat4d>
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
   public void applyTransform(RigidBodyTransform transform3D)
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

   @Override
   public void setToZero()
   {
      this.set(0.0, 0.0, 0.0, 1.0);
   }

   @Override
   public void set(TransformableQuat4d other)
   {
      super.set(other);
   }

   @Override
   public void setToNaN()
   {
      super.set(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
   }

   @Override
   public boolean containsNaN()
   {
      if (Double.isNaN(getW())) return true;
      if (Double.isNaN(getX())) return true;
      if (Double.isNaN(getY())) return true;
      if (Double.isNaN(getZ())) return true;

      return false;
   }

   @Override
   public boolean epsilonEquals(TransformableQuat4d other, double epsilon)
   {
      //TODO: Not sure if this is the best for epsilonEquals. Also, not sure how to handle the negative equal cases...
      
      if (Math.abs(getW() - other.getW()) > epsilon) return false;
      if (Math.abs(getX() - other.getX()) > epsilon) return false;
      if (Math.abs(getY() - other.getY()) > epsilon) return false;
      if (Math.abs(getZ() - other.getZ()) > epsilon) return false;
      
      return true;
   }

}
