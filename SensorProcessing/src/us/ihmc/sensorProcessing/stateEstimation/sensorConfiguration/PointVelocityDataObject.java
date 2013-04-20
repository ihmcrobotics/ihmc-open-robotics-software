package us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration;

import javax.vecmath.Vector3d;

public class PointVelocityDataObject
{
   private final Vector3d velocity = new Vector3d();
   public double velocityTrustFactor = 0.0;

   public PointVelocityDataObject()
   {
   }

   public void getVelocity(Vector3d velocityToPack)
   {
      velocityToPack.set(velocity);
   }

   public void setVelocity(Vector3d velocity)
   {
      this.velocity.set(velocity);
   }

   public double getVelocityTrustFactor()
   {
      return velocityTrustFactor;
   }

   public void setVelocityTrustFactor(double velocityTrustFactor)
   {
      this.velocityTrustFactor = velocityTrustFactor;
   }

   public void set(PointVelocityDataObject pointVelocityDataObject)
   {
      this.velocity.set(pointVelocityDataObject.velocity);
      this.velocityTrustFactor = pointVelocityDataObject.velocityTrustFactor;
   }
}
