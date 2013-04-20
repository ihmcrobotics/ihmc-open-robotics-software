package us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration;

import javax.vecmath.Vector3d;

public class PointVelocityDataObject
{
   private Vector3d offsetFromJointInJointFrame = new Vector3d();
   private final Vector3d velocity = new Vector3d();

   public double covarianceScaling = 1.0;

   public PointVelocityDataObject()
   {
   }

   public void getOffsetFromJointInJointFrame(Vector3d offsetFromJointInJointFrameToPack)
   {
      offsetFromJointInJointFrameToPack.set(this.offsetFromJointInJointFrame);
   }

   public void setOffsetFromJointInJointFrame(Vector3d offsetFromJointInJointFrame)
   {
      this.offsetFromJointInJointFrame.set(offsetFromJointInJointFrame);
   }

   public void getVelocity(Vector3d velocityToPack)
   {
      velocityToPack.set(velocity);
   }

   public void setVelocity(Vector3d velocity)
   {
      this.velocity.set(velocity);
   }

   public double getCovarianceScaling()
   {
      return covarianceScaling;
   }

   public void setCovarianceScaling(double covarianceScaling)
   {
      this.covarianceScaling = covarianceScaling;
   }

   public void set(PointVelocityDataObject pointVelocityDataObject)
   {
      this.offsetFromJointInJointFrame.set(pointVelocityDataObject.offsetFromJointInJointFrame);
      this.velocity.set(pointVelocityDataObject.velocity);
      this.covarianceScaling = pointVelocityDataObject.covarianceScaling;
   }
}
