package us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class PointPositionDataObject
{
   private Vector3d offsetFromJointInJointFrame = new Vector3d();
   private final Point3d position = new Point3d();

   public double covarianceScaling = 1.0;

   public PointPositionDataObject()
   {
   }

   public void getOffsetFromJointInJointFrame(Vector3d offsetFromJointInJointFrameToPack)
   {
      offsetFromJointInJointFrameToPack.set(this.offsetFromJointInJointFrame );
   }
   
   public void setOffsetFromJointInJointFrame(Vector3d offsetFromJointInJointFrame)
   {
      this.offsetFromJointInJointFrame.set(offsetFromJointInJointFrame);
   }  
   
   public void getPosition(Point3d positionToPack)
   {
      positionToPack.set(position);
   }

   public void setPosition(Point3d position)
   {
      this.position.set(position);
   }

   public double getCovarianceScaling()
   {
      return covarianceScaling;
   }

   public void setCovarianceScaling(double covarianceScaling)
   {
      this.covarianceScaling = covarianceScaling;
   }

   public void set(PointPositionDataObject pointPositionDataObject)
   {
      this.offsetFromJointInJointFrame.set(pointPositionDataObject.offsetFromJointInJointFrame);
      this.position.set(pointPositionDataObject.position);
      this.covarianceScaling = pointPositionDataObject.covarianceScaling;
   }
}
