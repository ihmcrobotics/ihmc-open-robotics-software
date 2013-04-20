package us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration;

import javax.vecmath.Point3d;

public class PointPositionDataObject
{
   private final Point3d position = new Point3d();

   public double positionTrustFactor = 0.0;

   public PointPositionDataObject()
   {
   }

   public void getPosition(Point3d positionToPack)
   {
      positionToPack.set(position);
   }

   public void setPosition(Point3d position)
   {
      this.position.set(position);
   }

   public double getPositionTrustFactor()
   {
      return positionTrustFactor;
   }

   public void setPositionTrustFactor(double positionTrustFactor)
   {
      this.positionTrustFactor = positionTrustFactor;
   }

   public void set(PointPositionDataObject pointPositionDataObject)
   {
      this.position.set(pointPositionDataObject.position);
      this.positionTrustFactor = pointPositionDataObject.positionTrustFactor;
   }
}
