package us.ihmc.sensorProcessing.test;

import us.ihmc.sensorProcessing.ProcessedVelocitySensor;

public class ProcessedVelocitySensorForTests implements ProcessedVelocitySensor
{
   private double qd;

   public double getQd()
   {
      return qd;
   }

   public void setQd(double qd)
   {
      this.qd = qd;
   }
}
