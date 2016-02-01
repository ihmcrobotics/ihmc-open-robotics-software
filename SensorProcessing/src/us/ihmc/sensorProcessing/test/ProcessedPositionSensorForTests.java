package us.ihmc.sensorProcessing.test;

import us.ihmc.sensorProcessing.ProcessedPositionSensor;

public class ProcessedPositionSensorForTests implements ProcessedPositionSensor
{
   private double q;

   public double getQ()
   {
      return q;
   }

   public void setQ(double q)
   {
      this.q = q;
   }
}
