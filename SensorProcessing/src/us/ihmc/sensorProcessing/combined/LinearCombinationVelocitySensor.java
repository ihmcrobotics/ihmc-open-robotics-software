package us.ihmc.sensorProcessing.combined;

import java.util.HashMap;
import java.util.LinkedHashMap;

import us.ihmc.sensorProcessing.ProcessedVelocitySensor;

public class LinearCombinationVelocitySensor implements ProcessedVelocitySensor
{
   private LinkedHashMap<ProcessedVelocitySensor, Double> sensorsAndCoefficients = new LinkedHashMap<ProcessedVelocitySensor, Double>();
   
   public LinearCombinationVelocitySensor(HashMap<ProcessedVelocitySensor, Double> sensorsAndCoefficients)
   {      
      this.sensorsAndCoefficients.putAll(sensorsAndCoefficients);
   }

   public double getQd()
   {
      double ret = 0.0;
      for (ProcessedVelocitySensor sensor : sensorsAndCoefficients.keySet())
      {
         ret += sensorsAndCoefficients.get(sensor) * sensor.getQd();
      }
      return ret;
   }
}
