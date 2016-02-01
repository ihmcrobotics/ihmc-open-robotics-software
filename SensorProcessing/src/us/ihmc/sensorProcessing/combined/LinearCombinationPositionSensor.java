package us.ihmc.sensorProcessing.combined;

import java.util.HashMap;
import java.util.LinkedHashMap;

import us.ihmc.sensorProcessing.ProcessedPositionSensor;

public class LinearCombinationPositionSensor implements ProcessedPositionSensor
{
   private LinkedHashMap<ProcessedPositionSensor, Double> sensorsAndCoefficients = new LinkedHashMap<ProcessedPositionSensor, Double>();
   
   public LinearCombinationPositionSensor(HashMap<ProcessedPositionSensor, Double> sensorsAndCoefficients)
   {
      this.sensorsAndCoefficients.putAll(sensorsAndCoefficients);
   }

   public double getQ()
   {
      double ret = 0.0;
      for (ProcessedPositionSensor sensor : sensorsAndCoefficients.keySet())
      {
         ret += sensorsAndCoefficients.get(sensor) * sensor.getQ();
      }
      return ret;
   }
}
