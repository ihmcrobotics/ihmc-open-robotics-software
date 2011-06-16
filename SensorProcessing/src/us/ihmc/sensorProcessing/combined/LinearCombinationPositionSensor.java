package us.ihmc.sensorProcessing.combined;

import java.util.HashMap;

import us.ihmc.sensorProcessing.ProcessedPositionSensor;

public class LinearCombinationPositionSensor implements ProcessedPositionSensor
{
   private HashMap<ProcessedPositionSensor, Double> sensorsAndCoefficients = new HashMap<ProcessedPositionSensor, Double>();
   
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
