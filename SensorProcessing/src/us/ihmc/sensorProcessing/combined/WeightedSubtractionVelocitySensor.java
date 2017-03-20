package us.ihmc.sensorProcessing.combined;

import java.util.LinkedHashMap;

import us.ihmc.robotics.MathTools;
import us.ihmc.sensorProcessing.ProcessedVelocitySensor;

public class WeightedSubtractionVelocitySensor extends LinearCombinationVelocitySensor
{
   /**
    * weight1 * sensor1.getQ() - weight2 * sensor2.getQ() 
    * @param sensor1
    * @param sensor2
    * @param weight1
    * @param weight2
    */
   public WeightedSubtractionVelocitySensor(ProcessedVelocitySensor sensor1, ProcessedVelocitySensor sensor2, double weight1, double weight2)
   {
      super(createSensorsAndCoefficientsMap(sensor1, sensor2, weight1, weight2));
   }

   private static LinkedHashMap<ProcessedVelocitySensor, Double> createSensorsAndCoefficientsMap(ProcessedVelocitySensor sensor1, ProcessedVelocitySensor sensor2, double weight1, double weight2)
   {
      MathTools.checkIntervalContains(weight1, 0.0, Double.POSITIVE_INFINITY);
      MathTools.checkIntervalContains(weight2, 0.0, Double.POSITIVE_INFINITY);
      
      LinkedHashMap<ProcessedVelocitySensor, Double> ret = new LinkedHashMap<ProcessedVelocitySensor, Double>();
      ret.put(sensor1, weight1);
      ret.put(sensor2, -weight2);
      return ret;
   }

}
