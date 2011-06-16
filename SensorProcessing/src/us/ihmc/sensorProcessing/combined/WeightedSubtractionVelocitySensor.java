package us.ihmc.sensorProcessing.combined;

import java.util.HashMap;

import us.ihmc.sensorProcessing.ProcessedVelocitySensor;
import us.ihmc.utilities.math.MathTools;

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

   private static HashMap<ProcessedVelocitySensor, Double> createSensorsAndCoefficientsMap(ProcessedVelocitySensor sensor1, ProcessedVelocitySensor sensor2, double weight1, double weight2)
   {
      MathTools.checkIfInRange(weight1, 0.0, Double.POSITIVE_INFINITY);
      MathTools.checkIfInRange(weight2, 0.0, Double.POSITIVE_INFINITY);
      
      HashMap<ProcessedVelocitySensor, Double> ret = new HashMap<ProcessedVelocitySensor, Double>();
      ret.put(sensor1, weight1);
      ret.put(sensor2, -weight2);
      return ret;
   }

}
