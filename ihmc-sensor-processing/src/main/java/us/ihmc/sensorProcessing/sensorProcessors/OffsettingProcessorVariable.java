package us.ihmc.sensorProcessing.sensorProcessors;

import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType;
import us.ihmc.yoVariables.filters.ProcessingYoVariable;

public interface OffsettingProcessorVariable extends ProcessingYoVariable
{
   /**
    * Returns the value that is removed from the sensed signal. SO the result is equal to
    * output = input - offset
    * or
    * offset = input - output
    * @return
    */
   double getOffset();
}
