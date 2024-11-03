package us.ihmc.avatar.factory;

import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;

public interface SimulatedHandOutputWriter
{
   void write(JointDesiredOutputListReadOnly jointDesiredOutputList);
}
