package us.ihmc.avatar.factory;

import us.ihmc.robotics.outputData.JointDesiredOutputListReadOnly;

public interface SimulatedHandOutputWriter
{
   void write(JointDesiredOutputListReadOnly jointDesiredOutputList);
}
