package us.ihmc.avatar.factory;

import us.ihmc.commons.robotics.outputData.JointDesiredOutputListReadOnly;

public interface SimulatedHandOutputWriter
{
   void write(JointDesiredOutputListReadOnly jointDesiredOutputList);
}
