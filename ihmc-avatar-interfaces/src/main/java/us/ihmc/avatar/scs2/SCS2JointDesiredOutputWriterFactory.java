package us.ihmc.avatar.scs2;

import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;

public interface SCS2JointDesiredOutputWriterFactory
{
   JointDesiredOutputWriter build(ControllerInput input, ControllerOutput output);
}
