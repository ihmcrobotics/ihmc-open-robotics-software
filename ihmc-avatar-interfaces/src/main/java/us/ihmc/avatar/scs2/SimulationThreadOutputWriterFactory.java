package us.ihmc.avatar.scs2;

import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.sensorProcessing.outputData.SimulationThreadOutputWriter;

public interface SimulationThreadOutputWriterFactory
{
   SimulationThreadOutputWriter build(ControllerInput input, ControllerOutput output);
}
