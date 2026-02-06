package us.ihmc.sensorProcessing.outputData;

import us.ihmc.scs2.definition.controller.interfaces.Controller;

public interface SimulationThreadOutputWriter extends Controller, JointDesiredOutputWriter
{
   /**
    * Set the holder for the desired output values of all joints
    *
    * @param jointDesiredOutputList
    */
   void setJointDesiredOutputList(JointDesiredOutputListBasics jointDesiredOutputList);
}
