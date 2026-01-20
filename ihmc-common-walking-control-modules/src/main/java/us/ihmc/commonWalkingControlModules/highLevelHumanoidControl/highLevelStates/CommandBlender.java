package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;

public interface CommandBlender
{
   void initialize(JointDesiredOutputBasics from);

   void computeAndUpdateJointControl(JointDesiredOutputBasics outputToPack,
                                     JointDesiredOutputReadOnly from,
                                     JointDesiredOutputReadOnly to,
                                     double alpha);
}
