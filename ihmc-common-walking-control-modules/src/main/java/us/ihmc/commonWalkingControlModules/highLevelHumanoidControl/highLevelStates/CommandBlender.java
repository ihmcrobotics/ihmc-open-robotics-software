package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;

public interface CommandBlender
{
   void computeAndUpdateJointControl(JointDesiredOutputBasics outputToPack,
                                     JointDesiredOutputReadOnly from,
                                     JointDesiredOutputReadOnly to,
                                     double alpha);
}
