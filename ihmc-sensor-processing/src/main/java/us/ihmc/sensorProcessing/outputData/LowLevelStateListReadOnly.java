package us.ihmc.sensorProcessing.outputData;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

public interface LowLevelStateListReadOnly
{
   boolean hasDataForJoint(OneDoFJointBasics joint);

   OneDoFJointBasics getOneDoFJoint(int index);

   LowLevelStateReadOnly getLowLevelState(OneDoFJointBasics joint);

   LowLevelStateReadOnly getLowLevelState(int index);

   int getNumberOfJointsWithLowLevelState();
}