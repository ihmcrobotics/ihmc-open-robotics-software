package us.ihmc.sensorProcessing.outputData;

import us.ihmc.robotics.screwTheory.OneDoFJoint;

public interface LowLevelStateListReadOnly
{
   boolean hasDataForJoint(OneDoFJoint joint);

   OneDoFJoint getOneDoFJoint(int index);

   LowLevelStateReadOnly getLowLevelState(OneDoFJoint joint);

   LowLevelStateReadOnly getLowLevelState(int index);

   int getNumberOfJointsWithLowLevelState();
}