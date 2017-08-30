package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

public interface StandPrepSetpoints
{
   double get(int jointIndex);

   double get(String jointName);
}
