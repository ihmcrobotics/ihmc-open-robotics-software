package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

public interface StandPrepParameters
{
   double getSetpoint(int jointIndex);
   double getSetpoint(String jointName);

}
