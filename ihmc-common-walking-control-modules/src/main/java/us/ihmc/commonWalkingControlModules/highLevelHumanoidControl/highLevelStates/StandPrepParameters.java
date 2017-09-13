package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

public interface StandPrepParameters
{
   double getSetpoint(int jointIndex);
   double getSetpoint(String jointName);

}
