package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

public interface WholeBodySetpointParameters
{
   double getSetpoint(int jointIndex);
   double getSetpoint(String jointName);
}
