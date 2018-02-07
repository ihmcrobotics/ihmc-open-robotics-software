package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

public interface JointLoadStatusProvider
{
   public abstract boolean isJointLoadBearing(String jointName);
}
