package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;


public interface ArmReferenceForPDControlModule
{
   public abstract void setDesiredJointPositionOnBothRobotSides(ArmJointName armJointName, double value);
   
}
