package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.robotics.humanoidRobot.partNames.ArmJointName;


public interface ArmReferenceForPDControlModule
{
   public abstract void setDesiredJointPositionOnBothRobotSides(ArmJointName armJointName, double value);
   
}
