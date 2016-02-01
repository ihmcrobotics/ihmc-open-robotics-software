package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.SdfLoader.partNames.ArmJointName;


public interface ArmReferenceForPDControlModule
{
   public abstract void setDesiredJointPositionOnBothRobotSides(ArmJointName armJointName, double value);
   
}
