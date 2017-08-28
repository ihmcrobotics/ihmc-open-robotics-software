package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import us.ihmc.robotics.screwTheory.OneDoFJoint;

public interface LowLevelOneDoFJointDesiredDataHolderReadOnly
{
   public abstract LowLevelJointControlMode getJointControlMode(OneDoFJoint joint);

   public abstract boolean hasDataForJoint(OneDoFJoint joint);


   public abstract OneDoFJoint getOneDoFJoint(int index);

   public abstract LowLevelJointDataReadOnly getLowLevelJointData(OneDoFJoint joint);

   public abstract int getNumberOfJointsWithLowLevelData();

}