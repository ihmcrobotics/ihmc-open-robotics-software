package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import us.ihmc.robotics.screwTheory.OneDoFJoint;

public interface LowLevelOneDoFJointDesiredDataHolderReadOnly
{
   public abstract LowLevelJointControlMode getJointControlMode(OneDoFJoint joint);

   public abstract double getDesiredJointTorque(OneDoFJoint joint);

   public abstract double getDesiredJointPosition(OneDoFJoint joint);

   public abstract double getDesiredJointVelocity(OneDoFJoint joint);

   public abstract double getDesiredJointAcceleration(OneDoFJoint joint);

   public abstract double getDesiredJointCurrent(OneDoFJoint joint);

   public abstract boolean pollResetJointIntegrators(OneDoFJoint joint);

   public abstract boolean peekResetJointIntegrators(OneDoFJoint joint);

   public abstract boolean hasDataForJoint(OneDoFJoint joint);

   public abstract boolean hasControlModeForJoint(OneDoFJoint joint);

   public abstract boolean hasDesiredTorqueForJoint(OneDoFJoint joint);

   public abstract boolean hasDesiredPositionForJoint(OneDoFJoint joint);

   public abstract boolean hasDesiredVelocityForJoint(OneDoFJoint joint);

   public abstract boolean hasDesiredAcceleration(OneDoFJoint joint);

   public abstract boolean hasDesiredCurrentForJoint(OneDoFJoint joint);

   public abstract OneDoFJoint getOneDoFJoint(int index);

   public abstract LowLevelJointDataReadOnly getLowLevelJointData(OneDoFJoint joint);

   public abstract int getNumberOfJointsWithLowLevelData();

}