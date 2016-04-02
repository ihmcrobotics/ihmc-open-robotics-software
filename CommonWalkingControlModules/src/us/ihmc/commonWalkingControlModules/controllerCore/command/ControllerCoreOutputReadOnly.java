package us.ihmc.commonWalkingControlModules.controllerCore.command;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.screwTheory.RigidBody;

public interface ControllerCoreOutputReadOnly
{
   public abstract void getDesiredCenterOfPressure(FramePoint2d copToPack, RigidBody rigidBody);
   public abstract void getLinearMomentumRate(FrameVector linearMomentumRateToPack);
   public abstract RootJointDesiredConfigurationDataReadOnly getRootJointDesiredConfigurationData();
   public abstract LowLevelOneDoFJointDesiredDataHolderReadOnly getLowLevelOneDoFJointDesiredDataHolder();
}