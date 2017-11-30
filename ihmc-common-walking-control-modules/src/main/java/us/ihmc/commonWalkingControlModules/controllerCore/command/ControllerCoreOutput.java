package us.ihmc.commonWalkingControlModules.controllerCore.command;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;

public class ControllerCoreOutput implements ControllerCoreOutputReadOnly
{
   private final CenterOfPressureDataHolder centerOfPressureDataHolder;
   private final FrameVector3D linearMomentumRate = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final RootJointDesiredConfigurationData rootJointDesiredConfigurationData = new RootJointDesiredConfigurationData();
   private final JointDesiredOutputList lowLevelOneDoFJointDesiredDataHolder;

   public ControllerCoreOutput(CenterOfPressureDataHolder centerOfPressureDataHolder, OneDoFJoint[] controlledOneDoFJoints, JointDesiredOutputList lowLevelControllerOutput)
   {
      this.centerOfPressureDataHolder = centerOfPressureDataHolder;
      linearMomentumRate.setToNaN(ReferenceFrame.getWorldFrame());
      lowLevelOneDoFJointDesiredDataHolder = lowLevelControllerOutput;
   }

   public void setDesiredCenterOfPressure(FramePoint2D cop, RigidBody rigidBody)
   {
      centerOfPressureDataHolder.setCenterOfPressure(cop, rigidBody);
   }

   @Override
   public void getDesiredCenterOfPressure(FramePoint2D copToPack, RigidBody rigidBody)
   {
      centerOfPressureDataHolder.getCenterOfPressure(copToPack, rigidBody);
   }

   public void setLinearMomentumRate(FrameVector3D linearMomentumRate)
   {
      this.linearMomentumRate.set(linearMomentumRate);
   }

   public void setAndMatchFrameLinearMomentumRate(FrameVector3D linearMomentumRate)
   {
      this.linearMomentumRate.setIncludingFrame(linearMomentumRate);
      this.linearMomentumRate.changeFrame(ReferenceFrame.getWorldFrame());
   }

   @Override
   public void getLinearMomentumRate(FrameVector3D linearMomentumRateToPack)
   {
      linearMomentumRateToPack.setIncludingFrame(linearMomentumRate);
   }

   public void setRootJointDesiredConfigurationData(RootJointDesiredConfigurationDataReadOnly rootJointDesiredConfigurationData)
   {
      this.rootJointDesiredConfigurationData.set(rootJointDesiredConfigurationData);
   }

   @Override
   public RootJointDesiredConfigurationDataReadOnly getRootJointDesiredConfigurationData()
   {
      return rootJointDesiredConfigurationData;
   }

   public void setLowLevelOneDoFJointDesiredDataHolder(JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder)
   {
      this.lowLevelOneDoFJointDesiredDataHolder.overwriteWith(lowLevelOneDoFJointDesiredDataHolder);
   }

   @Override
   public JointDesiredOutputListReadOnly getLowLevelOneDoFJointDesiredDataHolder()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }
}
