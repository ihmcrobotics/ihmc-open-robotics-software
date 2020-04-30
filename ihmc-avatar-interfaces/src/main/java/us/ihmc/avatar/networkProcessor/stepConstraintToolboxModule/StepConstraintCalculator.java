package us.ihmc.avatar.networkProcessor.stepConstraintToolboxModule;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxHelper;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

public class StepConstraintCalculator
{
   private final FloatingJointBasics rootJoint;
   private final OneDoFJointBasics[] oneDoFJoints;
   private final CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus();

   public StepConstraintCalculator(FloatingJointBasics rootJoint,
                                   OneDoFJointBasics[] oneDoFJoints)
   {
      this.rootJoint = rootJoint;
      this.oneDoFJoints = oneDoFJoints;
   }

   public void updateCapturabilityBasedStatus(CapturabilityBasedStatus capturabilityBasedStatus)
   {
         this.capturabilityBasedStatus.set(capturabilityBasedStatus);
   }

   public void updateRobotConfigurationData(RobotConfigurationData newConfigurationData)
   {
      KinematicsToolboxHelper.setRobotStateFromRobotConfigurationData(newConfigurationData, rootJoint, oneDoFJoints);
   }

   public void update()
   {
   }

}
