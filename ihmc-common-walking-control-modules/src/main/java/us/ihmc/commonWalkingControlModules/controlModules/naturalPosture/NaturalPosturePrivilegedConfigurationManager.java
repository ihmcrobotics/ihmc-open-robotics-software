package us.ihmc.commonWalkingControlModules.controlModules.naturalPosture;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OneDoFJointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPDGains;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class NaturalPosturePrivilegedConfigurationManager
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoPDGains pPoseSpinePitchGains = new YoPDGains("pPoseSpinePitch", registry);
   private final YoPDGains pPoseSpineRollGains = new YoPDGains("pPoseSpineRoll", registry);

   private final YoBoolean useSpineRollPitchJointCommands = new YoBoolean("useSpineRollPitchJointCommands", registry);

   private final FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();
   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();

   private final JointspaceAccelerationCommand jointspaceAccelerationCommand = new JointspaceAccelerationCommand();
   private final OneDoFJointFeedbackControlCommand spinePitchCommand = new OneDoFJointFeedbackControlCommand();
   private final OneDoFJointFeedbackControlCommand spineRollCommand = new OneDoFJointFeedbackControlCommand();

   private final FullHumanoidRobotModel fullRobotModel;

   public NaturalPosturePrivilegedConfigurationManager(FullHumanoidRobotModel fullRobotModel, YoRegistry parentRegistry)
   {
      this.fullRobotModel = fullRobotModel;

      OneDoFJointBasics spineRoll = fullRobotModel.getSpineJoint(SpineJointName.SPINE_ROLL);
      OneDoFJointBasics spinePitch = fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH);

      spinePitchCommand.clear();
      spinePitchCommand.setJoint(spinePitch);

      spineRollCommand.clear();
      spineRollCommand.setJoint(spineRoll);

      useSpineRollPitchJointCommands.set(true); // Can turn off joint limit for the spine when this is true.
      if (useSpineRollPitchJointCommands.getBooleanValue())
      {
         pPoseSpinePitchGains.setKp(25.0);
         pPoseSpineRollGains.setKp(25.0);
         pPoseSpinePitchGains.setZeta(0.7);
         pPoseSpineRollGains.setZeta(0.7);
         pPoseSpinePitchGains.createDerivativeGainUpdater(true);
         pPoseSpineRollGains.createDerivativeGainUpdater(true);
      }

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
   }

   public void compute()
   {
      feedbackControlCommandList.clear();
      inverseDynamicsCommandList.clear();

      computeSpineControlCommands();

      feedbackControlCommandList.addCommand(spinePitchCommand);
      feedbackControlCommandList.addCommand(spineRollCommand);
   }

   private void computeSpineControlCommands()
   {
      // Testing -- track spine joint x and y with highest priority
      if (useSpineRollPitchJointCommands.getBooleanValue())
      {
         OneDoFJointBasics spineRoll = fullRobotModel.getSpineJoint(SpineJointName.SPINE_ROLL);
         OneDoFJointBasics spinePitch = fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH);
         spinePitchCommand.setJoint(spinePitch);
         spinePitchCommand.setInverseDynamics(0.0, 0.0, 0.0);
         spinePitchCommand.setGains(pPoseSpinePitchGains);

         spineRollCommand.setJoint(spineRoll);
         spineRollCommand.setInverseDynamics(0.0, 0.0, 0.0);
         spineRollCommand.setGains(pPoseSpineRollGains);
      }

   }

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return feedbackControlCommandList;
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return inverseDynamicsCommandList;
   }
}
