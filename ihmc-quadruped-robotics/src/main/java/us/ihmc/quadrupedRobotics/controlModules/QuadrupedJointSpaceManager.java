package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.JointLimitEnforcementCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedJointSpaceManager
{
   private static final double POSITION_LIMIT_DAMPING = 10.0;
   private static final double POSITION_LIMIT_STIFFNESS = 100.0;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final OneDoFJointBasics[] controlledJoints;

   private final JointLimitEnforcementCommand jointLimitEnforcementCommand = new JointLimitEnforcementCommand();

   private final YoDouble jointPositionLimitDamping = new YoDouble("jointPositionLimitDamping", registry);
   private final YoDouble jointPositionLimitStiffness = new YoDouble("jointPositionLimitStiffness", registry);

   public QuadrupedJointSpaceManager(QuadrupedControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      controlledJoints = controllerToolbox.getFullRobotModel().getControllableOneDoFJoints();

      jointPositionLimitDamping.set(POSITION_LIMIT_DAMPING);
      jointPositionLimitStiffness.set(POSITION_LIMIT_STIFFNESS);

      parentRegistry.addChild(registry);
   }

   public void compute()
   {
      jointLimitEnforcementCommand.clear();

      for (OneDoFJointBasics joint : controlledJoints)
      {
         jointLimitEnforcementCommand.addJoint(joint, jointPositionLimitStiffness.getDoubleValue(), jointPositionLimitDamping.getDoubleValue());
      }
   }

   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return getFeedbackControlCommand();
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return null;
   }

   public VirtualModelControlCommand<?> getVirtualModelControlCommand()
   {
      return jointLimitEnforcementCommand;
   }
}
