package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointLimitEnforcementMethodCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.JointLimitEnforcementCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedJointSpaceManager
{
   private static final double POSITION_LIMIT_DAMPING = 10.0;
   private static final double POSITION_LIMIT_STIFFNESS = 100.0;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final OneDoFJointBasics[] controlledJoints;

   private final InverseDynamicsCommandList commandList = new InverseDynamicsCommandList();

   private final JointLimitEnforcementCommand jointLimitEnforcementCommand = new JointLimitEnforcementCommand();
   private final JointLimitEnforcementMethodCommand jointLimitEnforcementMethodCommand = new JointLimitEnforcementMethodCommand();

   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();

   private final YoDouble jointPositionLimitDamping = new YoDouble("jointPositionLimitDamping", registry);
   private final YoDouble jointPositionLimitStiffness = new YoDouble("jointPositionLimitStiffness", registry);

   private final DoubleProvider privilegedConfigurationWeight = new DoubleParameter("privilegedConfigurationWeight", registry, 5.0);
   private final DoubleProvider privilegedConfigurationStiffness = new DoubleParameter("privilegedConfigurationStiffness", registry, 40.0);
   private final DoubleProvider privilegedConfigurationDamping = new DoubleParameter("privilegedConfigurationDamping", registry, 6.0);

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

      privilegedConfigurationCommand.clear();

      privilegedConfigurationCommand.setDefaultWeight(privilegedConfigurationWeight.getValue());
      privilegedConfigurationCommand.setDefaultConfigurationGain(privilegedConfigurationStiffness.getValue());
      privilegedConfigurationCommand.setDefaultVelocityGain(privilegedConfigurationDamping.getValue());
      for (OneDoFJointBasics joint : controlledJoints)
         privilegedConfigurationCommand.addJoint(joint, PrivilegedConfigurationCommand.PrivilegedConfigurationOption.AT_MID_RANGE);
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

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      commandList.clear();

      commandList.addCommand(jointLimitEnforcementMethodCommand);
      commandList.addCommand(privilegedConfigurationCommand);
      return commandList;
   }
}
