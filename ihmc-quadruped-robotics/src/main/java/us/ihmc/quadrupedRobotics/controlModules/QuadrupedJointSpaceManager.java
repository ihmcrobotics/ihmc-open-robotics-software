package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointVelocityIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointspaceVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.JointLimitEnforcementCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.JointTorqueCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommandList;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedJointSpaceManager
{
   private static final double VMC_VISCOUS_DAMPING = 1.0;
   private static final double IK_VISCOUS_DAMPING = 0.00;
   private static final double IK_DAMPING_WEIGHT = 0.0;

   private static final double POSITION_LIMIT_DAMPING = 10.0;
   private static final double POSITION_LIMIT_STIFFNESS = 100.0;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final OneDoFJoint[] controlledJoints;

   private final VirtualModelControlCommandList virtualModelControlCommandList = new VirtualModelControlCommandList();
   private final JointLimitEnforcementCommand jointLimitEnforcementCommand = new JointLimitEnforcementCommand();
   private final JointTorqueCommand vmcJointDampingCommand = new JointTorqueCommand();

   private final YoDouble vmcJointViscousDamping = new YoDouble("vmcJointViscousDamping", registry);
   private final YoDouble jointPositionLimitDamping = new YoDouble("jointPositionLimitDamping", registry);
   private final YoDouble jointPositionLimitStiffness = new YoDouble("jointPositionLimitStiffness", registry);

   private final InverseKinematicsCommandList inverseKinematicsCommandList = new InverseKinematicsCommandList();
   private final JointspaceVelocityCommand ikJointVelocityCommand = new JointspaceVelocityCommand();
   private final JointVelocityIntegrationCommand ikJointIntegrationCommand = new JointVelocityIntegrationCommand();

   private final YoDouble ikJointViscousDamping = new YoDouble("ikJointViscousDamping", registry);
   private final YoDouble ikJointWeight = new YoDouble("ikJointWeight", registry);

   public QuadrupedJointSpaceManager(QuadrupedControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      controlledJoints = controllerToolbox.getFullRobotModel().getControllableOneDoFJoints();

      vmcJointViscousDamping.set(VMC_VISCOUS_DAMPING);
      jointPositionLimitDamping.set(POSITION_LIMIT_DAMPING);
      jointPositionLimitStiffness.set(POSITION_LIMIT_STIFFNESS);

      ikJointViscousDamping.set(IK_VISCOUS_DAMPING);
      ikJointWeight.set(IK_DAMPING_WEIGHT);

      for (OneDoFJoint controlledJoint : controlledJoints)
      {
         ikJointIntegrationCommand.addJointToComputeDesiredPositionFor(controlledJoint);
      }

      for (int i = 0; i < ikJointIntegrationCommand.getNumberOfJointsToComputeDesiredPositionFor(); i++)
      {
         ikJointIntegrationCommand.setJointMaxima(i, 1.0, 100.0);
      }

      parentRegistry.addChild(registry);
   }

   public void compute()
   {
      vmcJointDampingCommand.clear();
      jointLimitEnforcementCommand.clear();

      ikJointVelocityCommand.clear();

      for (OneDoFJoint joint : controlledJoints)
      {
         vmcJointDampingCommand.addJoint(joint, -vmcJointViscousDamping.getDoubleValue() * joint.getQd());
         jointLimitEnforcementCommand.addJoint(joint, jointPositionLimitStiffness.getDoubleValue(), jointPositionLimitDamping.getDoubleValue());

         ikJointVelocityCommand.addJoint(joint, -ikJointViscousDamping.getDoubleValue() * joint.getQd());
      }
      ikJointVelocityCommand.setWeight(ikJointWeight.getDoubleValue());
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
      virtualModelControlCommandList.clear();
      virtualModelControlCommandList.addCommand(jointLimitEnforcementCommand);
      virtualModelControlCommandList.addCommand(vmcJointDampingCommand);

      return virtualModelControlCommandList;
   }

   public InverseKinematicsCommand<?> getInverseKinematicsCommand()
   {
      inverseKinematicsCommandList.clear();
      inverseKinematicsCommandList.addCommand(ikJointVelocityCommand);
      inverseKinematicsCommandList.addCommand(ikJointIntegrationCommand);

      return inverseKinematicsCommandList;
   }
}
