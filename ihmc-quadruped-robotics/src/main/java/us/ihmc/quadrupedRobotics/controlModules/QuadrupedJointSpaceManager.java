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
import us.ihmc.quadrupedRobotics.parameters.QuadrupedPrivilegedConfigurationParameters;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

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

   private final DoubleProvider privilegedConfigurationWeight;
   private final DoubleProvider privilegedConfigurationGain;
   private final DoubleProvider privilegedConfigurationVelocityGain;

   private final List<DoubleProvider> privilegedConfigurations = new ArrayList<>();
   private final List<OneDoFJointBasics> kneeJoints = new ArrayList<>();

   public QuadrupedJointSpaceManager(QuadrupedControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      controlledJoints = controllerToolbox.getFullRobotModel().getControllableOneDoFJoints();
      QuadrupedPrivilegedConfigurationParameters privilegedConfigurationParameters = controllerToolbox.getRuntimeEnvironment().getPrivilegedConfigurationParameters();

      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         OneDoFJointBasics joint = controllerToolbox.getFullRobotModel().getLegJoint(quadrant, LegJointName.KNEE_PITCH);
         kneeJoints.add(joint);
         privilegedConfigurations.add(new DoubleParameter(joint.getName() + "_PrivilegedConfiguration", registry, privilegedConfigurationParameters.getPrivilegedConfiguration(quadrant)));
      }


      privilegedConfigurationWeight = new DoubleParameter("kneePrivWeight", registry, privilegedConfigurationParameters.getDefaultWeight());
      privilegedConfigurationGain = new DoubleParameter("kneePrivGain", registry, privilegedConfigurationParameters.getDefaultConfigurationGain());
      privilegedConfigurationVelocityGain = new DoubleParameter("kneePrivVelocityGain", registry, privilegedConfigurationParameters.getDefaultVelocityGain());

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

      for (int i = 0; i < kneeJoints.size(); i++)
      {
         privilegedConfigurationCommand.addJoint(kneeJoints.get(i), privilegedConfigurations.get(i).getValue());
         privilegedConfigurationCommand.setConfigurationGains(privilegedConfigurationGain.getValue());
         privilegedConfigurationCommand.setVelocityGains(privilegedConfigurationVelocityGain.getValue());
         privilegedConfigurationCommand.setWeight(i, privilegedConfigurationWeight.getValue());
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

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      commandList.clear();

      commandList.addCommand(jointLimitEnforcementMethodCommand);
      commandList.addCommand(privilegedConfigurationCommand);
      return commandList;
   }
}
