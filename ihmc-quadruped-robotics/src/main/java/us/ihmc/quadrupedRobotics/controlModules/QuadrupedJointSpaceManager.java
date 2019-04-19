package us.ihmc.quadrupedRobotics.controlModules;

import javafx.util.Pair;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointLimitEnforcementMethodCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.JointLimitEnforcementCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitEnforcement;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedPrivilegedConfigurationParameters;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

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
   private final PairList<OneDoFJointBasics, Double> jointConfigurations = new PairList<>();

   public QuadrupedJointSpaceManager(QuadrupedControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      controlledJoints = controllerToolbox.getFullRobotModel().getControllableOneDoFJoints();
      QuadrupedPrivilegedConfigurationParameters privilegedConfigurationParameters = controllerToolbox.getRuntimeEnvironment().getPrivilegedConfigurationParameters();

      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         OneDoFJointBasics kneePitch = controllerToolbox.getFullRobotModel().getLegJoint(quadrant, LegJointName.KNEE_PITCH);
         OneDoFJointBasics hipRoll = controllerToolbox.getFullRobotModel().getLegJoint(quadrant, LegJointName.HIP_ROLL);
         OneDoFJointBasics hipPitch = controllerToolbox.getFullRobotModel().getLegJoint(quadrant, LegJointName.HIP_PITCH);
         kneeJoints.add(kneePitch);
         privilegedConfigurations.add(new DoubleParameter(kneePitch.getName() + "_PrivilegedConfiguration", registry, privilegedConfigurationParameters.getPrivilegedConfiguration(LegJointName.KNEE_PITCH, quadrant)));
         jointConfigurations.add(hipRoll, privilegedConfigurationParameters.getPrivilegedConfiguration(LegJointName.HIP_ROLL, quadrant));
         jointConfigurations.add(hipPitch, privilegedConfigurationParameters.getPrivilegedConfiguration(LegJointName.HIP_PITCH, quadrant));
      }

      JointLimitParameters jointLimitParameters = new JointLimitParameters();
      jointLimitParameters.setMaxAbsJointVelocity(20.0);
      jointLimitParameters.setJointLimitDistanceForMaxVelocity(Math.toRadians(10.0));
      jointLimitParameters.setVelocityControlGain(20.0);
      for (OneDoFJointBasics joint : controlledJoints)
      {
         jointLimitEnforcementMethodCommand.addLimitEnforcementMethod(joint, JointLimitEnforcement.DEFAULT, jointLimitParameters);
      }

      privilegedConfigurationWeight = new DoubleParameter("kneePrivWeight", registry, privilegedConfigurationParameters.getKneeWeight());
      privilegedConfigurationGain = new DoubleParameter("kneePrivGain", registry, privilegedConfigurationParameters.getKneeConfigurationGain());
      privilegedConfigurationVelocityGain = new DoubleParameter("kneePrivVelocityGain", registry, privilegedConfigurationParameters.getKneeConfigurationVelocityGain());

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

      for (int i = 0; i < jointConfigurations.size(); i++)
      {
         privilegedConfigurationCommand.addJoint(jointConfigurations.get(i).getLeft(), jointConfigurations.get(i).getRight());
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
