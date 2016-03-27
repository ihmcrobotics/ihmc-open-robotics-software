package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import java.util.Map;

import us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlMode;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.trajectories.DoubleTrajectoryGenerator;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class JointSpaceHandControlState extends HandControlState
{
   private final OneDoFJoint[] oneDoFJoints;
   private Map<OneDoFJoint, ? extends DoubleTrajectoryGenerator> trajectories;
   private final JointspaceFeedbackControlCommand jointspaceFeedbackControlCommand = new JointspaceFeedbackControlCommand();

   private final YoVariableRegistry registry;
   private final YoPIDGains gains;
   private final DoubleYoVariable weight;

   public JointSpaceHandControlState(String namePrefix, OneDoFJoint[] controlledJoints, YoPIDGains gains, YoVariableRegistry parentRegistry)
   {
      super(HandControlMode.JOINT_SPACE);
      this.gains = gains;

      String name = namePrefix + getClass().getSimpleName();
      registry = new YoVariableRegistry(name);

      oneDoFJoints = controlledJoints;

      weight = new DoubleYoVariable(namePrefix + "JointspaceWeight", registry);
      weight.set(SolverWeightLevels.ARM_JOINTSPACE_WEIGHT);

      jointspaceFeedbackControlCommand.setGains(gains);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         jointspaceFeedbackControlCommand.addJoint(joint, Double.NaN, Double.NaN, Double.NaN);
      }

      parentRegistry.addChild(registry);
   }

   public void setWeight(double weight)
   {
      this.weight.set(weight);
   }

   @Override
   public void doAction()
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];

         DoubleTrajectoryGenerator trajectoryGenerator = trajectories.get(joint);
         trajectoryGenerator.compute(getTimeInCurrentState());

         double desiredPosition = trajectoryGenerator.getValue();
         double desiredVelocity = trajectoryGenerator.getVelocity();
         double feedForwardAcceleration = trajectoryGenerator.getAcceleration();

         jointspaceFeedbackControlCommand.setOneDoFJoint(i, desiredPosition, desiredVelocity, feedForwardAcceleration);
         jointspaceFeedbackControlCommand.setGains(gains);
         jointspaceFeedbackControlCommand.setWeightForSolver(weight.getDoubleValue());
      }
   }

   @Override
   public void doTransitionIntoAction()
   {
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }

   @Override
   public boolean isDone()
   {
      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         if (!trajectories.get(oneDoFJoint).isDone())
            return false;
      }

      return true;
   }

   public double getJointDesiredPosition(OneDoFJoint joint)
   {
      return trajectories.get(joint).getValue();
   }

   public double getJointDesiredVelocity(OneDoFJoint joint)
   {
      return trajectories.get(joint).getVelocity();
   }

   public void setTrajectories(Map<OneDoFJoint, ? extends DoubleTrajectoryGenerator> trajectories)
   {
      this.trajectories = trajectories;
   }

   public boolean hasJointTrajectories()
   {
      return trajectories != null;
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return null;
   }

   @Override
   public JointspaceFeedbackControlCommand getFeedbackControlCommand()
   {
      return jointspaceFeedbackControlCommand;
   }
}
