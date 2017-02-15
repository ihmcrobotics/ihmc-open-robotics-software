package us.ihmc.commonWalkingControlModules.controlModules.head;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controlModules.ControllerCommandValidationTools;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.NeckTrajectoryCommand;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class JointspaceHeadControlState extends HeadControlState
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final Map<OneDoFJoint, MultipleWaypointsTrajectoryGenerator> jointTrajectoryGenerators = new HashMap<>();

   private final DoubleYoVariable weight = new DoubleYoVariable("headJointspaceWeight", registry);
   private final YoPIDGains gains;
   private final JointspaceFeedbackControlCommand feedbackControlCommand = new JointspaceFeedbackControlCommand();

   private final OneDoFJoint[] jointsOriginal;

   public JointspaceHeadControlState(OneDoFJoint[] neckJoints, YoPIDGains gains, YoVariableRegistry parentRegistry)
   {
      super(HeadControlMode.JOINTSPACE);

      this.jointsOriginal = neckJoints;
      this.gains = gains;

      for (int i = 0; i < neckJoints.length; i++)
      {
         OneDoFJoint joint = neckJoints[i];
         String jointName = joint.getName();
         MultipleWaypointsTrajectoryGenerator jointTrajectoryGenerator = new MultipleWaypointsTrajectoryGenerator(jointName, registry);
         jointTrajectoryGenerators.put(joint, jointTrajectoryGenerator);

         feedbackControlCommand.addJoint(joint, Double.NaN, Double.NaN, Double.NaN);
      }

      parentRegistry.addChild(registry);
   }

   public boolean handleNeckTrajectoryCommand(NeckTrajectoryCommand command, double[] initialJointPositions)
   {
      if (!ControllerCommandValidationTools.checkNeckTrajectoryCommand(jointsOriginal, command))
      {
         if (jointsOriginal.length != command.getNumberOfJoints())
            System.err.println("Wrong number of neck joints: expected " + jointsOriginal.length + " got " + command.getNumberOfJoints());
         else
            System.err.println("Neck command out of joint limits!");
         return false;
      }

      int numberOfJoints = command.getNumberOfJoints();

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         OneDoFJoint joint = jointsOriginal[jointIndex];
         MultipleWaypointsTrajectoryGenerator trajectoryGenerator = jointTrajectoryGenerators.get(joint);
         trajectoryGenerator.clear();

         if (command.getJointTrajectoryPoint(jointIndex, 0).getTime() > 1.0e-5)
         {
            trajectoryGenerator.appendWaypoint(0.0, initialJointPositions[jointIndex], 0.0);
         }

         SimpleTrajectoryPoint1DList jointTrajectory = command.getJointTrajectoryPointList(jointIndex);
         trajectoryGenerator.appendWaypoints(jointTrajectory);
         trajectoryGenerator.initialize();
      }

      return true;
   }

   @Override
   public void setWeight(double weight)
   {
      this.weight.set(weight);
   }

   @Override
   public void doTransitionIntoAction()
   {
      for (int i = 0; i < jointsOriginal.length; i++)
      {
         jointTrajectoryGenerators.get(jointsOriginal[i]).initialize();
      }
   }

   @Override
   public void doAction()
   {
      for (int i = 0; i < jointsOriginal.length; i++)
      {
         MultipleWaypointsTrajectoryGenerator jointTajectoryGenerator = jointTrajectoryGenerators.get(jointsOriginal[i]);
         jointTajectoryGenerator.compute(getTimeInCurrentState());
         double desiredPosition = jointTajectoryGenerator.getValue();
         double desiredVelocity = jointTajectoryGenerator.getVelocity();
         double feedForwardAcceleration = jointTajectoryGenerator.getAcceleration();
         feedbackControlCommand.setOneDoFJoint(i, desiredPosition, desiredVelocity, feedForwardAcceleration);
      }

      feedbackControlCommand.setGains(gains);
      feedbackControlCommand.setWeightForSolver(weight.getDoubleValue());
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }

   public double getJointDesiredPosition(OneDoFJoint joint)
   {
      return jointTrajectoryGenerators.get(joint).getValue();
   }

   public double getJointDesiredVelocity(OneDoFJoint joint)
   {
      return jointTrajectoryGenerators.get(joint).getVelocity();
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return null;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return feedbackControlCommand;
   }
}
