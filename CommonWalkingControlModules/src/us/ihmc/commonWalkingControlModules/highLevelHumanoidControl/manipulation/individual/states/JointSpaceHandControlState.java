package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controlModules.ControllerCommandValidationTools;
import us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlMode;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ArmTrajectoryCommand;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.trajectories.DoubleTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.tools.io.printing.PrintTools;

public class JointSpaceHandControlState extends HandControlState
{
   private final OneDoFJoint[] controlledJoints;
   private final Map<OneDoFJoint, Double> homeConfiguration;
   private final Map<OneDoFJoint, MultipleWaypointsTrajectoryGenerator> jointTrajectoryGenerators;
   private final JointspaceFeedbackControlCommand jointspaceFeedbackControlCommand = new JointspaceFeedbackControlCommand();

   private final YoVariableRegistry registry;
   private final YoPIDGains gains;
   private final DoubleYoVariable weight;
   private final RobotSide robotSide;

   public JointSpaceHandControlState(String namePrefix, RobotSide robotSide, Map<OneDoFJoint, Double> homeConfiguration, OneDoFJoint[] controlledJoints,
         YoPIDGains gains, YoVariableRegistry parentRegistry)
   {
      super(HandControlMode.JOINTSPACE);
      this.robotSide = robotSide;
      this.homeConfiguration = homeConfiguration;
      this.gains = gains;

      String name = namePrefix + getClass().getSimpleName();
      registry = new YoVariableRegistry(name);

      this.controlledJoints = controlledJoints;

      weight = new DoubleYoVariable(namePrefix + "JointspaceWeight", registry);
      weight.set(SolverWeightLevels.ARM_JOINTSPACE_WEIGHT);

      jointspaceFeedbackControlCommand.setGains(gains);

      for (int i = 0; i < controlledJoints.length; i++)
      {
         OneDoFJoint joint = controlledJoints[i];
         jointspaceFeedbackControlCommand.addJoint(joint, Double.NaN, Double.NaN, Double.NaN);
      }

      jointTrajectoryGenerators = new LinkedHashMap<>();

      for (OneDoFJoint joint : controlledJoints)
      {
         MultipleWaypointsTrajectoryGenerator multiWaypointTrajectoryGenerator = new MultipleWaypointsTrajectoryGenerator(joint.getName(), registry);
         jointTrajectoryGenerators.put(joint, multiWaypointTrajectoryGenerator);
      }

      parentRegistry.addChild(registry);
   }

   public void setWeight(double weight)
   {
      this.weight.set(weight);
   }

   public void goHome(double trajectoryTime, boolean initializeToCurrent)
   {
      for (int jointIndex = 0; jointIndex < controlledJoints.length; jointIndex++)
      {
         OneDoFJoint joint = controlledJoints[jointIndex];
         MultipleWaypointsTrajectoryGenerator trajectory = jointTrajectoryGenerators.get(joint);
         double initialPosition = initializeToCurrent ? joint.getQ() : trajectory.getValue();
         double initialVelocity = initializeToCurrent ? joint.getQd() : trajectory.getVelocity();
         trajectory.clear();
         trajectory.appendWaypoint(0.0, initialPosition, initialVelocity);
         trajectory.appendWaypoint(trajectoryTime, homeConfiguration.get(joint), 0.0);
         trajectory.initialize();
      }
   }

   public void holdCurrentConfiguration()
   {
      for (OneDoFJoint oneDoFJoint : controlledJoints)
      {
         MultipleWaypointsTrajectoryGenerator trajectory = jointTrajectoryGenerators.get(oneDoFJoint);
         trajectory.clear();
         trajectory.appendWaypoint(0.0, oneDoFJoint.getQ(), 0.0);
         trajectory.initialize();
      }
   }

   public boolean handleArmTrajectoryCommand(ArmTrajectoryCommand command, boolean initializeToCurrent)
   {
      if (command.getRobotSide() != robotSide)
      {
         PrintTools.warn(this, "Received a " + command.getClass().getSimpleName() + " for the wrong side.");
         return false;
      }

      if (!ControllerCommandValidationTools.checkJointspaceTrajectoryPointLists(controlledJoints, command.getTrajectoryPointLists()))
         return false;

      int numberOfJoints = command.getNumberOfJoints();

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         OneDoFJoint joint = controlledJoints[jointIndex];
         MultipleWaypointsTrajectoryGenerator trajectoryGenerator = jointTrajectoryGenerators.get(joint);

         if (command.getJointTrajectoryPoint(jointIndex, 0).getTime() > 1.0e-5)
         {
            double initialPosition = initializeToCurrent ? joint.getQ() : trajectoryGenerator.getValue();
            double initialVelocity = initializeToCurrent ? joint.getQd() : trajectoryGenerator.getVelocity();
            trajectoryGenerator.clear();
            trajectoryGenerator.appendWaypoint(0.0, initialPosition, initialVelocity);
         }
         else
         {
            trajectoryGenerator.clear();
         }

         SimpleTrajectoryPoint1DList jointTrajectory = command.getJointTrajectoryPointList(jointIndex);
         trajectoryGenerator.appendWaypoints(jointTrajectory);
         trajectoryGenerator.initialize();
      }

      return true;
   }

   @Override
   public void doAction()
   {
      for (int i = 0; i < controlledJoints.length; i++)
      {
         OneDoFJoint joint = controlledJoints[i];

         DoubleTrajectoryGenerator trajectoryGenerator = jointTrajectoryGenerators.get(joint);
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
      for (OneDoFJoint oneDoFJoint : controlledJoints)
      {
         if (!jointTrajectoryGenerators.get(oneDoFJoint).isDone())
            return false;
      }

      return true;
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
   public JointspaceFeedbackControlCommand getFeedbackControlCommand()
   {
      return jointspaceFeedbackControlCommand;
   }
}
