package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import static us.ihmc.communication.packets.Packet.INVALID_MESSAGE_ID;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controlModules.ControllerCommandValidationTools;
import us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlMode;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ArmTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.OneDoFJointTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.lists.RecyclingArrayDeque;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsTrajectoryGenerator;
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

   private final BooleanYoVariable abortJointspaceControlState;
   private final LongYoVariable lastCommandId;

   private final BooleanYoVariable isReadyToHandleQueuedCommands;
   private final Map<OneDoFJoint, IntegerYoVariable> numberOfQueuedCommands = new HashMap<>();
   private final Map<OneDoFJoint, RecyclingArrayDeque<OneDoFJointTrajectoryCommand>> commandQueues = new LinkedHashMap<>();

   public JointSpaceHandControlState(String namePrefix, Map<OneDoFJoint, Double> homeConfiguration, OneDoFJoint[] controlledJoints, YoPIDGains gains,
         YoVariableRegistry parentRegistry)
   {
      super(HandControlMode.JOINTSPACE);
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

         numberOfQueuedCommands.put(joint, new IntegerYoVariable(joint.getName() + "NumberOfQueuedCommands", registry));
         commandQueues.put(joint, new RecyclingArrayDeque<>(OneDoFJointTrajectoryCommand.class));
      }

      isReadyToHandleQueuedCommands = new BooleanYoVariable(namePrefix + "IsReadyToHandleQueuedArmTrajectoryCommands", registry);
      abortJointspaceControlState = new BooleanYoVariable(namePrefix + "AbortJointspaceControlState", registry);
      lastCommandId = new LongYoVariable(namePrefix + "LastCommandId", registry);
      lastCommandId.set(Packet.INVALID_MESSAGE_ID);

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

      isReadyToHandleQueuedCommands.set(false);
      clearCommandQueues(INVALID_MESSAGE_ID);
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

      isReadyToHandleQueuedCommands.set(false);
      clearCommandQueues(INVALID_MESSAGE_ID);
   }

   public boolean handleArmTrajectoryCommand(ArmTrajectoryCommand command, boolean initializeToCurrent)
   {
      if (!ControllerCommandValidationTools.checkArmTrajectoryCommand(controlledJoints, command))
         return false;

      isReadyToHandleQueuedCommands.set(true);
      clearCommandQueues(command.getCommandId());

      RecyclingArrayList<OneDoFJointTrajectoryCommand> jointTrajectoryCommands = command.getTrajectoryPointLists();
      int numberOfJoints = command.getNumberOfJoints();

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         OneDoFJointTrajectoryCommand jointTrajectoryCommand = jointTrajectoryCommands.get(jointIndex);
         initializeTrajectoryGenerator(initializeToCurrent, 0.0, jointIndex, jointTrajectoryCommand);
      }

      return true;
   }

   public boolean queueArmTrajectoryCommand(ArmTrajectoryCommand command)
   {
      if (!isReadyToHandleQueuedCommands.getBooleanValue())
      {
         PrintTools.warn(this, "The very first " + command.getClass().getSimpleName() + " of a series must be " + ExecutionMode.OVERRIDE + ". Aborting motion.");
         return false;
      }

      long previousCommandId = command.getPreviousCommandId();

      if (previousCommandId != INVALID_MESSAGE_ID && lastCommandId.getLongValue() != INVALID_MESSAGE_ID && lastCommandId.getLongValue() != previousCommandId)
      {
         PrintTools.warn(this, "Previous command ID mismatch: previous ID from command = " + previousCommandId
               + ", last message ID received by the controller = " + lastCommandId.getLongValue() + ". Aborting motion.");
         isReadyToHandleQueuedCommands.set(false);
         clearCommandQueues(INVALID_MESSAGE_ID);
         abortJointspaceControlState.set(true);
         return false;
      }

      for (int jointIndex = 0; jointIndex < command.getNumberOfJoints(); jointIndex++)
      {
         OneDoFJoint joint = controlledJoints[jointIndex];
         OneDoFJointTrajectoryCommand localCommand = commandQueues.get(joint).addLast();
         numberOfQueuedCommands.get(joint).increment();

         OneDoFJointTrajectoryCommand jointTrajectoryCommand = command.getJointTrajectoryPointList(jointIndex);

         if (jointTrajectoryCommand.getTrajectoryPoint(0).getTime() < 1.0e-5)
         {
            PrintTools.warn(this, "Time of the first trajectory point of a queued command must be greater than zero. Aborting motion.");
            isReadyToHandleQueuedCommands.set(false);
            clearCommandQueues(INVALID_MESSAGE_ID);
            abortJointspaceControlState.set(true);
            return false;
         }

         localCommand.set(jointTrajectoryCommand);
         localCommand.setCommandId(command.getCommandId());
      }

      lastCommandId.set(command.getCommandId());

      return true;
   }

   @Override
   public void doAction()
   {
      for (int i = 0; i < controlledJoints.length; i++)
      {
         OneDoFJoint joint = controlledJoints[i];

         MultipleWaypointsTrajectoryGenerator trajectoryGenerator = jointTrajectoryGenerators.get(joint);
         RecyclingArrayDeque<OneDoFJointTrajectoryCommand> commandQueue = commandQueues.get(joint);

         trajectoryGenerator.compute(getTimeInCurrentState());

         if (trajectoryGenerator.isDone() && !commandQueue.isEmpty())
         {
            double firstWaypointTime = trajectoryGenerator.getLastWaypointTime();
            OneDoFJointTrajectoryCommand jointTrajectoryCommand = commandQueue.poll();
            numberOfQueuedCommands.get(joint).decrement();
            initializeTrajectoryGenerator(false, firstWaypointTime, i, jointTrajectoryCommand);
            trajectoryGenerator.compute(getTimeInCurrentState());
         }

         double desiredPosition = trajectoryGenerator.getValue();
         double desiredVelocity = trajectoryGenerator.getVelocity();
         double feedForwardAcceleration = trajectoryGenerator.getAcceleration();

         jointspaceFeedbackControlCommand.setOneDoFJoint(i, desiredPosition, desiredVelocity, feedForwardAcceleration);
         jointspaceFeedbackControlCommand.setGains(gains);
         jointspaceFeedbackControlCommand.setWeightForSolver(weight.getDoubleValue());
      }
   }

   private void initializeTrajectoryGenerator(boolean initializeToCurrent, double firstWaypointTime, int jointIndex,
         OneDoFJointTrajectoryCommand jointTrajectoryCommand)
   {
      OneDoFJoint joint = controlledJoints[jointIndex];
      MultipleWaypointsTrajectoryGenerator trajectoryGenerator = jointTrajectoryGenerators.get(joint);

      jointTrajectoryCommand.addTimeOffset(firstWaypointTime);

      if (jointTrajectoryCommand.getNumberOfTrajectoryPoints() == 0)
      {
         if (trajectoryGenerator.isEmpty())
         {
            trajectoryGenerator.appendWaypoint(0.0, joint.getQ(), 0.0);
            trajectoryGenerator.initialize();
         }
         return;
      }

      if (jointTrajectoryCommand.getTrajectoryPoint(0).getTime() > firstWaypointTime + 1.0e-5)
      {
         double initialPosition = initializeToCurrent ? joint.getQ() : trajectoryGenerator.getValue();
         double initialVelocity = initializeToCurrent ? joint.getQd() : trajectoryGenerator.getVelocity();
         trajectoryGenerator.clear();
         trajectoryGenerator.appendWaypoint(firstWaypointTime, initialPosition, initialVelocity);
      }
      else
      {
         trajectoryGenerator.clear();
      }

      int numberOfTrajectoryPoints = queueExcedingTrajectoryPointsIfNeeded(jointIndex, jointTrajectoryCommand);

      for (int i = 0; i < numberOfTrajectoryPoints; i++)
         trajectoryGenerator.appendWaypoint(jointTrajectoryCommand.getTrajectoryPoint(i));

      trajectoryGenerator.initialize();
   }

   private int queueExcedingTrajectoryPointsIfNeeded(int jointIndex, OneDoFJointTrajectoryCommand command)
   {
      int numberOfTrajectoryPoints = command.getNumberOfTrajectoryPoints();

      OneDoFJoint joint = controlledJoints[jointIndex];
      MultipleWaypointsTrajectoryGenerator jointTrajectoryGenerator = jointTrajectoryGenerators.get(joint);
      int maximumNumberOfWaypoints = jointTrajectoryGenerator.getMaximumNumberOfWaypoints();
      maximumNumberOfWaypoints -= jointTrajectoryGenerator.getCurrentNumberOfWaypoints();

      if (numberOfTrajectoryPoints <= maximumNumberOfWaypoints)
         return numberOfTrajectoryPoints;

      OneDoFJointTrajectoryCommand commandForExcedent = commandQueues.get(joint).addFirst();
      numberOfQueuedCommands.get(joint).increment();
      commandForExcedent.clear();
      commandForExcedent.setCommandId(command.getCommandId());

      for (int trajectoryPointIndex = maximumNumberOfWaypoints; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         commandForExcedent.addTrajectoryPoint(command.getTrajectoryPoint(trajectoryPointIndex));
      }

      double timeOffsetToSubtract = command.getTrajectoryPoint(maximumNumberOfWaypoints - 1).getTime();
      commandForExcedent.subtractTimeOffset(timeOffsetToSubtract);

      return maximumNumberOfWaypoints;
   }

   @Override
   public void doTransitionIntoAction()
   {
   }

   @Override
   public void doTransitionOutOfAction()
   {
      abortJointspaceControlState.set(false);
   }

   private void clearCommandQueues(long lastCommandId)
   {
      for (int i = 0; i < controlledJoints.length; i++)
      {
         OneDoFJoint joint = controlledJoints[i];
         commandQueues.get(joint).clear();
         numberOfQueuedCommands.get(joint).set(0);
      }
      this.lastCommandId.set(lastCommandId);
   }

   @Override
   public boolean isDone()
   {
      for (int i = 0; i < controlledJoints.length; i++)
      {
         if (!commandQueues.get(controlledJoints[i]).isEmpty())
            return false;
      }

      return areTrajectoriesDone();
   }

   private boolean areTrajectoriesDone()
   {
      for (OneDoFJoint oneDoFJoint : controlledJoints)
      {
         if (!jointTrajectoryGenerators.get(oneDoFJoint).isDone())
            return false;
      }

      return true;
   }

   @Override
   public boolean isAbortRequested()
   {
      return abortJointspaceControlState.getBooleanValue();
   }

   public boolean isReadyToHandleQueuedCommands()
   {
      return isReadyToHandleQueuedCommands.getBooleanValue();
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
