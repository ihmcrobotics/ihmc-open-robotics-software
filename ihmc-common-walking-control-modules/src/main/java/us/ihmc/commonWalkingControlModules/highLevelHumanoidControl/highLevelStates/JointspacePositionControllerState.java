package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage;
import gnu.trove.impl.Constants;
import gnu.trove.map.hash.TIntIntHashMap;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.OneDoFJointTrajectoryStatusMessageHelper;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyJointspaceControlState;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.lists.RecyclingArrayDeque;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.OneDoFJointTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.WholeBodyJointspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.OneDoFTrajectoryPoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;

/**
 * This high-level controller state offers an API for tracking trajectories defined in joint-space
 * only. The input command for this controller is {@link WholeBodyJointspaceTrajectoryCommand} which
 * allows to define trajectories for each individual joint. This controller relies on the low-level
 * robot controller to perform position tracking in joint-space given desired position and velocity
 * over time.
 */
public class JointspacePositionControllerState extends HighLevelControllerState
{
   private final JointDesiredOutputListReadOnly highLevelControllerOutput;
   private final JointDesiredOutputList jointDesiredOutputList;

   private final OneDoFJointBasics[] joints;
   private final OneDoFJointManager[] jointManagers;
   private final TIntIntHashMap hashCodeToJointIndexMap = new TIntIntHashMap(40, Constants.DEFAULT_LOAD_FACTOR, -1, -1);
   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusOutputManager;

   private final RecyclingArrayList<JointspaceTrajectoryStatusMessage> combinedStatuses = new RecyclingArrayList<>(JointspaceTrajectoryStatusMessage::new);

   public JointspacePositionControllerState(HighLevelControllerName stateEnum, CommandInputManager commandInputManager,
                                            StatusMessageOutputManager statusOutputManager, OneDoFJointBasics[] controlledJoints,
                                            HighLevelHumanoidControllerToolbox controllerToolbox, HighLevelControllerParameters highLevelControllerParameters,
                                            JointDesiredOutputListReadOnly highLevelControllerOutput)
   {
      super(stateEnum, highLevelControllerParameters, controlledJoints);
      this.commandInputManager = commandInputManager;
      this.statusOutputManager = statusOutputManager;

      this.highLevelControllerOutput = highLevelControllerOutput;

      joints = controlledJoints;
      jointManagers = new OneDoFJointManager[joints.length];

      for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
      {
         OneDoFJointBasics joint = joints[jointIndex];
         OneDoFJointManager manager = new OneDoFJointManager(joint, controllerToolbox.getYoTime(), registry);
         jointManagers[jointIndex] = manager;
         if (joint.hashCode() == hashCodeToJointIndexMap.getNoEntryKey())
            throw new IllegalStateException("Cannot register a joint's hash-code that is equal to the NO_ENTRY key value.");
         hashCodeToJointIndexMap.put(joint.hashCode(), jointIndex);
      }

      jointDesiredOutputList = new JointDesiredOutputList(joints);
   }

   public void holdCurrent()
   {
      for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
      {
         jointManagers[jointIndex].holdCurrent();
      }
   }

   @Override
   public void onEntry()
   {
      for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
      {
         OneDoFJointBasics joint = joints[jointIndex];
         OneDoFJointManager manager = jointManagers[jointIndex];

         JointDesiredOutputReadOnly jointData = highLevelControllerOutput.getJointDesiredOutput(joint);
         if (jointData != null && jointData.hasDesiredPosition())
            manager.holderPosition(jointData.getDesiredPosition());
         else
            manager.holdCurrent();
      }
   }

   @Override
   public void doAction(double timeInState)
   {
      consumeCommands();

      combinedStatuses.clear();

      for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
      {
         OneDoFJointBasics joint = joints[jointIndex];
         OneDoFJointManager manager = jointManagers[jointIndex];

         JointDesiredOutputBasics desiredOutput = jointDesiredOutputList.getJointDesiredOutput(joint);
         desiredOutput.clear();
         manager.doControl(desiredOutput);

         JointspaceTrajectoryStatusMessage status = manager.pollStatusToReport();

         if (status != null)
         {
            combineStatus(status);
         }
      }

      jointDesiredOutputList.completeWith(getStateSpecificJointSettings());

      for (int i = 0; i < combinedStatuses.size(); i++)
         statusOutputManager.reportStatusMessage(combinedStatuses.get(i));
   }

   private void combineStatus(JointspaceTrajectoryStatusMessage jointStatus)
   {
      JointspaceTrajectoryStatusMessage matchingCombinedStatus = null;

      for (int i = 0; i < combinedStatuses.size(); i++)
      {
         JointspaceTrajectoryStatusMessage candidate = combinedStatuses.get(i);

         if (candidate.getTimestamp() != jointStatus.getTimestamp())
            continue;
         if (candidate.getSequenceId() != jointStatus.getSequenceId())
            continue;
         if (candidate.getTrajectoryExecutionStatus() != jointStatus.getTrajectoryExecutionStatus())
            continue;

         matchingCombinedStatus = candidate;
         break;
      }

      if (matchingCombinedStatus == null)
      {
         matchingCombinedStatus = combinedStatuses.add();
         matchingCombinedStatus.getJointNames().clear();
         matchingCombinedStatus.getActualJointPositions().reset();
         matchingCombinedStatus.getDesiredJointPositions().reset();
         matchingCombinedStatus.setTimestamp(jointStatus.getTimestamp());
         matchingCombinedStatus.setSequenceId(jointStatus.getSequenceId());
         matchingCombinedStatus.setTrajectoryExecutionStatus(jointStatus.getTrajectoryExecutionStatus());
      }

      for (int i = 0; i < jointStatus.getJointNames().size(); i++)
      { // There should be only one joint, but oh well
         matchingCombinedStatus.getJointNames().add(jointStatus.getJointNames().getString(i));
         matchingCombinedStatus.getActualJointPositions().add(jointStatus.getActualJointPositions().get(i));
         matchingCombinedStatus.getDesiredJointPositions().add(jointStatus.getDesiredJointPositions().get(i));
      }
   }

   @Override
   public void onExit()
   {
      // Do nothing
   }

   private void consumeCommands()
   {
      if (commandInputManager.isNewCommandAvailable(WholeBodyJointspaceTrajectoryCommand.class))
      {
         WholeBodyJointspaceTrajectoryCommand command = commandInputManager.pollNewestCommand(WholeBodyJointspaceTrajectoryCommand.class);

         for (int commandIdx = 0; commandIdx < command.getNumberOfJoints(); commandIdx++)
         {
            int jointHashCode = command.getJointHashCode(commandIdx);
            OneDoFJointTrajectoryCommand jointTrajectory = command.getJointTrajectoryPointList(commandIdx);
            int jointIndex = hashCodeToJointIndexMap.get(jointHashCode);

            if (jointIndex == -1)
            {
               LogTools.warn("Joint not supported, hash-code: {}", jointHashCode);
               continue;
            }

            jointManagers[jointIndex].handleTrajectoryCommand(jointTrajectory, command);
         }
      }
   }

   @Override
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return jointDesiredOutputList;
   }

   public static class OneDoFJointManager
   {
      public static final String shortName = "JointManager";

      private final OneDoFJointBasics joint;

      private final String warningPrefix;
      private final YoBoolean trajectoryDone;

      private final YoDouble trajectoryStartTime;

      private final MultipleWaypointsTrajectoryGenerator jointTrajectoryGenerator;
      private final RecyclingArrayDeque<OneDoFTrajectoryPoint> pointQueue;

      private final YoInteger numberOfPointsInQueue;
      private final YoInteger numberOfPointsInGenerator;
      private final YoInteger numberOfPoints;

      private final OneDoFTrajectoryPoint lastPointAdded = new OneDoFTrajectoryPoint();

      private final OneDoFJointTrajectoryStatusMessageHelper statusHelper;

      private final YoLong lastCommandId;

      private final DoubleProvider time;

      public OneDoFJointManager(OneDoFJointBasics joint, DoubleProvider time, YoRegistry registry)
      {
         this.joint = joint;
         this.time = time;

         String jointName = joint.getName();
         warningPrefix = shortName + " for " + jointName + ": ";
         trajectoryDone = new YoBoolean(jointName + shortName + "Done", registry);

         trajectoryStartTime = new YoDouble(jointName + "_trajectoryStartTime", registry);
         jointTrajectoryGenerator = new MultipleWaypointsTrajectoryGenerator(jointName, RigidBodyJointspaceControlState.maxPointsInGenerator, registry);
         pointQueue = new RecyclingArrayDeque<>(RigidBodyJointspaceControlState.maxPoints, OneDoFTrajectoryPoint::new, OneDoFTrajectoryPoint::set);

         numberOfPointsInQueue = new YoInteger(jointName + "_numberOfPointsInQueue", registry);
         numberOfPointsInGenerator = new YoInteger(jointName + "_numberOfPointsInGenerator", registry);
         numberOfPoints = new YoInteger(jointName + "_numberOfPoints", registry);

         statusHelper = new OneDoFJointTrajectoryStatusMessageHelper(joint);

         lastCommandId = new YoLong(joint.getName() + "LastCommandId", registry);
         lastCommandId.set(Packet.INVALID_MESSAGE_ID);
      }

      public void doControl(JointDesiredOutputBasics jointDesiredOutput)
      {
         double timeInTrajectory = time.getValue() - trajectoryStartTime.getValue();

         statusHelper.updateWithTimeInTrajectory(timeInTrajectory);

         boolean allDone = true;

         boolean generatorDone = jointTrajectoryGenerator.isDone() || jointTrajectoryGenerator.getLastWaypointTime() <= timeInTrajectory;

         if (!trajectoryDone.getBooleanValue() && generatorDone)
         {
            allDone = fillAndReinitializeTrajectories() && allDone;
         }
         else if (trajectoryDone.getBooleanValue())
         {
            allDone = true;
         }
         else
         {
            allDone = false;
         }

         jointTrajectoryGenerator.compute(timeInTrajectory);
         jointDesiredOutput.setDesiredPosition(jointTrajectoryGenerator.getValue());
         jointDesiredOutput.setDesiredVelocity(jointTrajectoryGenerator.getVelocity());
         jointDesiredOutput.setDesiredAcceleration(jointTrajectoryGenerator.getAcceleration());

         numberOfPointsInQueue.set(pointQueue.size());
         numberOfPointsInGenerator.set(jointTrajectoryGenerator.getCurrentNumberOfWaypoints());
         numberOfPoints.set(numberOfPointsInQueue.getIntegerValue() + numberOfPointsInGenerator.getIntegerValue());

         trajectoryDone.set(allDone);
      }

      private boolean fillAndReinitializeTrajectories()
      {
         if (pointQueue.isEmpty())
            return true;

         if (!jointTrajectoryGenerator.isEmpty())
         {
            jointTrajectoryGenerator.getLastWaypoint(lastPointAdded);
            jointTrajectoryGenerator.clear();
            jointTrajectoryGenerator.appendWaypoint(lastPointAdded);
         }

         int currentNumberOfWaypoints = jointTrajectoryGenerator.getCurrentNumberOfWaypoints();
         int pointsToAdd = RigidBodyJointspaceControlState.maxPointsInGenerator - currentNumberOfWaypoints;

         for (int pointIdx = 0; pointIdx < pointsToAdd; pointIdx++)
         {
            if (pointQueue.isEmpty())
            {
               break;
            }

            OneDoFTrajectoryPoint pointToAdd = pointQueue.pollFirst();
            jointTrajectoryGenerator.appendWaypoint(pointToAdd);
         }

         jointTrajectoryGenerator.initialize();
         return false;
      }

      public boolean handleTrajectoryCommand(OneDoFJointTrajectoryCommand command, QueueableCommand<?, ?> queueingProperties)
      {
         return handleTrajectoryCommand(command, queueingProperties, getJointDesiredPosition());
      }

      public boolean handleTrajectoryCommand(OneDoFJointTrajectoryCommand command, QueueableCommand<?, ?> queueingProperties, double initialJointPosition)
      {
         if (!validateQueueableProperties(queueingProperties))
            return false;

         // Both OVERRIDE and STREAM should override the current trajectory stored.
         boolean override = queueingProperties.getExecutionMode() != ExecutionMode.QUEUE;

         if (override || isEmpty())
         {
            overrideTrajectory();

            if (command.getNumberOfTrajectoryPoints() > 0)
            {
               OneDoFTrajectoryPoint trajectoryPoint = command.getTrajectoryPoint(0);

               if (trajectoryPoint.getTime() > RigidBodyTaskspaceControlState.timeEpsilonForInitialPoint)
                  queueInitialPoint(initialJointPosition);
            }
            else
            {
               queueInitialPoint(initialJointPosition);
            }
         }

         for (int pointIdx = 0; pointIdx < command.getNumberOfTrajectoryPoints(); pointIdx++)
         {
            OneDoFTrajectoryPoint trajectoryPoint = command.getTrajectoryPoint(pointIdx);

            if (trajectoryPoint != null)
            {
               if (!checkTime(trajectoryPoint.getTime()))
                  return false;

               if (!queuePoint(trajectoryPoint))
                  return false;
            }
         }

         trajectoryDone.set(false);
         statusHelper.registerNewTrajectory(command, queueingProperties.getExecutionMode());
         return true;
      }

      private boolean validateQueueableProperties(QueueableCommand<?, ?> queueingProperties)
      {
         if (queueingProperties.getCommandId() == Packet.INVALID_MESSAGE_ID)
         {
            LogTools.warn(warningPrefix + "Recieved packet with invalid ID.");
            return false;
         }

         boolean wantToQueue = queueingProperties.getExecutionMode() == ExecutionMode.QUEUE;
         boolean previousIdMatch = queueingProperties.getPreviousCommandId() == lastCommandId.getLongValue();

         if (!isEmpty() && wantToQueue && !previousIdMatch)
         {
            LogTools.warn(warningPrefix + "Unexpected command ID. Msg previous id: " + queueingProperties.getPreviousCommandId() + " but was "
                  + lastCommandId.getLongValue());
            return false;
         }

         if (!wantToQueue || isEmpty())
            trajectoryStartTime.set(time.getValue());
         else
            queueingProperties.addTimeOffset(getLastTrajectoryPointTime());

         lastCommandId.set(queueingProperties.getCommandId());

         trajectoryDone.set(false);
         return true;
      }

      public void holdCurrent()
      {
         holderPosition(joint.getQ());
      }

      public void holdCurrentDesired()
      {
         holderPosition(getJointDesiredPosition());
      }

      public void holderPosition(double position)
      {
         overrideTrajectory();
         resetLastCommandId();
         trajectoryStartTime.set(time.getValue());
         queueInitialPoint(position);
         trajectoryDone.set(false);

      }

      private void resetLastCommandId()
      {
         lastCommandId.set(Packet.INVALID_MESSAGE_ID);
      }

      public void queueInitialPointAtCurrent()
      {
         queueInitialPoint(joint.getQ());
      }

      public void queueInitialPointAtCurrentDesired()
      {
         queueInitialPoint(getJointDesiredPosition());
      }

      private void queueInitialPoint(double initialPosition)
      {
         pointQueue.addLast().set(0.0, initialPosition, 0.0);
      }

      private boolean queuePoint(OneDoFTrajectoryPoint trajectoryPoint)
      {
         if (atCapacityLimit(pointQueue))
         {
            return false;
         }

         OneDoFTrajectoryPoint point = pointQueue.addLast();
         point.set(trajectoryPoint);
         return true;
      }

      public void queuePointAtTimeWithZeroVelocity(double time, double jointPosition)
      {
         queuePoint(jointPosition, 0.0, time);
      }

      private boolean queuePoint(double q, double qd, double t)
      {
         if (atCapacityLimit(pointQueue))
         {
            return false;
         }

         OneDoFTrajectoryPoint point = pointQueue.addLast();
         point.set(t, q, qd);
         return true;
      }

      private boolean atCapacityLimit(RecyclingArrayDeque<OneDoFTrajectoryPoint> pointQueue)
      {
         if (pointQueue.size() >= RigidBodyJointspaceControlState.maxPoints)
         {
            LogTools.info(warningPrefix + "Reached maximum capacity of " + RigidBodyJointspaceControlState.maxPoints + " can not execute trajectory.");
            return true;
         }
         return false;
      }

      private boolean checkTime(double time)
      {
         boolean timeValid = time > getLastTrajectoryPointTime();
         if (!timeValid)
         {
            LogTools.warn(warningPrefix + "Time in trajectory must be strictly increasing.");
         }
         return timeValid;
      }

      private double getLastTrajectoryPointTime()
      {
         if (isEmpty())
         {
            return Double.NEGATIVE_INFINITY;
         }

         if (pointQueue.isEmpty())
         {
            return jointTrajectoryGenerator.getLastWaypointTime();
         }
         return pointQueue.peekLast().getTime();
      }

      private void overrideTrajectory()
      {
         jointTrajectoryGenerator.clear();
         pointQueue.clear();
      }

      private boolean isEmpty()
      {
         return pointQueue.isEmpty() && jointTrajectoryGenerator.isDone();
      }

      public double getJointDesiredPosition()
      {
         return jointTrajectoryGenerator.getValue();
      }

      public double getJointDesiredVelocity()
      {
         return jointTrajectoryGenerator.getVelocity();
      }

      public JointspaceTrajectoryStatusMessage pollStatusToReport()
      {
         return statusHelper.pollStatusMessage(joint.getQ(), getJointDesiredPosition());
      }
   }
}
