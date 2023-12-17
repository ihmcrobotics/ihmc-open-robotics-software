package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controlModules.ControllerCommandValidationTools;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OneDoFJointFeedbackControlCommand;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.lists.RecyclingArrayDeque;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.JointspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.OneDoFJointTrajectoryCommand;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPIDGains;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorNew;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.OneDoFTrajectoryPoint;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class RigidBodyJointControlHelper
{
   public static final String shortName = "JointControlHelper";

   private final YoRegistry registry;
   private final String warningPrefix;
   private final YoBoolean trajectoryDone;

   private final List<MultipleWaypointsTrajectoryGenerator> jointTrajectoryGenerators = new ArrayList<>();
   private final List<RecyclingArrayDeque<OneDoFTrajectoryPoint>> pointQueues = new ArrayList<>();

   private final List<YoInteger> numberOfPointsInQueue = new ArrayList<>();
   private final List<YoInteger> numberOfPointsInGenerator = new ArrayList<>();
   private final List<YoInteger> numberOfPoints = new ArrayList<>();
   /**
    * Used when streaming to account for time variations occurring during the transport of the message
    * over the network.
    */
   private final YoDouble streamTimestampOffset;
   private final YoDouble streamTimestampSource;

   private final YoBoolean usingWeightFromMessage;
   private final List<DoubleProvider> defaultWeights = new ArrayList<>();
   private final List<YoDouble> currentWeights = new ArrayList<>();
   private final List<YoDouble> messageWeights = new ArrayList<>();
   private final List<PIDGainsReadOnly> highLevelGains = new ArrayList<>();
   private final List<PIDGainsReadOnly> lowLevelGains = new ArrayList<>();
   private final List<YoFunctionGeneratorNew> functionGenerators = new ArrayList<>();

   private final YoBoolean hasWeights;
   private final YoBoolean hasHighLevelGains;
   private final YoBoolean[] hasLowLevelGains;

   private final OneDoFTrajectoryPoint lastPointAdded = new OneDoFTrajectoryPoint();
   private final JointspaceFeedbackControlCommand feedbackControlCommand = new JointspaceFeedbackControlCommand();

   private final OneDoFJointBasics[] joints;
   private final int numberOfJoints;

   private final DoubleProvider time;

   public RigidBodyJointControlHelper(String bodyName, OneDoFJointBasics[] jointsToControl, DoubleProvider time, boolean enableFunctionGenerators, YoRegistry parentRegistry)
   {
      warningPrefix = shortName + " for " + bodyName + ": ";
      registry = new YoRegistry(bodyName + shortName);
      trajectoryDone = new YoBoolean(shortName + "Done", registry);

      this.time = time;
      this.joints = jointsToControl;
      numberOfJoints = joints.length;

      String prefix = bodyName + "Jointspace";
      hasWeights = new YoBoolean(prefix + "HasWeights", registry);
      hasHighLevelGains = new YoBoolean(prefix + "HasHighLevelGains", registry);
      hasLowLevelGains = new YoBoolean[jointsToControl.length];
      usingWeightFromMessage = new YoBoolean(prefix + "UsingWeightFromMessage", registry);

      for (int jointIdx = 0; jointIdx < jointsToControl.length; jointIdx++)
      {
         OneDoFJointBasics joint = jointsToControl[jointIdx];
         String jointName = joint.getName();
         jointTrajectoryGenerators.add(new MultipleWaypointsTrajectoryGenerator(jointName, RigidBodyJointspaceControlState.maxPointsInGenerator, registry));

         RecyclingArrayDeque<OneDoFTrajectoryPoint> pointQueue = new RecyclingArrayDeque<>(RigidBodyJointspaceControlState.maxPoints,
                                                                                           OneDoFTrajectoryPoint.class,
                                                                                           OneDoFTrajectoryPoint::set);
         pointQueue.clear();
         pointQueues.add(pointQueue);

         feedbackControlCommand.addJointCommand(joint);

         numberOfPointsInQueue.add(new YoInteger(prefix + "_" + jointName + "_numberOfPointsInQueue", registry));
         numberOfPointsInGenerator.add(new YoInteger(prefix + "_" + jointName + "_numberOfPointsInGenerator", registry));
         numberOfPoints.add(new YoInteger(prefix + "_" + jointName + "_numberOfPoints", registry));

         messageWeights.add(new YoDouble(prefix + "_" + jointName + "_messageWeight", registry));
         currentWeights.add(new YoDouble(prefix + "_" + jointName + "_currentWeight", registry));

         if (enableFunctionGenerators)
         {
            functionGenerators.add(new YoFunctionGeneratorNew(prefix + "_" + jointName + "_FG", time, registry));
         }

         hasLowLevelGains[jointIdx] = new YoBoolean(joint.getName() + "HasLowLevelGains", registry);
      }

      streamTimestampOffset = new YoDouble(prefix + "StreamTimestampOffset", registry);
      streamTimestampOffset.setToNaN();
      streamTimestampSource = new YoDouble(prefix + "StreamTimestampSource", registry);
      streamTimestampSource.setToNaN();

      parentRegistry.addChild(registry);
   }

   public void setDefaultWeights(Map<String, DoubleProvider> weights)
   {
      hasWeights.set(true);
      defaultWeights.clear();
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         OneDoFJointBasics joint = joints[jointIdx];
         if (weights.containsKey(joint.getName()))
         {
            defaultWeights.add(weights.get(joint.getName()));
         }
         else
         {
            defaultWeights.clear();
            hasWeights.set(false);
            return;
         }
      }
      setWeightsToDefaults();
   }

   public void setDefaultWeight(DoubleProvider weight)
   {
      hasWeights.set(true);
      defaultWeights.clear();
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         this.defaultWeights.add(weight);
      }
      setWeightsToDefaults();
   }

   public void setGains(Map<String, PIDGainsReadOnly> highLevelGains, Map<String, PIDGainsReadOnly> lowLevelGains)
   {
      hasHighLevelGains.set(true);
      this.highLevelGains.clear();
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         OneDoFJointBasics joint = joints[jointIdx];
         if (highLevelGains.containsKey(joint.getName()))
         {
            this.highLevelGains.add(highLevelGains.get(joint.getName()));
         }
         else
         {
            this.highLevelGains.clear();
            hasHighLevelGains.set(false);
            return;
         }
      }

      this.lowLevelGains.clear();
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         hasLowLevelGains[jointIdx].set(false);
      }

      if (lowLevelGains == null)
         return;

      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         OneDoFJointBasics joint = joints[jointIdx];
         this.lowLevelGains.add(jointIdx, lowLevelGains.get(joint.getName()));
         hasLowLevelGains[jointIdx].set(lowLevelGains.containsKey(joint.getName()));
      }
   }

   public void setHighLevelGains(YoPIDGains highLevelGains)
   {
      hasHighLevelGains.set(true);
      this.highLevelGains.clear();
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         this.highLevelGains.add(highLevelGains);
      }
   }

   public void setWeightsToDefaults()
   {
      usingWeightFromMessage.set(false);
   }

   public boolean doAction(double timeInTrajectory)
   {
      if (!hasHighLevelGains.getBooleanValue() || !hasWeights.getBooleanValue())
      {
         LogTools.warn(warningPrefix + "Can not send joint trajectory commands. Do not have all weights and gains set.");
         throw new RuntimeException(warningPrefix + "Has no gains or weights.");
      }

      boolean allDone = true;

      List<? extends DoubleProvider> weights = usingWeightFromMessage.getBooleanValue() ? messageWeights : defaultWeights;

      feedbackControlCommand.clear();
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         MultipleWaypointsTrajectoryGenerator generator = jointTrajectoryGenerators.get(jointIdx);
         boolean generatorDone = generator.isDone() || generator.getLastWaypointTime() <= timeInTrajectory;

         if (!trajectoryDone.getBooleanValue() && generatorDone)
         {
            allDone = fillAndReinitializeTrajectories(jointIdx) && allDone;
         }
         else if (trajectoryDone.getBooleanValue())
         {
            allDone = true;
         }
         else
         {
            allDone = false;
         }

         if (allDone)
         {
            streamTimestampOffset.setToNaN();
            streamTimestampSource.setToNaN();
         }

         generator.compute(timeInTrajectory);
         double desiredPosition = generator.getValue();
         double desiredVelocity = generator.getVelocity();
         double feedForwardAcceleration = generator.getAcceleration();

         if (!functionGenerators.isEmpty())
         {
            functionGenerators.get(jointIdx).update();
            desiredPosition += functionGenerators.get(jointIdx).getValue();
            desiredVelocity += functionGenerators.get(jointIdx).getValueDot();
            feedForwardAcceleration += functionGenerators.get(jointIdx).getValueDDot();
         }

         OneDoFJointBasics joint = joints[jointIdx];
         PIDGainsReadOnly gain = highLevelGains.get(jointIdx);
         double weight = weights.get(jointIdx).getValue();
         currentWeights.get(jointIdx).set(weight);
         if (weight > 0.0)
         {
            OneDoFJointFeedbackControlCommand jointCommand = feedbackControlCommand.addJointCommand(joint);
            jointCommand.setInverseDynamics(desiredPosition, desiredVelocity, feedForwardAcceleration);
            jointCommand.setGains(gain);
            jointCommand.setWeightForSolver(weight);
         }

         YoInteger numberOfPointsInQueue = this.numberOfPointsInQueue.get(jointIdx);
         YoInteger numberOfPointsInGenerator = this.numberOfPointsInGenerator.get(jointIdx);
         YoInteger numberOfPoints = this.numberOfPoints.get(jointIdx);
         numberOfPointsInQueue.set(pointQueues.get(jointIdx).size());
         numberOfPointsInGenerator.set(generator.getCurrentNumberOfWaypoints());
         numberOfPoints.set(numberOfPointsInQueue.getIntegerValue() + numberOfPointsInGenerator.getIntegerValue());
      }

      trajectoryDone.set(allDone);
      return trajectoryDone.getBooleanValue();
   }

   private boolean fillAndReinitializeTrajectories(int jointIdx)
   {
      RecyclingArrayDeque<OneDoFTrajectoryPoint> pointQueue = pointQueues.get(jointIdx);
      if (pointQueue.isEmpty())
      {
         return true;
      }

      MultipleWaypointsTrajectoryGenerator generator = jointTrajectoryGenerators.get(jointIdx);
      if (!generator.isEmpty())
      {
         generator.getLastWaypoint(lastPointAdded);
         generator.clear();
         generator.appendWaypoint(lastPointAdded);
      }

      int currentNumberOfWaypoints = generator.getCurrentNumberOfWaypoints();
      int pointsToAdd = RigidBodyJointspaceControlState.maxPointsInGenerator - currentNumberOfWaypoints;
      for (int pointIdx = 0; pointIdx < pointsToAdd; pointIdx++)
      {
         if (pointQueue.isEmpty())
         {
            break;
         }

         OneDoFTrajectoryPoint pointToAdd = pointQueue.pollFirst();
         generator.appendWaypoint(pointToAdd);
      }

      generator.initialize();
      return false;
   }

   public boolean handleTrajectoryCommand(JointspaceTrajectoryCommand command, double[] initialJointPositions)
   {
      if (!hasHighLevelGains.getBooleanValue() || !hasWeights.getBooleanValue())
      {
         LogTools.warn(warningPrefix + "Can not send joint trajectory commands. Do not have all weights and gains set.");
         return false;
      }

      if (!ControllerCommandValidationTools.checkOneDoFJointTrajectoryCommandList(joints, command.getTrajectoryPointLists()))
      {
         return false;
      }

      double streamTimeOffset = 0.0;
      double streamTimestampOffset = this.streamTimestampOffset.getValue();
      double streamTimestampSource = this.streamTimestampSource.getValue();

      if (command.getExecutionMode() == ExecutionMode.STREAM)
      { // Need to do time checks before moving on.
         if (command.getTimestamp() <= 0)
         {
            streamTimestampOffset = Double.NaN;
            streamTimestampSource = Double.NaN;
         }
         else
         {
            double senderTime = Conversions.nanosecondsToSeconds(command.getTimestamp());

            if (!Double.isNaN(streamTimestampSource) && senderTime < streamTimestampSource)
            {
               // Messages are out of order which is fine, we just don't want to handle the new message.
               return true;
            }

            streamTimestampSource = senderTime;

            streamTimeOffset = time.getValue() - senderTime;

            if (Double.isNaN(streamTimestampOffset))
            {
               streamTimestampOffset = streamTimeOffset;
            }
            else
            {
               /*
                * Update to the smallest time offset, which is closer to the true offset between the sender CPU and
                * control CPU. If the change in offset is too large though, we always set the streamTimestampOffset
                * for safety.
                */
               if (Math.abs(streamTimeOffset - streamTimestampOffset) > 0.5)
                  streamTimestampOffset = streamTimeOffset;
               else
                  streamTimestampOffset = Math.min(streamTimeOffset, streamTimestampOffset);
            }
         }
      }

      // Both OVERRIDE and STREAM should override the current trajectory stored.
      boolean override = command.getExecutionMode() != ExecutionMode.QUEUE;

      if (override || isEmpty())
      {
         overrideTrajectory();
         boolean messageHasValidWeights = true;

         for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
         {
            OneDoFJointTrajectoryCommand trajectoryPoints = command.getJointTrajectoryPointList(jointIdx);

            if (trajectoryPoints.getNumberOfTrajectoryPoints() > 0)
            {
               OneDoFTrajectoryPoint trajectoryPoint = trajectoryPoints.getTrajectoryPoint(0);
               if (trajectoryPoint.getTime() > RigidBodyTaskspaceControlState.timeEpsilonForInitialPoint)
               {
                  queueInitialPoint(initialJointPositions[jointIdx], jointIdx);
               }
            }
            else
            {
               queueInitialPoint(initialJointPositions[jointIdx], jointIdx);
            }

            double weight = trajectoryPoints.getWeight();
            messageWeights.get(jointIdx).set(weight);

            if (Double.isNaN(weight) || weight < 0.0)
            {
               messageHasValidWeights = false;
            }
         }

         usingWeightFromMessage.set(messageHasValidWeights);
      }

      if (command.getExecutionMode() == ExecutionMode.STREAM)
      {
         this.streamTimestampOffset.set(streamTimestampOffset);
         this.streamTimestampSource.set(streamTimestampSource);
      }

      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         OneDoFJointTrajectoryCommand trajectoryPoints = command.getJointTrajectoryPointList(jointIdx);

         if (command.getExecutionMode() == ExecutionMode.STREAM)
         {
            if (trajectoryPoints.getNumberOfTrajectoryPoints() != 1)
            {
               LogTools.warn("When streaming, trajectories should contain only 1 trajectory point, was: " + trajectoryPoints.getNumberOfTrajectoryPoints());
               return false;
            }

            OneDoFTrajectoryPoint trajectoryPoint = trajectoryPoints.getTrajectoryPoint(0);

            if (trajectoryPoint.getTime() != 0.0)
            {
               LogTools.warn("When streaming, the trajectory point should have a time of zero, was: " + trajectoryPoint.getTime());
               return false;
            }

            // When a message is delayed during shipping over network, timeOffset > streamTimestampOffset.getValue().
            // This new time will put the trajectory point in the past, closer to when it should have been received.
            double t0 = Double.isNaN(streamTimestampOffset) ? 0.0 : streamTimestampOffset - streamTimeOffset;
            double q0 = trajectoryPoint.getPosition();
            double qd0 = trajectoryPoint.getVelocity();

            if (!queuePoint(q0, qd0, t0, jointIdx))
               return false;

            double t1 = command.getStreamIntegrationDuration() + t0;
            double qd1 = trajectoryPoint.getVelocity();
            double q1 = trajectoryPoint.getPosition() + command.getStreamIntegrationDuration() * qd1;

            if (!queuePoint(q1, qd1, t1, jointIdx))
               return false;
         }
         else
         {
            for (int pointIdx = 0; pointIdx < trajectoryPoints.getNumberOfTrajectoryPoints(); pointIdx++)
            {
               OneDoFTrajectoryPoint trajectoryPoint = trajectoryPoints.getTrajectoryPoint(pointIdx);

               if (trajectoryPoint != null)
               {
                  if (!checkTime(trajectoryPoint.getTime(), jointIdx))
                     return false;

                  if (!queuePoint(trajectoryPoint, jointIdx))
                     return false;
               }
            }
         }
      }

      trajectoryDone.set(false);
      return true;
   }

   public void queuePointsAtTimeWithZeroVelocity(double time, double[] jointPositions)
   {
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         queuePoint(jointPositions[jointIdx], 0.0, time, jointIdx);
      }
   }

   public void startTrajectoryExecution()
   {
      trajectoryDone.set(false);
   }

   private void queueInitialPoint(double initialPosition, int jointIdx)
   {
      RecyclingArrayDeque<OneDoFTrajectoryPoint> pointQueue = pointQueues.get(jointIdx);
      OneDoFTrajectoryPoint point = pointQueue.addLast();
      point.set(0.0, initialPosition, 0.0);
   }

   private boolean queuePoint(OneDoFTrajectoryPoint trajectoryPoint, int jointIdx)
   {
      RecyclingArrayDeque<OneDoFTrajectoryPoint> pointQueue = pointQueues.get(jointIdx);
      if (atCapacityLimit(pointQueue))
      {
         return false;
      }

      OneDoFTrajectoryPoint point = pointQueue.addLast();
      point.set(trajectoryPoint);
      return true;
   }

   private boolean queuePoint(double q, double qd, double t, int jointIdx)
   {
      RecyclingArrayDeque<OneDoFTrajectoryPoint> pointQueue = pointQueues.get(jointIdx);
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

   private boolean checkTime(double time, int jointIdx)
   {
      boolean timeValid = time > getLastTrajectoryPointTime(jointIdx);
      if (!timeValid)
      {
         LogTools.warn(warningPrefix + "Time in trajectory must be strictly increasing.");
      }
      return timeValid;
   }

   public double getLastTrajectoryPointTime()
   {
      double lastTrajectoryPointTime = Double.NEGATIVE_INFINITY;
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         double jointLastTime = getLastTrajectoryPointTime(jointIdx);
         lastTrajectoryPointTime = Math.max(lastTrajectoryPointTime, jointLastTime);
      }
      return lastTrajectoryPointTime;
   }

   private double getLastTrajectoryPointTime(int jointIdx)
   {
      if (isEmpty(jointIdx))
      {
         return Double.NEGATIVE_INFINITY;
      }

      RecyclingArrayDeque<OneDoFTrajectoryPoint> pointQueue = pointQueues.get(jointIdx);
      MultipleWaypointsTrajectoryGenerator generator = jointTrajectoryGenerators.get(jointIdx);

      if (pointQueue.isEmpty())
      {
         return generator.getLastWaypointTime();
      }
      return pointQueue.peekLast().getTime();
   }

   public void overrideTrajectory()
   {
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         overrideTrajectory(jointIdx);
      }
      streamTimestampOffset.setToNaN();
      streamTimestampSource.setToNaN();
   }

   private void overrideTrajectory(int jointIdx)
   {
      jointTrajectoryGenerators.get(jointIdx).clear();
      pointQueues.get(jointIdx).clear();
   }

   public boolean isEmpty()
   {
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         if (!isEmpty(jointIdx))
         {
            return false;
         }
      }
      return true;
   }

   private boolean isEmpty(int jointIdx)
   {
      RecyclingArrayDeque<OneDoFTrajectoryPoint> pointQueue = pointQueues.get(jointIdx);
      MultipleWaypointsTrajectoryGenerator generator = jointTrajectoryGenerators.get(jointIdx);
      return pointQueue.isEmpty() && generator.isDone();
   }

   public double getJointDesiredPosition(int jointIdx)
   {
      return jointTrajectoryGenerators.get(jointIdx).getValue();
   }

   public void queueInitialPointsAtCurrentDesired()
   {
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         queueInitialPoint(getJointDesiredPosition(jointIdx), jointIdx);
      }
   }

   public double getJointDesiredVelocity(int jointIdx)
   {
      return jointTrajectoryGenerators.get(jointIdx).getVelocity();
   }

   public JointspaceFeedbackControlCommand getJointspaceCommand()
   {
      return feedbackControlCommand;
   }

   public void queueInitialPointsAtCurrent()
   {
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         OneDoFJointBasics joint = joints[jointIdx];
         queueInitialPoint(joint.getQ(), jointIdx);
      }
   }

   public boolean hasLowLevelJointGains(int jointIdx)
   {
      return hasLowLevelGains[jointIdx].getValue();
   }
   
   public PIDGainsReadOnly getLowLevelJointGain(int jointIdx)
   {
      return lowLevelGains.get(jointIdx);
   }
}
