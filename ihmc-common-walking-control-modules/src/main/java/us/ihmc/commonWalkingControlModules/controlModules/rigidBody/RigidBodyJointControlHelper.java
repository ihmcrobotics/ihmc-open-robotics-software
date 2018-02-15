package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controlModules.ControllerCommandValidationTools;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.JointspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.OneDoFJointTrajectoryCommand;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPIDGains;
import us.ihmc.robotics.lists.RecyclingArrayDeque;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1D;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class RigidBodyJointControlHelper
{
   public static final String shortName = "JointControlHelper";

   private final YoVariableRegistry registry;
   private final String warningPrefix;
   private final YoBoolean trajectoryDone;

   private final List<MultipleWaypointsTrajectoryGenerator> jointTrajectoryGenerators = new ArrayList<>();
   private final List<RecyclingArrayDeque<SimpleTrajectoryPoint1D>> pointQueues = new ArrayList<>();

   private final List<YoInteger> numberOfPointsInQueue = new ArrayList<>();
   private final List<YoInteger> numberOfPointsInGenerator = new ArrayList<>();
   private final List<YoInteger> numberOfPoints = new ArrayList<>();

   private final YoBoolean usingWeightFromMessage;
   private final List<DoubleProvider> defaultWeights = new ArrayList<>();
   private final List<YoDouble> currentWeights = new ArrayList<>();
   private final List<YoDouble> messageWeights = new ArrayList<>();
   private final List<PIDGainsReadOnly> gains = new ArrayList<>();

   private final YoBoolean hasWeights;
   private final YoBoolean hasGains;

   private final SimpleTrajectoryPoint1D lastPointAdded = new SimpleTrajectoryPoint1D();
   private final JointspaceFeedbackControlCommand feedbackControlCommand = new JointspaceFeedbackControlCommand();

   private final OneDoFJoint[] joints;
   private final int numberOfJoints;

   public RigidBodyJointControlHelper(String bodyName, OneDoFJoint[] jointsToControl, YoVariableRegistry parentRegistry)
   {
      warningPrefix = shortName + " for " + bodyName + ": ";
      registry = new YoVariableRegistry(bodyName + shortName);
      trajectoryDone = new YoBoolean(shortName + "Done", registry);

      this.joints = jointsToControl;
      numberOfJoints = joints.length;

      String prefix = bodyName + "Jointspace";
      hasWeights = new YoBoolean(prefix + "HasWeights", registry);
      hasGains = new YoBoolean(prefix + "HasGains", registry);
      usingWeightFromMessage = new YoBoolean(prefix + "UsingWeightFromMessage", registry);

      for (int jointIdx = 0; jointIdx < jointsToControl.length; jointIdx++)
      {
         OneDoFJoint joint = jointsToControl[jointIdx];
         String jointName = joint.getName();
         jointTrajectoryGenerators.add(new MultipleWaypointsTrajectoryGenerator(jointName, RigidBodyJointspaceControlState.maxPointsInGenerator, registry));

         RecyclingArrayDeque<SimpleTrajectoryPoint1D> pointQueue = new RecyclingArrayDeque<>(RigidBodyJointspaceControlState.maxPoints, SimpleTrajectoryPoint1D.class);
         pointQueue.clear();
         pointQueues.add(pointQueue);

         feedbackControlCommand.addJoint(joint, Double.NaN, Double.NaN, Double.NaN);

         numberOfPointsInQueue.add(new YoInteger(prefix + "_" + jointName + "_numberOfPointsInQueue", registry));
         numberOfPointsInGenerator.add(new YoInteger(prefix + "_" + jointName + "_numberOfPointsInGenerator", registry));
         numberOfPoints.add(new YoInteger(prefix + "_" + jointName + "_numberOfPoints", registry));

         messageWeights.add(new YoDouble(prefix + "_" + jointName + "_messageWeight", registry));
         currentWeights.add(new YoDouble(prefix + "_" + jointName + "_currentWeight", registry));
      }

      parentRegistry.addChild(registry);
   }

   public void setDefaultWeights(Map<String, DoubleProvider> weights)
   {
      hasWeights.set(true);
      defaultWeights.clear();
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         OneDoFJoint joint = joints[jointIdx];
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

   public void setGains(Map<String, PIDGainsReadOnly> gains)
   {
      hasGains.set(true);
      this.gains.clear();
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         OneDoFJoint joint = joints[jointIdx];
         if (gains.containsKey(joint.getName()))
         {
            this.gains.add(gains.get(joint.getName()));
         }
         else
         {
            this.gains.clear();
            hasGains.set(false);
            return;
         }
      }
   }

   public void setGains(YoPIDGains gains)
   {
      hasGains.set(true);
      this.gains.clear();
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         this.gains.add(gains);
      }
   }

   public void setWeightsToDefaults()
   {
      usingWeightFromMessage.set(false);
   }

   public boolean doAction(double timeInTrajectory)
   {
      if (!hasGains.getBooleanValue() || !hasWeights.getBooleanValue())
      {
         PrintTools.warn(warningPrefix + "Can not send joint trajectory commands. Do not have all weights and gains set.");
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

         generator.compute(timeInTrajectory);
         double desiredPosition = generator.getValue();
         double desiredVelocity = generator.getVelocity();
         double feedForwardAcceleration = generator.getAcceleration();

         OneDoFJoint joint = joints[jointIdx];
         PIDGainsReadOnly gain = gains.get(jointIdx);
         double weight = weights.get(jointIdx).getValue();
         currentWeights.get(jointIdx).set(weight);
         if (weight > 0.0)
         {
            feedbackControlCommand.addJoint(joint, desiredPosition, desiredVelocity, feedForwardAcceleration, gain, weight);
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
      RecyclingArrayDeque<SimpleTrajectoryPoint1D> pointQueue = pointQueues.get(jointIdx);
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

         SimpleTrajectoryPoint1D pointToAdd = pointQueue.pollFirst();
         generator.appendWaypoint(pointToAdd);
      }

      generator.initialize();
      return false;
   }

   public boolean handleTrajectoryCommand(JointspaceTrajectoryCommand command, double[] initialJointPositions)
   {
      if (!hasGains.getBooleanValue() || !hasWeights.getBooleanValue())
      {
         PrintTools.warn(warningPrefix + "Can not send joint trajectory commands. Do not have all weights and gains set.");
         return false;
      }

      if (!ControllerCommandValidationTools.checkOneDoFJointTrajectoryCommandList(joints, command.getTrajectoryPointLists()))
      {
         return false;
      }

      boolean override = command.getExecutionMode() == ExecutionMode.OVERRIDE;
      if (override || isEmpty())
      {
         overrideTrajectory();
         boolean messageHasValidWeights = true;

         for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
         {
            OneDoFJointTrajectoryCommand trajectoryPoints = command.getJointTrajectoryPointList(jointIdx);
            if (trajectoryPoints.getNumberOfTrajectoryPoints() > 0)
            {
               SimpleTrajectoryPoint1D trajectoryPoint = trajectoryPoints.getTrajectoryPoint(0);
               if (trajectoryPoint.getTime() > Double.MIN_VALUE)
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

      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         OneDoFJointTrajectoryCommand trajectoryPoints = command.getJointTrajectoryPointList(jointIdx);
         for (int pointIdx = 0; pointIdx < trajectoryPoints.getNumberOfTrajectoryPoints(); pointIdx++)
         {
            SimpleTrajectoryPoint1D trajectoryPoint = trajectoryPoints.getTrajectoryPoint(pointIdx);
            if (trajectoryPoint != null)
            {
               if (!checkTime(trajectoryPoint.getTime(), jointIdx))
               {
                  return false;
               }
               if (!queuePoint(trajectoryPoint, jointIdx))
               {
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
      RecyclingArrayDeque<SimpleTrajectoryPoint1D> pointQueue = pointQueues.get(jointIdx);
      SimpleTrajectoryPoint1D point = pointQueue.addLast();
      point.set(0.0, initialPosition, 0.0);
   }

   private boolean queuePoint(SimpleTrajectoryPoint1D trajectoryPoint, int jointIdx)
   {
      RecyclingArrayDeque<SimpleTrajectoryPoint1D> pointQueue = pointQueues.get(jointIdx);
      if (atCapacityLimit(pointQueue))
      {
         return false;
      }

      SimpleTrajectoryPoint1D point = pointQueue.addLast();
      point.set(trajectoryPoint);
      return true;
   }

   private boolean queuePoint(double q, double qd, double t, int jointIdx)
   {
      RecyclingArrayDeque<SimpleTrajectoryPoint1D> pointQueue = pointQueues.get(jointIdx);
      if (atCapacityLimit(pointQueue))
      {
         return false;
      }

      SimpleTrajectoryPoint1D point = pointQueue.addLast();
      point.set(t, q, qd);
      return true;
   }

   private boolean atCapacityLimit(RecyclingArrayDeque<SimpleTrajectoryPoint1D> pointQueue)
   {
      if (pointQueue.size() >= RigidBodyJointspaceControlState.maxPoints)
      {
         PrintTools.info(warningPrefix + "Reached maximum capacity of " + RigidBodyJointspaceControlState.maxPoints + " can not execute trajectory.");
         return true;
      }
      return false;
   }

   private boolean checkTime(double time, int jointIdx)
   {
      boolean timeValid = time > getLastTrajectoryPointTime(jointIdx);
      if (!timeValid)
      {
         PrintTools.warn(warningPrefix + "Time in trajectory must be strictly increasing.");
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

      RecyclingArrayDeque<SimpleTrajectoryPoint1D> pointQueue = pointQueues.get(jointIdx);
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
      RecyclingArrayDeque<SimpleTrajectoryPoint1D> pointQueue = pointQueues.get(jointIdx);
      MultipleWaypointsTrajectoryGenerator generator = jointTrajectoryGenerators.get(jointIdx);
      return pointQueue.isEmpty() && generator.isDone();
   }

   public double getJointDesiredPosition(int jointIdx)
   {
      return jointTrajectoryGenerators.get(jointIdx).getValue();
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
         OneDoFJoint joint = joints[jointIdx];
         queueInitialPoint(joint.getQ(), jointIdx);
      }
   }
}
