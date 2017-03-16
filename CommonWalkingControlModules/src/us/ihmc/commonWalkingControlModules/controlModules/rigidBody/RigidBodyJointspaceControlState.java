package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.controlModules.ControllerCommandValidationTools;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commons.PrintTools;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.JointspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.lists.RecyclingArrayDeque;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1D;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class RigidBodyJointspaceControlState extends RigidBodyControlState
{
   public static final int maxPoints = 200;
   public static final int maxPointsInGenerator = 5;

   private final List<MultipleWaypointsTrajectoryGenerator> jointTrajectoryGenerators = new ArrayList<>();
   private final List<RecyclingArrayDeque<SimpleTrajectoryPoint1D>> pointQueues = new ArrayList<>();

   private final List<IntegerYoVariable> numberOfPointsInQueue = new ArrayList<>();
   private final List<IntegerYoVariable> numberOfPointsInGenerator = new ArrayList<>();
   private final List<IntegerYoVariable> numberOfPoints = new ArrayList<>();

   private final List<DoubleYoVariable> weights = new ArrayList<>();
   private final List<YoPIDGains> gains = new ArrayList<>();

   private final BooleanYoVariable hasWeights;
   private final BooleanYoVariable hasGains;

   private final SimpleTrajectoryPoint1D lastPointAdded = new SimpleTrajectoryPoint1D();
   private final JointspaceFeedbackControlCommand feedbackControlCommand = new JointspaceFeedbackControlCommand();

   private final OneDoFJoint[] jointsOriginal;
   private final double[] jointsHomeConfiguration;
   private final int numberOfJoints;

   public RigidBodyJointspaceControlState(String bodyName, OneDoFJoint[] jointsToControl, TObjectDoubleHashMap<String> homeConfiguration,
         DoubleYoVariable yoTime, YoVariableRegistry parentRegistry)
   {
      super(RigidBodyControlMode.JOINTSPACE, bodyName, yoTime);

      this.jointsOriginal = jointsToControl;
      numberOfJoints = jointsOriginal.length;

      String prefix = bodyName + "Jointspace";
      hasWeights = new BooleanYoVariable(prefix + "HasWeights", registry);
      hasGains = new BooleanYoVariable(prefix + "HasGains", registry);
      jointsHomeConfiguration = new double[numberOfJoints];

      for (int jointIdx = 0; jointIdx < jointsToControl.length; jointIdx++)
      {
         OneDoFJoint joint = jointsToControl[jointIdx];
         String jointName = joint.getName();
         jointTrajectoryGenerators.add(new MultipleWaypointsTrajectoryGenerator(jointName, maxPointsInGenerator, registry));

         RecyclingArrayDeque<SimpleTrajectoryPoint1D> pointQueue = new RecyclingArrayDeque<>(maxPoints, SimpleTrajectoryPoint1D.class);
         pointQueue.clear();
         pointQueues.add(pointQueue);

         feedbackControlCommand.addJoint(joint, Double.NaN, Double.NaN, Double.NaN);

         numberOfPointsInQueue.add(new IntegerYoVariable(prefix + "_" + jointName + "_numberOfPointsInQueue", registry));
         numberOfPointsInGenerator.add(new IntegerYoVariable(prefix + "_" + jointName + "_numberOfPointsInGenerator", registry));
         numberOfPoints.add(new IntegerYoVariable(prefix + "_" + jointName + "_numberOfPoints", registry));

         weights.add(new DoubleYoVariable(prefix + "_" + jointName + "_weight", registry));

         if (!homeConfiguration.contains(jointName))
            throw new RuntimeException(warningPrefix + "Can not create control manager since joint home configuration is not defined.");
         jointsHomeConfiguration[jointIdx] = homeConfiguration.get(jointName);
      }

      parentRegistry.addChild(registry);
   }

   public void setWeights(TObjectDoubleHashMap<String> weights)
   {
      hasWeights.set(true);
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         OneDoFJoint joint = jointsOriginal[jointIdx];
         if (weights.containsKey(joint.getName()))
            this.weights.get(jointIdx).set(weights.get(joint.getName()));
         else
            hasWeights.set(false);
      }
   }

   public void setWeight(double weight)
   {
      hasWeights.set(true);
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
         this.weights.get(jointIdx).set(weight);
   }

   public void setGains(Map<String, YoPIDGains> gains)
   {
      hasGains.set(true);
      this.gains.clear();
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         OneDoFJoint joint = jointsOriginal[jointIdx];
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
         this.gains.add(gains);
   }

   public void holdCurrent()
   {
      overrideTrajectory();
      resetLastCommandId();

      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         OneDoFJoint joint = jointsOriginal[jointIdx];
         queueInitialPoint(joint.getQ(), jointIdx);
      }

      trajectoryStopped.set(false);
      trajectoryDone.set(false);
   }

   public void goHome(double trajectoryTime, double[] initialJointPositions)
   {
      overrideTrajectory();
      resetLastCommandId();

      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         queueInitialPoint(initialJointPositions[jointIdx], jointIdx);
         queuePoint(jointsHomeConfiguration[jointIdx], 0.0, trajectoryTime, jointIdx);
      }

      trajectoryStopped.set(false);
      trajectoryDone.set(false);
   }

   public void goHomeFromCurrent(double trajectoryTime)
   {
      overrideTrajectory();
      resetLastCommandId();

      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         OneDoFJoint joint = jointsOriginal[jointIdx];
         queueInitialPoint(joint.getQ(), jointIdx);
         queuePoint(jointsHomeConfiguration[jointIdx], 0.0, trajectoryTime, jointIdx);
      }

      trajectoryStopped.set(false);
      trajectoryDone.set(false);
   }

   @Override
   public void doAction()
   {
      if (!hasGains.getBooleanValue() || !hasWeights.getBooleanValue())
      {
         PrintTools.warn(warningPrefix + "Can not send joint trajectory commands. Do not have all weights and gains set.");
         throw new RuntimeException(warningPrefix + "Has no gains or weights.");
      }

      double timeInTrajectory = getTimeInTrajectory();
      boolean allDone = true;

      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         MultipleWaypointsTrajectoryGenerator generator = jointTrajectoryGenerators.get(jointIdx);

         if (!trajectoryDone.getBooleanValue() && generator.isDone())
            allDone = fillAndReinitializeTrajectories(jointIdx) && allDone;
         else if (trajectoryDone.getBooleanValue())
            allDone = true;
         else
            allDone = false;

         if (!trajectoryStopped.getBooleanValue())
            generator.compute(timeInTrajectory);

         double desiredPosition = generator.getValue();
         double desiredVelocity = generator.getVelocity();
         double feedForwardAcceleration = generator.getAcceleration();

         feedbackControlCommand.setOneDoFJoint(jointIdx, desiredPosition, desiredVelocity, feedForwardAcceleration);
         feedbackControlCommand.setGains(jointIdx, gains.get(jointIdx));
         feedbackControlCommand.setWeightForSolver(jointIdx, weights.get(jointIdx).getDoubleValue());

         IntegerYoVariable numberOfPointsInQueue = this.numberOfPointsInQueue.get(jointIdx);
         IntegerYoVariable numberOfPointsInGenerator = this.numberOfPointsInGenerator.get(jointIdx);
         IntegerYoVariable numberOfPoints = this.numberOfPoints.get(jointIdx);
         numberOfPointsInQueue.set(pointQueues.get(jointIdx).size());
         numberOfPointsInGenerator.set(generator.getCurrentNumberOfWaypoints());
         numberOfPoints.set(numberOfPointsInQueue.getIntegerValue() + numberOfPointsInGenerator.getIntegerValue());
      }

      trajectoryDone.set(allDone);
   }

   private boolean fillAndReinitializeTrajectories(int jointIdx)
   {
      RecyclingArrayDeque<SimpleTrajectoryPoint1D> pointQueue = pointQueues.get(jointIdx);
      if (pointQueue.isEmpty())
         return true;

      MultipleWaypointsTrajectoryGenerator generator = jointTrajectoryGenerators.get(jointIdx);
      if (!generator.isEmpty())
      {
         generator.getLastWaypoint(lastPointAdded);
         generator.clear();
         generator.appendWaypoint(lastPointAdded);
      }

      int currentNumberOfWaypoints = generator.getCurrentNumberOfWaypoints();
      int pointsToAdd = maxPointsInGenerator - currentNumberOfWaypoints;
      for (int pointIdx = 0; pointIdx < pointsToAdd; pointIdx++)
      {
         if (pointQueue.isEmpty())
            break;

         SimpleTrajectoryPoint1D pointToAdd = pointQueue.pollFirst();
         generator.appendWaypoint(pointToAdd);
      }

      generator.initialize();
      return false;
   }

   public boolean handleTrajectoryCommand(JointspaceTrajectoryCommand<?, ?> command, double[] initialJointPositions)
   {
      if (!hasGains.getBooleanValue() || !hasWeights.getBooleanValue())
      {
         PrintTools.warn(warningPrefix + "Can not send joint trajectory commands. Do not have all weights and gains set.");
         return false;
      }

      if (!ControllerCommandValidationTools.checkOneDoFJointTrajectoryCommandList(jointsOriginal, command.getTrajectoryPointLists()))
         return false;

      if (!handleCommandInternal(command))
         return false;

      boolean override = command.getExecutionMode() == ExecutionMode.OVERRIDE;
      if (override || isEmpty())
      {
         overrideTrajectory();
         for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
         {
            SimpleTrajectoryPoint1DList trajectoryPoints = command.getJointTrajectoryPointList(jointIdx);
            if (trajectoryPoints.getTrajectoryPoint(0).getTime() > 1.0e-5)
               queueInitialPoint(initialJointPositions[jointIdx], jointIdx);
         }
      }

      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         SimpleTrajectoryPoint1DList trajectoryPoints = command.getJointTrajectoryPointList(jointIdx);
         for (int pointIdx = 0; pointIdx < trajectoryPoints.getNumberOfTrajectoryPoints(); pointIdx++)
         {
            SimpleTrajectoryPoint1D trajectoryPoint = trajectoryPoints.getTrajectoryPoint(pointIdx);
            if (!checkTime(trajectoryPoint.getTime(), jointIdx))
               return false;
            if (!queuePoint(trajectoryPoint, jointIdx))
               return false;
         }
      }

      return true;
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
         return false;

      SimpleTrajectoryPoint1D point = pointQueue.addLast();
      point.set(trajectoryPoint);
      return true;
   }

   private boolean queuePoint(double q, double qd, double t, int jointIdx)
   {
      RecyclingArrayDeque<SimpleTrajectoryPoint1D> pointQueue = pointQueues.get(jointIdx);
      if (atCapacityLimit(pointQueue))
         return false;

      SimpleTrajectoryPoint1D point = pointQueue.addLast();
      point.set(t, q, qd);
      return true;
   }

   private boolean atCapacityLimit(RecyclingArrayDeque<SimpleTrajectoryPoint1D> pointQueue)
   {
      if (pointQueue.size() >= maxPoints)
      {
         PrintTools.info(warningPrefix + "Reached maximum capacity of " + maxPoints + " can not execute trajectory.");
         return true;
      }
      return false;
   }

   private boolean checkTime(double time, int jointIdx)
   {
      boolean timeValid = time > getLastTrajectoryPointTime(jointIdx);
      if (!timeValid)
         PrintTools.warn(warningPrefix + "Time in trajectory must be strictly increasing.");
      return timeValid;
   }

   @Override
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

   public double getLastTrajectoryPointTime(int jointIdx)
   {
      if (isEmpty(jointIdx))
         return Double.NEGATIVE_INFINITY;

      RecyclingArrayDeque<SimpleTrajectoryPoint1D> pointQueue = pointQueues.get(jointIdx);
      MultipleWaypointsTrajectoryGenerator generator = jointTrajectoryGenerators.get(jointIdx);

      if (pointQueue.isEmpty())
         return generator.getLastWaypointTime();
      return pointQueue.peekLast().getTime();
   }

   private void overrideTrajectory()
   {
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         jointTrajectoryGenerators.get(jointIdx).clear();
         pointQueues.get(jointIdx).clear();
      }
   }

   @Override
   public boolean isEmpty()
   {
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         if (!isEmpty(jointIdx))
            return false;
      }
      return true;
   };

   public boolean isEmpty(int jointIdx)
   {
      RecyclingArrayDeque<SimpleTrajectoryPoint1D> pointQueue = pointQueues.get(jointIdx);
      MultipleWaypointsTrajectoryGenerator generator = jointTrajectoryGenerators.get(jointIdx);
      return pointQueue.isEmpty() && generator.isDone();
   };

   public double getJointDesiredPosition(int jointIdx)
   {
      return jointTrajectoryGenerators.get(jointIdx).getValue();
   }

   public double getJointDesiredVelocity(int jointIdx)
   {
      return jointTrajectoryGenerators.get(jointIdx).getVelocity();
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
