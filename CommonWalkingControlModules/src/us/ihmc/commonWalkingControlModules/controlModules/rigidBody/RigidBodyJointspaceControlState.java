package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controlModules.ControllerCommandValidationTools;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.JointspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1D;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.tools.io.printing.PrintTools;

public class RigidBodyJointspaceControlState extends RigidBodyControlState
{
   public static final int maxPoints = 200;
   public static final int maxPointsInGenerator = 5;

   // TODO: get rid of hash maps and use a simple array instead
   private final Map<OneDoFJoint, MultipleWaypointsTrajectoryGenerator> jointTrajectoryGenerators = new HashMap<>();
   private final Map<OneDoFJoint, RecyclingArrayList<SimpleTrajectoryPoint1D>> pointQueues = new HashMap<>();

   private final Map<OneDoFJoint, IntegerYoVariable> numberOfPointsInQueue = new HashMap<>();
   private final Map<OneDoFJoint, IntegerYoVariable> numberOfPointsInGenerator = new HashMap<>();
   private final Map<OneDoFJoint, IntegerYoVariable> numberOfPoints = new HashMap<>();

   private final SimpleTrajectoryPoint1D lastPointAdded = new SimpleTrajectoryPoint1D();

   private final DoubleYoVariable weight;
   private final YoPIDGains gains;
   private final JointspaceFeedbackControlCommand feedbackControlCommand = new JointspaceFeedbackControlCommand();

   private final OneDoFJoint[] jointsOriginal;
   private final int numberOfJoints;

   public RigidBodyJointspaceControlState(String bodyName, OneDoFJoint[] jointsToControl, YoPIDGains gains, DoubleYoVariable yoTime, YoVariableRegistry parentRegistry)
   {
      super(RigidBodyControlMode.JOINTSPACE, bodyName, yoTime);

      this.gains = gains;
      this.jointsOriginal = jointsToControl;
      numberOfJoints = jointsOriginal.length;

      String prefix = bodyName + "Jointspace";
      weight = new DoubleYoVariable(prefix + "Weight", registry);

      for (int i = 0; i < jointsToControl.length; i++)
      {
         OneDoFJoint joint = jointsToControl[i];
         String jointName = joint.getName();
         MultipleWaypointsTrajectoryGenerator jointTrajectoryGenerator = new MultipleWaypointsTrajectoryGenerator(jointName, maxPointsInGenerator, registry);
         jointTrajectoryGenerators.put(joint, jointTrajectoryGenerator);

         RecyclingArrayList<SimpleTrajectoryPoint1D> pointQueue = new RecyclingArrayList<>(maxPoints, SimpleTrajectoryPoint1D.class);
         pointQueue.clear();
         pointQueues.put(joint, pointQueue);

         feedbackControlCommand.addJoint(joint, Double.NaN, Double.NaN, Double.NaN);

         IntegerYoVariable numberOfPointsInQueue = new IntegerYoVariable(prefix + "_" + jointName + "_numberOfPointsInQueue", registry);
         IntegerYoVariable numberOfPointsInGenerator = new IntegerYoVariable(prefix + "_" + jointName + "_numberOfPointsInGenerator", registry);
         IntegerYoVariable numberOfPoints = new IntegerYoVariable(prefix + "_" + jointName + "_numberOfPoints", registry);
         this.numberOfPointsInQueue.put(joint, numberOfPointsInQueue);
         this.numberOfPointsInGenerator.put(joint, numberOfPointsInGenerator);
         this.numberOfPoints.put(joint, numberOfPoints);
      }

      parentRegistry.addChild(registry);
   }

   public void setWeight(double weight)
   {
      this.weight.set(weight);
   }

   public void holdCurrent()
   {
      overrideTrajectory();
      resetLastCommandId();

      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         OneDoFJoint joint = jointsOriginal[jointIdx];
         queueInitialPoint(joint.getQ(), joint);
      }

      trajectoryStopped.set(false);
      trajectoryDone.set(false);
   }

   @Override
   public void doAction()
   {
      double timeInTrajectory = getTimeInTrajectory();
      boolean allDone = true;

      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         OneDoFJoint joint = jointsOriginal[jointIdx];
         MultipleWaypointsTrajectoryGenerator generator = jointTrajectoryGenerators.get(joint);

         if (!trajectoryDone.getBooleanValue() && generator.isDone())
            allDone = fillAndReinitializeTrajectories(joint) && allDone;
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

         IntegerYoVariable numberOfPointsInQueue = this.numberOfPointsInQueue.get(joint);
         IntegerYoVariable numberOfPointsInGenerator = this.numberOfPointsInGenerator.get(joint);
         IntegerYoVariable numberOfPoints = this.numberOfPoints.get(joint);
         numberOfPointsInQueue.set(pointQueues.get(joint).size());
         numberOfPointsInGenerator.set(generator.getCurrentNumberOfWaypoints());
         numberOfPoints.set(numberOfPointsInQueue.getIntegerValue() + numberOfPointsInGenerator.getIntegerValue());
      }

      trajectoryDone.set(allDone);
      feedbackControlCommand.setGains(gains);
      feedbackControlCommand.setWeightForSolver(weight.getDoubleValue());
   }

   private boolean fillAndReinitializeTrajectories(OneDoFJoint joint)
   {
      RecyclingArrayList<SimpleTrajectoryPoint1D> pointQueue = pointQueues.get(joint);
      if (pointQueue.isEmpty())
         return true;

      MultipleWaypointsTrajectoryGenerator generator = jointTrajectoryGenerators.get(joint);
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

         SimpleTrajectoryPoint1D pointToAdd = pointQueue.get(0);
         generator.appendWaypoint(pointToAdd);
         pointQueue.remove(0); // TODO: replace with queue
      }

      generator.initialize();
      return false;
   }

   public boolean handleTrajectoryCommand(JointspaceTrajectoryCommand<?, ?> command, double[] initialJointPositions)
   {
      if (gains == null)
      {
         PrintTools.warn(warningPrefix + "Can not send trajectory commands. Joint gains are null.");
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
               queueInitialPoint(initialJointPositions[jointIdx], jointsOriginal[jointIdx]);
         }
      }

      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         OneDoFJoint joint = jointsOriginal[jointIdx];
         SimpleTrajectoryPoint1DList trajectoryPoints = command.getJointTrajectoryPointList(jointIdx);
         for (int pointIdx = 0; pointIdx < trajectoryPoints.getNumberOfTrajectoryPoints(); pointIdx++)
         {
            SimpleTrajectoryPoint1D trajectoryPoint = trajectoryPoints.getTrajectoryPoint(pointIdx);
            if (!checkTime(trajectoryPoint.getTime(), joint))
               return false;
            if (!queuePoint(trajectoryPoint, joint))
               return false;
         }
      }

      return true;
   }

   private void queueInitialPoint(double initialPosition, OneDoFJoint joint)
   {
      RecyclingArrayList<SimpleTrajectoryPoint1D> pointQueue = pointQueues.get(joint);
      SimpleTrajectoryPoint1D point = pointQueue.add();
      point.set(0.0, initialPosition, 0.0);
   }

   private boolean queuePoint(SimpleTrajectoryPoint1D trajectoryPoint, OneDoFJoint joint)
   {
      RecyclingArrayList<SimpleTrajectoryPoint1D> pointQueue = pointQueues.get(joint);
      if (atCapacityLimit(pointQueue))
         return false;

      SimpleTrajectoryPoint1D point = pointQueue.add();
      point.set(trajectoryPoint);
      return true;
   }

   private boolean atCapacityLimit(RecyclingArrayList<SimpleTrajectoryPoint1D> pointQueue)
   {
      if (pointQueue.size() >= maxPoints)
      {
         PrintTools.info(warningPrefix + "Reached maximum capacity of " + maxPoints + " can not execute trajectory.");
         return true;
      }
      return false;
   }

   private boolean checkTime(double time, OneDoFJoint joint)
   {
      boolean timeValid = time > getLastTrajectoryPointTime(joint);
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
         double jointLastTime = getLastTrajectoryPointTime(jointsOriginal[jointIdx]);
         lastTrajectoryPointTime = Math.max(lastTrajectoryPointTime, jointLastTime);
      }
      return lastTrajectoryPointTime;
   }

   public double getLastTrajectoryPointTime(OneDoFJoint joint)
   {
      if (isEmpty(joint))
         return Double.NEGATIVE_INFINITY;

      RecyclingArrayList<SimpleTrajectoryPoint1D> pointQueue = pointQueues.get(joint);
      MultipleWaypointsTrajectoryGenerator generator = jointTrajectoryGenerators.get(joint);

      if (pointQueue.isEmpty())
         return generator.getLastWaypointTime();
      return pointQueue.getLast().getTime();
   }

   private void overrideTrajectory()
   {
      for (OneDoFJoint joint : jointsOriginal)
      {
         jointTrajectoryGenerators.get(joint).clear();
         pointQueues.get(joint).clear();
      }
   }

   @Override
   public boolean isEmpty()
   {
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         if (!isEmpty(jointsOriginal[jointIdx]))
            return false;
      }
      return true;
   };

   public boolean isEmpty(OneDoFJoint joint)
   {
      RecyclingArrayList<SimpleTrajectoryPoint1D> pointQueue = pointQueues.get(joint);
      MultipleWaypointsTrajectoryGenerator generator = jointTrajectoryGenerators.get(joint);
      return pointQueue.isEmpty() && generator.isDone();
   };

   public double getJointDesiredPosition(OneDoFJoint joint)
   {
      return jointTrajectoryGenerators.get(joint).getValue();
   }

   public double getJointDesiredVelocity(OneDoFJoint joint)
   {
      return jointTrajectoryGenerators.get(joint).getVelocity();
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
