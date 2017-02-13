package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controlModules.ControllerCommandValidationTools;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.JointspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.tools.io.printing.PrintTools;

public class RigidBodyJointspaceControlState extends RigidBodyControlState
{
   private final YoVariableRegistry registry;
   private final String warningPrefix;

   private final Map<OneDoFJoint, MultipleWaypointsTrajectoryGenerator> jointTrajectoryGenerators = new HashMap<>();
   private final Map<OneDoFJoint, BooleanYoVariable> jointTrackingPosition = new HashMap<>();

   private final DoubleYoVariable weight;
   private final YoPIDGains gains;
   private final JointspaceFeedbackControlCommand feedbackControlCommand = new JointspaceFeedbackControlCommand();

   private final OneDoFJoint[] jointsOriginal;

   private final BooleanYoVariable trajectoryStopped;
   private final DoubleYoVariable receivedNewCommandTime;
   private final DoubleYoVariable yoTime;

   public RigidBodyJointspaceControlState(String bodyName, OneDoFJoint[] jointsToControl, YoPIDGains gains, DoubleYoVariable yoTime, YoVariableRegistry parentRegistry)
   {
      super(RigidBodyControlMode.JOINTSPACE);

      registry = new YoVariableRegistry(bodyName + "JointspaceControlModule");
      warningPrefix = getClass().getSimpleName() + " for " + bodyName + ": ";
      jointsOriginal = jointsToControl;
      this.gains = gains;
      this.yoTime = yoTime;

      String prefix = bodyName + "Jointspace";
      weight = new DoubleYoVariable(prefix + "Weight", registry);
      trajectoryStopped = new BooleanYoVariable(prefix + "TrajectoryStopped", registry);
      receivedNewCommandTime = new DoubleYoVariable(prefix + "ReceivedNewCommandTime", registry);

      for (int i = 0; i < jointsToControl.length; i++)
      {
         OneDoFJoint joint = jointsToControl[i];
         String jointName = joint.getName();
         MultipleWaypointsTrajectoryGenerator jointTrajectoryGenerator = new MultipleWaypointsTrajectoryGenerator(jointName, registry);
         jointTrajectoryGenerators.put(joint, jointTrajectoryGenerator);
         jointTrackingPosition.put(joint, new BooleanYoVariable(jointName + "TrackingPosition", registry));

         feedbackControlCommand.addJoint(joint, Double.NaN, Double.NaN, Double.NaN);
      }

      parentRegistry.addChild(registry);
   }

   public void setWeight(double weight)
   {
      this.weight.set(weight);
   }

   @Override
   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      trajectoryStopped.set(command.isStopAllTrajectory());
   }

   public void holdCurrent()
   {
      for (OneDoFJoint oneDoFJoint : jointsOriginal)
      {
         MultipleWaypointsTrajectoryGenerator trajectory = jointTrajectoryGenerators.get(oneDoFJoint);
         trajectory.clear();
         trajectory.appendWaypoint(0.0, oneDoFJoint.getQ(), 0.0);
         trajectory.initialize();
         jointTrackingPosition.get(oneDoFJoint).set(true);
      }
   }

   @Override
   public void doAction()
   {
      double timeInTrajectory = yoTime.getDoubleValue() - receivedNewCommandTime.getDoubleValue();

      for (int i = 0; i < jointsOriginal.length; i++)
      {
         OneDoFJoint joint = jointsOriginal[i];
         MultipleWaypointsTrajectoryGenerator jointTajectoryGenerator = jointTrajectoryGenerators.get(joint);

         BooleanYoVariable jointTracking = jointTrackingPosition.get(joint);
         if (!trajectoryStopped.getBooleanValue() && jointTracking.getBooleanValue())
         {
            jointTajectoryGenerator.compute(timeInTrajectory);
            jointTracking.set(!jointTajectoryGenerator.isDone());
         }

         double desiredPosition = jointTajectoryGenerator.getValue();
         double desiredVelocity = jointTajectoryGenerator.getVelocity();
         double feedForwardAcceleration = jointTajectoryGenerator.getAcceleration();
         feedbackControlCommand.setOneDoFJoint(i, desiredPosition, desiredVelocity, feedForwardAcceleration);
      }

      feedbackControlCommand.setGains(gains);
      feedbackControlCommand.setWeightForSolver(weight.getDoubleValue());
   }

   public boolean handleTrajectoryCommand(JointspaceTrajectoryCommand<?, ?> command, double[] initialJointPositions)
   {
      if (gains == null)
      {
         PrintTools.warn(warningPrefix + "Can not send trajectory commands. Joint gains are null.");
         return false;
      }

      if (!ControllerCommandValidationTools.checkOneDoFJointTrajectoryCommandList(jointsOriginal, command.getTrajectoryPointLists()))
      {
         if (jointsOriginal.length != command.getNumberOfJoints())
            PrintTools.warn(warningPrefix + "Wrong number of joints: expected " + jointsOriginal.length + " got " + command.getNumberOfJoints());
         else
            PrintTools.warn(warningPrefix + "Command out of joint limits!");
         return false;
      }

      int numberOfJoints = command.getNumberOfJoints();

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         OneDoFJoint joint = jointsOriginal[jointIndex];
         MultipleWaypointsTrajectoryGenerator trajectoryGenerator = jointTrajectoryGenerators.get(joint);
         trajectoryGenerator.clear();

         if (command.getJointTrajectoryPoint(jointIndex, 0).getTime() > 1.0e-5)
            trajectoryGenerator.appendWaypoint(0.0, initialJointPositions[jointIndex], 0.0);

         jointTrackingPosition.get(joint).set(true);
         SimpleTrajectoryPoint1DList jointTrajectory = command.getJointTrajectoryPointList(jointIndex);
         trajectoryGenerator.appendWaypoints(jointTrajectory);
         trajectoryGenerator.initialize();
      }

      receivedNewCommandTime.set(yoTime.getDoubleValue());
      trajectoryStopped.set(false);
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
