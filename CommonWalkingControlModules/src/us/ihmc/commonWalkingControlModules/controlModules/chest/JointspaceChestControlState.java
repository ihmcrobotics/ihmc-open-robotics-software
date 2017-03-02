package us.ihmc.commonWalkingControlModules.controlModules.chest;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controlModules.ControllerCommandValidationTools;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SpineTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.tools.io.printing.PrintTools;

public class JointspaceChestControlState extends ChestControlState
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final Map<OneDoFJoint, MultipleWaypointsTrajectoryGenerator> jointTrajectoryGenerators = new HashMap<>();
   private final Map<OneDoFJoint, BooleanYoVariable> jointTrackingPosition = new HashMap<>();

   private final DoubleYoVariable weight = new DoubleYoVariable("chestJointspaceWeight", registry);
   private final YoPIDGains gains;
   private final JointspaceFeedbackControlCommand feedbackControlCommand = new JointspaceFeedbackControlCommand();

   private final OneDoFJoint[] jointsOriginal;

   private final BooleanYoVariable isTrajectoryStopped = new BooleanYoVariable("isSpineTrajectoryStopped", registry);
   private final DoubleYoVariable receivedNewCommandTime = new DoubleYoVariable("receivedNewCommandTime", registry);
   private final DoubleYoVariable yoTime;

   public JointspaceChestControlState(OneDoFJoint[] spineJoints, YoPIDGains gains, DoubleYoVariable yoTime, YoVariableRegistry parentRegistry)
   {
      super(ChestControlMode.JOINTSPACE);

      this.jointsOriginal = spineJoints;
      this.gains = gains;
      this.yoTime = yoTime;

      for (int i = 0; i < spineJoints.length; i++)
      {
         OneDoFJoint joint = spineJoints[i];
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

   public boolean handleSpineTrajectoryCommand(SpineTrajectoryCommand command, double[] initialJointPositions)
   {
      if (gains == null)
      {
         PrintTools.warn("Can not send spine trajectory commands. Gains are null.");
         return false;
      }

      if (!ControllerCommandValidationTools.checkSpineTrajectoryCommand(jointsOriginal, command))
      {
         if (jointsOriginal.length != command.getNumberOfJoints())
            System.err.println("Wrong number of spine joints: expected " + jointsOriginal.length + " got " + command.getNumberOfJoints());
         else
            System.err.println("Spine command out of joint limits!");
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
      isTrajectoryStopped.set(false);
      return true;
   }

   @Override
   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      isTrajectoryStopped.set(command.isStopAllTrajectory());
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

   @Override
   public void doAction()
   {
      double timeInTrajectory = yoTime.getDoubleValue() - receivedNewCommandTime.getDoubleValue();

      for (int i = 0; i < jointsOriginal.length; i++)
      {
         OneDoFJoint joint = jointsOriginal[i];
         MultipleWaypointsTrajectoryGenerator jointTajectoryGenerator = jointTrajectoryGenerators.get(joint);

         BooleanYoVariable jointTracking = jointTrackingPosition.get(joint);
         if (!isTrajectoryStopped.getBooleanValue() && jointTracking.getBooleanValue())
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

   @Override
   public void doTransitionIntoAction()
   {
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

}
