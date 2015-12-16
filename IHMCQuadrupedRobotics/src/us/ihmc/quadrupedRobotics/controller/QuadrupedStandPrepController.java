package us.ihmc.quadrupedRobotics.controller;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointName;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;
import us.ihmc.robotics.trajectories.MinimumJerkTrajectory;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;

/**
 * A controller that will track the minimum jerk trajectory to bring joints to a preparatory pose.
 */
public class QuadrupedStandPrepController extends QuadrupedController
{
   private final QuadrupedRobotParameters parameters;
   private final SDFFullRobotModel fullRobotModel;
   private final double dt;

   private final List<MinimumJerkTrajectory> trajectories;

   /**
    * The time from the beginning of the current preparation trajectory in seconds.
    */
   private double timeInTrajectory = 0.0;

   public QuadrupedStandPrepController(final QuadrupedRobotParameters parameters, final SDFFullRobotModel fullRobotModel, final double dt)
   {
      super(QuadrupedControllerState.STAND_PREP);

      this.parameters = parameters;
      this.fullRobotModel = fullRobotModel;
      this.dt = dt;

      this.trajectories = new ArrayList<MinimumJerkTrajectory>(fullRobotModel.getOneDoFJoints().length);
      for (int i = 0; i < fullRobotModel.getOneDoFJoints().length; i++)
      {
         trajectories.add(new MinimumJerkTrajectory());
      }
   }

   @Override
   public void doAction()
   {
      fullRobotModel.updateFrames();

      for (int i = 0; i < fullRobotModel.getOneDoFJoints().length; i++)
      {
         OneDoFJoint joint = fullRobotModel.getOneDoFJoints()[i];
         MinimumJerkTrajectory trajectory = trajectories.get(i);

         trajectory.computeTrajectory(timeInTrajectory);
         joint.setqDesired(trajectory.getPosition());
      }

      timeInTrajectory += dt;
   }

   @Override
   public void doTransitionIntoAction()
   {
      for (int i = 0; i < fullRobotModel.getOneDoFJoints().length; i++)
      {
         OneDoFJoint joint = fullRobotModel.getOneDoFJoints()[i];
         joint.setUnderPositionControl(true);

         QuadrupedJointName jointId = parameters.getJointMap().getJointNameForSDFName(joint.getName());
         double desiredPosition = parameters.getQuadrupedInitialPositionParameters().getInitialPosition(jointId);

         // Start the trajectory from the current pos/vel/acc.
         MinimumJerkTrajectory trajectory = trajectories.get(i);
         trajectory.setMoveParameters(joint.getQ(), joint.getQd(), joint.getQdd(), desiredPosition, 0.0, 0.0,
               parameters.getQuadrupedStandPrepParameters().getTrajectoryTime());
      }

      // This is a new trajectory. We start at time 0.
      timeInTrajectory = 0.0;
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }

   @Override
   public RobotMotionStatus getMotionStatus()
   {
      // If the trajectory has been exhausted, then the robot is standing.
      if (timeInTrajectory > parameters.getQuadrupedStandPrepParameters().getTrajectoryTime())
      {
         return RobotMotionStatus.STANDING;
      }

      return RobotMotionStatus.IN_MOTION;
   }
}

/**
 * A transition condition that transitions when the stand prep controller has completed its trajectory.
 */
class QuadrupedStandPrepControllerExitCondition implements StateTransitionCondition
{
   private final QuadrupedStandPrepController controller;

   public QuadrupedStandPrepControllerExitCondition(QuadrupedStandPrepController controller)
   {
      this.controller = controller;
   }

   @Override
   public boolean checkCondition()
   {
      return controller.getMotionStatus() == RobotMotionStatus.STANDING;
   }
}