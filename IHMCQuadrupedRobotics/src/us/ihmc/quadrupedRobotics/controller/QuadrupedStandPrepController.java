package us.ihmc.quadrupedRobotics.controller;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.quadrupedRobotics.controller.state.QuadrupedControllerState;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointName;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.trajectories.YoMinimumJerkTrajectory;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;

/**
 * A controller that will track the minimum jerk trajectory to bring joints to a preparatory pose.
 */
public class QuadrupedStandPrepController extends QuadrupedController
{
   private final QuadrupedRobotParameters parameters;
   private final FullRobotModel fullRobotModel;
   private final OneDoFJoint[] oneDoFJoints;
   private final double dt;

   private final List<YoMinimumJerkTrajectory> trajectories;

   /**
    * The time from the beginning of the current preparation trajectory in seconds.
    */
   private DoubleYoVariable timeInTrajectory;

   public QuadrupedStandPrepController(final QuadrupedRobotParameters parameters, final FullRobotModel fullRobotModel, final double dt,
         YoVariableRegistry yoVariableRegistry)
   {
      super(QuadrupedControllerState.STAND_PREP);

      YoVariableRegistry registry = new YoVariableRegistry("QuadrupedStandPrepController");

      this.parameters = parameters;
      this.fullRobotModel = fullRobotModel;
      this.oneDoFJoints = fullRobotModel.getOneDoFJoints();
      this.dt = dt;

      this.trajectories = new ArrayList<>(oneDoFJoints.length);
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         trajectories.add(new YoMinimumJerkTrajectory(oneDoFJoints[i].getName() + "StandPrepTrajectory", registry));
      }

      timeInTrajectory = new DoubleYoVariable("QuadrupedStandPrepControllerTrajectoryTime", registry);
      
      yoVariableRegistry.addChild(registry);
   }

   @Override
   public void doAction()
   {
      fullRobotModel.updateFrames();

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         YoMinimumJerkTrajectory trajectory = trajectories.get(i);

         double time = MathTools.clipToMinMax(timeInTrajectory.getDoubleValue(), 0.0, parameters.getQuadrupedStandPrepParameters().getTrajectoryTime());
         trajectory.computeTrajectory(time);
         joint.setqDesired(trajectory.getPosition());
      }

      timeInTrajectory.add(dt);
   }

   @Override
   public void doTransitionIntoAction()
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         joint.setUnderPositionControl(true);

         QuadrupedJointName jointId = parameters.getJointMap().getJointNameForSDFName(joint.getName());
         double desiredPosition = parameters.getQuadrupedInitialPositionParameters().getInitialPosition(jointId);

         // Start the trajectory from the current pos/vel/acc.
         YoMinimumJerkTrajectory trajectory = trajectories.get(i);
         trajectory.setParams(joint.getQ(), joint.getQd(), joint.getQdd(), desiredPosition, 0.0, 0.0, 0.0,
               parameters.getQuadrupedStandPrepParameters().getTrajectoryTime());
      }

      // This is a new trajectory. We start at time 0.
      timeInTrajectory.set(0.0);
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }

   @Override
   public RobotMotionStatus getMotionStatus()
   {
      // If the trajectory has been exhausted, then the robot is standing.
      if (timeInTrajectory.getDoubleValue() > parameters.getQuadrupedStandPrepParameters().getTrajectoryTime())
      {
         return RobotMotionStatus.STANDING;
      }

      return RobotMotionStatus.IN_MOTION;
   }
}
