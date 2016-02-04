package us.ihmc.aware.controller.position;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.aware.controller.QuadrupedController;
import us.ihmc.aware.parameters.QuadrupedRuntimeEnvironment;
import us.ihmc.aware.params.ParameterMap;
import us.ihmc.aware.params.ParameterMapRepository;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointName;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.trajectories.MinimumJerkTrajectory;

/**
 * A controller that will track the minimum jerk trajectory to bring joints to a preparatory pose.
 */
public class QuadrupedPositionStandPrepController implements QuadrupedPositionController
{
   public static final String PARAM_TRAJECTORY_TIME = "trajectoryTime";

   private final QuadrupedRobotParameters parameters;
   private final ParameterMap params;
   private final FullRobotModel fullRobotModel;
   private final double dt;

   private final List<MinimumJerkTrajectory> trajectories;

   /**
    * The time from the beginning of the current preparation trajectory in seconds.
    */
   private double timeInTrajectory = 0.0;

   public QuadrupedPositionStandPrepController(QuadrupedRuntimeEnvironment environment, QuadrupedRobotParameters parameters,
         ParameterMapRepository paramMapRepository)
   {
      this.parameters = parameters;
      this.params = paramMapRepository.get(QuadrupedPositionStandPrepController.class);
      this.fullRobotModel = environment.getFullRobotModel();
      this.dt = environment.getControlDT();

      this.trajectories = new ArrayList<>(fullRobotModel.getOneDoFJoints().length);
      for (int i = 0; i < fullRobotModel.getOneDoFJoints().length; i++)
      {
         trajectories.add(new MinimumJerkTrajectory());
      }

      // Configure default parameters
      params.setDefault(PARAM_TRAJECTORY_TIME, 1.0);
   }

   @Override
   public void onEntry()
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
               params.get(PARAM_TRAJECTORY_TIME));
      }

      // This is a new trajectory. We start at time 0.
      timeInTrajectory = 0.0;
   }

   @Override
   public QuadrupedPositionControllerEvent process()
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

      return isMotionExpired() ? QuadrupedPositionControllerEvent.STARTING_POSE_REACHED : null;
   }

   @Override
   public void onExit()
   {
   }

   private boolean isMotionExpired()
   {
      return timeInTrajectory > params.get(PARAM_TRAJECTORY_TIME);
   }
}

