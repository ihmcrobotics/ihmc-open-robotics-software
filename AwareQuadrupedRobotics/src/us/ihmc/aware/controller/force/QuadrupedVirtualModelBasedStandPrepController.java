package us.ihmc.aware.controller.force;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.aware.parameters.QuadrupedRuntimeEnvironment;
import us.ihmc.aware.params.ParameterMap;
import us.ihmc.aware.params.ParameterMapRepository;
import us.ihmc.aware.planning.ThreeDoFMinimumJerkTrajectory;
import us.ihmc.aware.vmc.QuadrupedVirtualModelController;
import us.ihmc.aware.vmc.QuadrupedVirtualModelControllerSettings;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointName;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.util.HeterogeneousMemoryPool;
import us.ihmc.robotics.controllers.EuclideanPositionController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;

public class QuadrupedVirtualModelBasedStandPrepController implements QuadrupedForceController
{
   private static final String PARAM_TRAJECTORY_TIME = "trajectoryTime";

   private static final String PARAM_JOINT_DAMPING = "jointDamping";

   private static final String PARAM_FOOT_POSITION_PROPORTIONAL_GAINS = "footPositionProportionalGains";
   private static final String PARAM_FOOT_POSITION_DERIVATIVE_GAINS = "footPositionDerivativeGains";
   private static final String PARAM_FOOT_POSITION_INTEGRAL_GAINS = "footPositionIntegralGains";
   private static final String PARAM_FOOT_POSITION_MAX_INTEGRAL_ERROR = "footPositionMaxIntegralError";

   private static final String PARAM_STANCE_LENGTH = "stanceLength";
   private static final String PARAM_STANCE_WIDTH = "stanceWidth";
   private static final String PARAM_STANCE_X_OFFSET = "stanceXOffset";
   private static final String PARAM_STANCE_Y_OFFSET = "stanceYOffset";
   private static final String PARAM_STANCE_HEIGHT = "stanceZOffset";

   private final QuadrupedRuntimeEnvironment environment;
   private final QuadrupedRobotParameters robotParameters;
   private final QuadrupedReferenceFrames referenceFrames;
   private final QuadrupedVirtualModelControllerSettings vmcSettings;
   private final QuadrupedVirtualModelController vmc;
   private final ParameterMap params;

   private final YoVariableRegistry registry = new YoVariableRegistry(
         QuadrupedVirtualModelBasedStandPrepController.class.getSimpleName());
   private final HeterogeneousMemoryPool pool = new HeterogeneousMemoryPool();

   private final TwistCalculator twistCalculator;
   private final QuadrantDependentList<RigidBody> footRigidBody = new QuadrantDependentList<>();
   private final QuadrantDependentList<ThreeDoFMinimumJerkTrajectory> footTrajectories = new QuadrantDependentList<>();
   private final QuadrantDependentList<EuclideanPositionController> footPidControllers = new QuadrantDependentList<>();

   // Estimates
   private final QuadrantDependentList<FramePoint> solePositionEstimates = new QuadrantDependentList<>();
   private final QuadrantDependentList<FrameVector> soleLinearVelocityEstimates = new QuadrantDependentList<>();

   // Setpoints
   private final QuadrantDependentList<FramePoint> solePositionSetpoints = new QuadrantDependentList<>();
   private final QuadrantDependentList<FrameVector> soleLinearVelocitySetpoints = new QuadrantDependentList<>();
   private final QuadrantDependentList<FrameVector> soleFeedForwardForceSetpoints = new QuadrantDependentList<>();
   private final QuadrantDependentList<FrameVector> soleVirtualForceSetpoints = new QuadrantDependentList<>();

   // YoVariable estimate
   private final QuadrantDependentList<YoFramePoint> yoSolePositionEstimates = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFrameVector> yoSoleLinearVelocityEstimates = new QuadrantDependentList<>();

   // YoVariable setpoints
   private final QuadrantDependentList<YoFramePoint> yoSolePositionSetpoints = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFrameVector> yoSoleLinearVelocitySetpoints = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFrameVector> yoSoleFeedForwardForceSetpoints = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFrameVector> yoSoleVirtualForceSetpoints = new QuadrantDependentList<>();

   private double trajectoryStartTime = 0.0;

   public QuadrupedVirtualModelBasedStandPrepController(QuadrupedRuntimeEnvironment environment,
         QuadrupedRobotParameters robotParameters, ParameterMapRepository parameterMapRepository)
   {
      this.environment = environment;
      this.robotParameters = robotParameters;
      this.referenceFrames = new QuadrupedReferenceFrames(environment.getFullRobotModel(),
            robotParameters.getJointMap(), robotParameters.getPhysicalProperties());
      this.vmcSettings = new QuadrupedVirtualModelControllerSettings();
      this.vmc = new QuadrupedVirtualModelController(environment.getFullRobotModel(), referenceFrames,
            robotParameters.getJointMap(), environment.getParentRegistry(),
            environment.getGraphicsListRegistryForDetachedOverhead());

      this.params = parameterMapRepository.get(QuadrupedVirtualModelBasedStandPrepController.class);
      params.setDefault(PARAM_TRAJECTORY_TIME, 1.0);

      params.setDefault(PARAM_JOINT_DAMPING, 15.0);

      params.setDefault(PARAM_FOOT_POSITION_PROPORTIONAL_GAINS, 10000.0, 10000.0, 10000.0);
      params.setDefault(PARAM_FOOT_POSITION_DERIVATIVE_GAINS, 100.0, 100.0, 100.0);
      params.setDefault(PARAM_FOOT_POSITION_INTEGRAL_GAINS, 0.0, 0.0, 0.0);
      params.setDefault(PARAM_FOOT_POSITION_MAX_INTEGRAL_ERROR, 0.0);

      params.setDefault(PARAM_STANCE_LENGTH, 1.0);
      params.setDefault(PARAM_STANCE_WIDTH, 0.5);
      params.setDefault(PARAM_STANCE_HEIGHT, 0.6);
      params.setDefault(PARAM_STANCE_X_OFFSET, 0.0);
      params.setDefault(PARAM_STANCE_Y_OFFSET, 0.0);

      SDFFullRobotModel fullRobotModel = environment.getFullRobotModel();
      this.twistCalculator = new TwistCalculator(referenceFrames.getWorldFrame(),
            environment.getFullRobotModel().getElevator());
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String jointBeforeFootName = robotParameters.getJointMap().getJointBeforeFootName(robotQuadrant);
         OneDoFJoint jointBeforeFoot = fullRobotModel.getOneDoFJointByName(jointBeforeFootName);
         footRigidBody.set(robotQuadrant, jointBeforeFoot.getSuccessor());
      }

      for (OneDoFJoint joint : fullRobotModel.getOneDoFJoints())
      {
         QuadrupedJointName jointName = robotParameters.getJointMap().getJointNameForSDFName(joint.getName());
         vmcSettings.setJointDamping(jointName, params.get(PARAM_JOINT_DAMPING));
      }

      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         footTrajectories.set(quadrant, new ThreeDoFMinimumJerkTrajectory());

         ReferenceFrame soleFrame = referenceFrames.getFootFrame(quadrant);
         EuclideanPositionController positionController = new EuclideanPositionController(
               "standPrep" + quadrant.getShortName(), soleFrame, environment.getControlDT(), registry);
         positionController.setProportionalGains(params.getVolatileArray(PARAM_FOOT_POSITION_PROPORTIONAL_GAINS));
         positionController.setDerivativeGains(params.getVolatileArray(PARAM_FOOT_POSITION_DERIVATIVE_GAINS));
         positionController.setIntegralGains(params.getVolatileArray(PARAM_FOOT_POSITION_INTEGRAL_GAINS),
               params.get(PARAM_FOOT_POSITION_MAX_INTEGRAL_ERROR));
         footPidControllers.set(quadrant, positionController);

         solePositionEstimates.set(quadrant, new FramePoint(referenceFrames.getWorldFrame()));
         soleLinearVelocityEstimates.set(quadrant, new FrameVector(referenceFrames.getWorldFrame()));

         solePositionSetpoints.set(quadrant, new FramePoint(referenceFrames.getWorldFrame()));
         soleLinearVelocitySetpoints.set(quadrant, new FrameVector(referenceFrames.getWorldFrame()));
         soleFeedForwardForceSetpoints.set(quadrant, new FrameVector(referenceFrames.getWorldFrame()));
         soleVirtualForceSetpoints.set(quadrant, new FrameVector(referenceFrames.getWorldFrame()));
      }

      // Initialize the YoVariables.
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         String prefix = quadrant.getCamelCaseNameForStartOfExpression();
         yoSolePositionEstimates.set(quadrant,
               new YoFramePoint(prefix + "SolePositionEstimate", referenceFrames.getWorldFrame(), registry));
         yoSoleLinearVelocityEstimates.set(quadrant,
               new YoFrameVector(prefix + "SoleLinearVelocityEstimate", referenceFrames.getWorldFrame(), registry));

         yoSolePositionSetpoints.set(quadrant,
               new YoFramePoint(prefix + "SolePositionSetpoint", referenceFrames.getWorldFrame(), registry));
         yoSoleLinearVelocitySetpoints.set(quadrant,
               new YoFrameVector(prefix + "SoleLinearVelocitySetpoint", referenceFrames.getWorldFrame(), registry));
         yoSoleFeedForwardForceSetpoints.set(quadrant,
               new YoFrameVector(prefix + "SoleFeedForwardForceSetpoint", referenceFrames.getWorldFrame(), registry));
         yoSoleVirtualForceSetpoints.set(quadrant,
               new YoFrameVector(prefix + "SoleVirtualForceSetpoint", referenceFrames.getWorldFrame(), registry));
      }

      environment.getParentRegistry().addChild(registry);
   }

   @Override
   public void onEntry()
   {
      referenceFrames.updateFrames();

      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         ThreeDoFMinimumJerkTrajectory trajectory = footTrajectories.get(quadrant);
         ReferenceFrame soleFrame = referenceFrames.getFootFrame(quadrant);

         FramePoint solePosition = pool.lease(FramePoint.class);
         solePosition.setToZero(soleFrame);
         solePosition.changeFrame(referenceFrames.getBodyFrame());

         FramePoint finalPosition = computeFinalSolePosition(quadrant);

         trajectory.setMoveParameters(solePosition, finalPosition, params.get(PARAM_TRAJECTORY_TIME));
      }

      // Show the VMC visualizations.
      vmc.setVisible(false);
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         vmc.setSoleVirtualForceVisible(quadrant, true);
      }

      trajectoryStartTime = environment.getRobotTimestamp().getDoubleValue();
   }

   @Override
   public QuadrupedForceControllerEvent process()
   {
      pool.evict();

      readYoVariables();
      updateEstimates();
      updateSetpoints();
      writeYoVariables();

      return isMotionExpired() ? QuadrupedForceControllerEvent.STARTING_POSE_REACHED : null;
   }

   @Override
   public void onExit()
   {
      // Hide the VMC visualizations.
      vmc.setVisible(false);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         vmc.setSoleVirtualForceVisible(robotQuadrant, false);
      }
   }

   private void updateEstimates()
   {
      referenceFrames.updateFrames();
      twistCalculator.compute();

      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         ReferenceFrame soleFrame = referenceFrames.getFootFrame(quadrant);

         // Compute sole positions and linear velocities.
         solePositionEstimates.get(quadrant).setToZero(soleFrame);

         Twist twist = pool.lease(Twist.class);
         twistCalculator.packTwistOfBody(twist, footRigidBody.get(quadrant));
         twist.changeFrame(soleFrame);

         twist.packLinearPart(soleLinearVelocityEstimates.get(quadrant));
      }
   }

   private void updateSetpoints()
   {
      double timeInTrajectory = getTimeInTrajectory();

      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         ReferenceFrame soleFrame = referenceFrames.getFootFrame(quadrant);

         // Compute the position setpoint along the minimum jerk trajectory.
         ThreeDoFMinimumJerkTrajectory trajectory = footTrajectories.get(quadrant);
         EuclideanPositionController positionController = footPidControllers.get(quadrant);
         trajectory.computeTrajectory(timeInTrajectory);

         trajectory.getPosition(solePositionSetpoints.get(quadrant));
         solePositionSetpoints.get(quadrant).changeFrame(soleFrame);

         soleLinearVelocitySetpoints.get(quadrant).setToZero(soleFrame);
         soleFeedForwardForceSetpoints.get(quadrant).setToZero(soleFrame);

         // Compute the virtual force setpoint via the PID controller.
         soleVirtualForceSetpoints.get(quadrant).setToZero(soleFrame);
         positionController.compute(soleVirtualForceSetpoints.get(quadrant), solePositionSetpoints.get(quadrant),
               soleLinearVelocitySetpoints.get(quadrant), soleLinearVelocityEstimates.get(quadrant),
               soleFeedForwardForceSetpoints.get(quadrant));

         // Forward virtual forces to VMC controller to set joint torques.
         vmc.setSoleVirtualForce(quadrant, soleVirtualForceSetpoints.get(quadrant));
         vmc.compute(robotParameters.getQuadrupedJointLimits(), vmcSettings);
      }
   }

   private void readYoVariables()
   {

   }

   private void writeYoVariables()
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         yoSolePositionEstimates.get(quadrant).setAndMatchFrame(solePositionEstimates.get(quadrant));
         yoSoleLinearVelocityEstimates.get(quadrant).setAndMatchFrame(soleLinearVelocityEstimates.get(quadrant));
         yoSolePositionSetpoints.get(quadrant).setAndMatchFrame(solePositionSetpoints.get(quadrant));
         yoSoleLinearVelocitySetpoints.get(quadrant).setAndMatchFrame(soleLinearVelocitySetpoints.get(quadrant));
         yoSoleFeedForwardForceSetpoints.get(quadrant).setAndMatchFrame(soleFeedForwardForceSetpoints.get(quadrant));
         yoSoleVirtualForceSetpoints.get(quadrant).setAndMatchFrame(soleVirtualForceSetpoints.get(quadrant));
      }
   }

   private double getTimeInTrajectory()
   {
      return environment.getRobotTimestamp().getDoubleValue() - trajectoryStartTime;
   }

   private FramePoint computeFinalSolePosition(RobotQuadrant quadrant)
   {
      FramePoint finalPosition = pool.lease(FramePoint.class);
      finalPosition.setToZero(referenceFrames.getBodyFrame());

      if (quadrant.isQuadrantInFront())
      {
         finalPosition.add(params.get(PARAM_STANCE_LENGTH) / 2, 0.0, 0.0);
      }
      else
      {
         finalPosition.add(-params.get(PARAM_STANCE_LENGTH) / 2, 0.0, 0.0);
      }

      if (quadrant.isQuadrantOnRightSide())
      {
         finalPosition.add(0.0, -params.get(PARAM_STANCE_WIDTH) / 2, 0.0);
      }
      else
      {
         finalPosition.add(0.0, params.get(PARAM_STANCE_WIDTH) / 2, 0.0);
      }

      finalPosition.add(params.get(PARAM_STANCE_X_OFFSET), params.get(PARAM_STANCE_Y_OFFSET), -params.get(
            PARAM_STANCE_HEIGHT));

      return finalPosition;
   }

   private boolean isMotionExpired()
   {
      return getTimeInTrajectory() > params.get(PARAM_TRAJECTORY_TIME);
   }
}
