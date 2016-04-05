package us.ihmc.aware.controller.force;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.aware.config.DoubleArrayParameter;
import us.ihmc.aware.config.DoubleParameter;
import us.ihmc.aware.config.ParameterFactory;
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
   private final ParameterFactory parameterFactory = new ParameterFactory(QuadrupedVirtualModelBasedStandPrepController.class.getName());
   private final DoubleParameter trajectoryTimeParameter = parameterFactory.createDouble("TrajectoryTime", 1.0);
   private final DoubleParameter jointDampingParameter = parameterFactory.createDouble("jointDamping", 15.0);
   private final DoubleArrayParameter footPositionProportionalGainsParameter = parameterFactory.createDoubleArray("footPositionProportionalGains", 10000.0, 10000.0, 10000.0);
   private final DoubleArrayParameter footPositionDerivativeGainsParameter = parameterFactory.createDoubleArray("footPositionDerivativeGains", 100.0, 100.0, 100.0);
   private final DoubleArrayParameter footPositionIntegralGainsParameter = parameterFactory.createDoubleArray("footPositionIntegralGains", 0.0, 0.0, 0.0);
   private final DoubleParameter footPositionMaxIntegralErrorParameter = parameterFactory.createDouble("footPositionMaxIntegralError", 0.0);
   private final DoubleParameter stanceLengthParameter = parameterFactory.createDouble("stanceLength", 1.0);
   private final DoubleParameter stanceWidthParameter = parameterFactory.createDouble("stanceWidth", 0.5);
   private final DoubleParameter stanceHeightParameter = parameterFactory.createDouble("stanceHeight", 0.60);
   private final DoubleParameter stanceXOffsetParameter = parameterFactory.createDouble("stanceXOffset", 0.05);
   private final DoubleParameter stanceYOffsetParameter = parameterFactory.createDouble("stanceYOffset", 0.0);

   private final QuadrupedRuntimeEnvironment environment;
   private final QuadrupedRobotParameters robotParameters;
   private final QuadrupedReferenceFrames referenceFrames;
   private final QuadrupedVirtualModelControllerSettings vmcSettings;
   private final QuadrupedVirtualModelController vmc;

   private final YoVariableRegistry registry = new YoVariableRegistry(QuadrupedVirtualModelBasedStandPrepController.class.getSimpleName());
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

   public QuadrupedVirtualModelBasedStandPrepController(QuadrupedRuntimeEnvironment environment, QuadrupedRobotParameters robotParameters)
   {
      this.environment = environment;
      this.robotParameters = robotParameters;
      this.referenceFrames = new QuadrupedReferenceFrames(environment.getFullRobotModel(), robotParameters.getJointMap(),
            robotParameters.getPhysicalProperties());
      this.vmcSettings = new QuadrupedVirtualModelControllerSettings();
      this.vmc = new QuadrupedVirtualModelController(environment.getFullRobotModel(), referenceFrames, robotParameters.getJointMap(), registry);

      SDFFullRobotModel fullRobotModel = environment.getFullRobotModel();
      this.twistCalculator = new TwistCalculator(referenceFrames.getWorldFrame(), environment.getFullRobotModel().getElevator());
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String jointBeforeFootName = robotParameters.getJointMap().getJointBeforeFootName(robotQuadrant);
         OneDoFJoint jointBeforeFoot = fullRobotModel.getOneDoFJointByName(jointBeforeFootName);
         footRigidBody.set(robotQuadrant, jointBeforeFoot.getSuccessor());
      }

      for (OneDoFJoint joint : fullRobotModel.getOneDoFJoints())
      {
         QuadrupedJointName jointName = robotParameters.getJointMap().getJointNameForSDFName(joint.getName());
         vmcSettings.setJointDamping(jointName, jointDampingParameter.get());
      }

      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         footTrajectories.set(quadrant, new ThreeDoFMinimumJerkTrajectory());

         ReferenceFrame soleFrame = referenceFrames.getFootFrame(quadrant);
         EuclideanPositionController positionController = new EuclideanPositionController("standPrep" + quadrant.getShortName(), soleFrame,
               environment.getControlDT(), registry);
         positionController.setProportionalGains(footPositionProportionalGainsParameter.get());
         positionController.setDerivativeGains(footPositionDerivativeGainsParameter.get());
         positionController.setIntegralGains(footPositionIntegralGainsParameter.get(), footPositionMaxIntegralErrorParameter.get());
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
         yoSolePositionEstimates.set(quadrant, new YoFramePoint(prefix + "SolePositionEstimate", referenceFrames.getWorldFrame(), registry));
         yoSoleLinearVelocityEstimates.set(quadrant, new YoFrameVector(prefix + "SoleLinearVelocityEstimate", referenceFrames.getWorldFrame(), registry));

         yoSolePositionSetpoints.set(quadrant, new YoFramePoint(prefix + "SolePositionSetpoint", referenceFrames.getWorldFrame(), registry));
         yoSoleLinearVelocitySetpoints.set(quadrant, new YoFrameVector(prefix + "SoleLinearVelocitySetpoint", referenceFrames.getWorldFrame(), registry));
         yoSoleFeedForwardForceSetpoints.set(quadrant, new YoFrameVector(prefix + "SoleFeedForwardForceSetpoint", referenceFrames.getWorldFrame(), registry));
         yoSoleVirtualForceSetpoints.set(quadrant, new YoFrameVector(prefix + "SoleVirtualForceSetpoint", referenceFrames.getWorldFrame(), registry));
      }

      environment.getParentRegistry().addChild(registry);
   }

   @Override
   public void onEntry()
   {
      updateEstimates();

      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         ThreeDoFMinimumJerkTrajectory trajectory = footTrajectories.get(quadrant);
         ReferenceFrame soleFrame = referenceFrames.getFootFrame(quadrant);

         FramePoint solePosition = pool.lease(FramePoint.class);
         solePosition.setToZero(soleFrame);
         solePosition.changeFrame(referenceFrames.getBodyFrame());

         FramePoint finalPosition = computeFinalSolePosition(quadrant);

         trajectory.initializeTrajectory(solePosition, finalPosition, trajectoryTimeParameter.get());
      }

      // Show the VMC visualizations.
      vmc.setVisible(true);
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         vmc.setSoleVirtualForceVisible(quadrant, true);
         vmc.setSoleContactForceVisible(quadrant, false);
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
         twistCalculator.getTwistOfBody(twist, footRigidBody.get(quadrant));
         twist.changeFrame(soleFrame);

         twist.getLinearPart(soleLinearVelocityEstimates.get(quadrant));
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
         positionController.compute(soleVirtualForceSetpoints.get(quadrant), solePositionSetpoints.get(quadrant), soleLinearVelocitySetpoints.get(quadrant),
               soleLinearVelocityEstimates.get(quadrant), soleFeedForwardForceSetpoints.get(quadrant));

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

      finalPosition.add(quadrant.getEnd().negateIfHindEnd(stanceLengthParameter.get() / 2.0), 0.0, 0.0);
      finalPosition.add(0.0, quadrant.getSide().negateIfRightSide(stanceWidthParameter.get() / 2.0), 0.0);

      finalPosition.add(stanceXOffsetParameter.get(), stanceYOffsetParameter.get(), -stanceHeightParameter.get());

      return finalPosition;
   }

   private boolean isMotionExpired()
   {
      return getTimeInTrajectory() > trajectoryTimeParameter.get();
   }
}
