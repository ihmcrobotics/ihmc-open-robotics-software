package us.ihmc.aware.controller.force;

import us.ihmc.aware.controller.force.toolbox.QuadrupedSolePositionController;
import us.ihmc.aware.controller.force.toolbox.QuadrupedTaskSpaceController;
import us.ihmc.aware.controller.force.toolbox.QuadrupedTaskSpaceEstimator;
import us.ihmc.aware.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.aware.model.QuadrupedRuntimeEnvironment;
import us.ihmc.aware.params.DoubleArrayParameter;
import us.ihmc.aware.params.DoubleParameter;
import us.ihmc.aware.params.ParameterFactory;
import us.ihmc.aware.planning.ContactState;
import us.ihmc.aware.planning.trajectory.ThreeDoFMinimumJerkTrajectory;
import us.ihmc.aware.util.TimeInterval;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedVirtualModelBasedStandPrepController implements QuadrupedForceController
{
   private final ParameterFactory parameterFactory = new ParameterFactory(getClass());
   private final DoubleParameter trajectoryTimeParameter = parameterFactory.createDouble("trajectoryTime", 1.0);
   private final DoubleParameter jointDampingParameter = parameterFactory.createDouble("jointDamping", 15.0);
   private final DoubleParameter stanceLengthParameter = parameterFactory.createDouble("stanceLength", 1.0);
   private final DoubleParameter stanceWidthParameter = parameterFactory.createDouble("stanceWidth", 0.5);
   private final DoubleParameter stanceHeightParameter = parameterFactory.createDouble("stanceHeight", 0.60);
   private final DoubleParameter stanceXOffsetParameter = parameterFactory.createDouble("stanceXOffset", 0.05);
   private final DoubleParameter stanceYOffsetParameter = parameterFactory.createDouble("stanceYOffset", 0.0);
   private final DoubleArrayParameter solePositionProportionalGainsParameter = parameterFactory.createDoubleArray("solePositionProportionalGains", 20000, 20000, 20000);
   private final DoubleArrayParameter solePositionDerivativeGainsParameter = parameterFactory.createDoubleArray("solePositionDerivativeGains", 200, 200, 200);
   private final DoubleArrayParameter solePositionIntegralGainsParameter = parameterFactory.createDoubleArray("solePositionIntegralGains", 0, 0, 0);
   private final DoubleParameter solePositionMaxIntegralErrorParameter = parameterFactory.createDouble("solePositionMaxIntegralError", 0);

   private final DoubleYoVariable robotTime;
   private final QuadrupedReferenceFrames referenceFrames;
   private final YoVariableRegistry registry = new YoVariableRegistry(QuadrupedVirtualModelBasedStandPrepController.class.getSimpleName());

   // Sole trajectories
   private final TimeInterval trajectoryTimeInterval = new TimeInterval();
   private final FramePoint finalSolePosition = new FramePoint();
   private final QuadrantDependentList<ThreeDoFMinimumJerkTrajectory> solePositionTrajectories = new QuadrantDependentList<>();

   // Task space controllers
   private final QuadrupedTaskSpaceEstimator.Estimates taskSpaceEstimates;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final QuadrupedTaskSpaceController.Commands taskSpaceControllerCommands;
   private final QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings;
   private final QuadrupedTaskSpaceController taskSpaceController;
   private final QuadrupedSolePositionController solePositionController;
   private final QuadrupedSolePositionController.Setpoints solePositionControllerSetpoints;

   public QuadrupedVirtualModelBasedStandPrepController(QuadrupedRuntimeEnvironment environment, QuadrupedForceControllerToolbox controllerToolbox)
   {
      this.robotTime = environment.getRobotTimestamp();
      this.referenceFrames = controllerToolbox.getReferenceFrames();

      // Task space controllers
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimator.Estimates();
      taskSpaceEstimator = controllerToolbox.getTaskSpaceEstimator();
      taskSpaceControllerCommands = new QuadrupedTaskSpaceController.Commands();
      taskSpaceControllerSettings = new QuadrupedTaskSpaceController.Settings();
      taskSpaceController = controllerToolbox.getTaskSpaceController();
      solePositionController = controllerToolbox.getSolePositionController();
      solePositionControllerSetpoints = new QuadrupedSolePositionController.Setpoints();

      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         solePositionTrajectories.set(quadrant, new ThreeDoFMinimumJerkTrajectory());

      }

      environment.getParentRegistry().addChild(registry);
   }

   @Override
   public void onEntry()
   {
      updateEstimates();

      double currentTime = robotTime.getDoubleValue();
      trajectoryTimeInterval.setInterval(0, trajectoryTimeParameter.get());
      trajectoryTimeInterval.shiftInterval(currentTime);

      // Initialize sole trajectories
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         ThreeDoFMinimumJerkTrajectory trajectory = solePositionTrajectories.get(quadrant);

         FramePoint initialSolePosition = taskSpaceEstimates.getSolePosition(quadrant);
         initialSolePosition.changeFrame(referenceFrames.getBodyFrame());

         computeFinalSolePosition(quadrant, finalSolePosition);
         finalSolePosition.changeFrame(referenceFrames.getBodyFrame());

         trajectory.initializeTrajectory(initialSolePosition, finalSolePosition, trajectoryTimeInterval);
      }

      // Initialize sole position controller
      solePositionControllerSetpoints.initialize(taskSpaceEstimates);
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         solePositionController.getGains(quadrant).setProportionalGains(solePositionProportionalGainsParameter.get());
         solePositionController.getGains(quadrant).setIntegralGains(solePositionIntegralGainsParameter.get(), solePositionMaxIntegralErrorParameter.get());
         solePositionController.getGains(quadrant).setDerivativeGains(solePositionDerivativeGainsParameter.get());
      }
      solePositionController.reset();

      // Initialize task space controller
      taskSpaceControllerSettings.initialize();
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointDamping(jointDampingParameter.get());
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         taskSpaceControllerSettings.setContactState(robotQuadrant, ContactState.NO_CONTACT);
      }
      taskSpaceController.reset();
   }

   @Override
   public QuadrupedForceControllerEvent process()
   {
      updateEstimates();
      updateSetpoints();

      return isMotionExpired() ? QuadrupedForceControllerEvent.STARTING_POSE_REACHED : null;
   }

   @Override
   public void onExit()
   {
   }

   private void updateEstimates()
   {
      taskSpaceEstimator.compute(taskSpaceEstimates);
   }

   private void updateSetpoints()
   {
      double currentTime = robotTime.getDoubleValue();

      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         // Compute the sole position setpoint along the minimum jerk trajectory.
         ThreeDoFMinimumJerkTrajectory trajectory = solePositionTrajectories.get(quadrant);
         trajectory.computeTrajectory(currentTime);

         trajectory.getPosition(solePositionControllerSetpoints.getSolePosition(quadrant));
         // trajectory.getVelocity(solePositionControllerSetpoints.getSoleLinearVelocity(quadrant));
      }

      solePositionController.compute(taskSpaceControllerCommands.getSoleForce(), solePositionControllerSetpoints, taskSpaceEstimates);
      taskSpaceController.compute(taskSpaceControllerSettings, taskSpaceControllerCommands);
   }

   private void computeFinalSolePosition(RobotQuadrant quadrant, FramePoint finalSolePosition)
   {
      finalSolePosition.setToZero(referenceFrames.getBodyFrame());

      finalSolePosition.add(quadrant.getEnd().negateIfHindEnd(stanceLengthParameter.get() / 2.0), 0.0, 0.0);
      finalSolePosition.add(0.0, quadrant.getSide().negateIfRightSide(stanceWidthParameter.get() / 2.0), 0.0);

      finalSolePosition.add(stanceXOffsetParameter.get(), stanceYOffsetParameter.get(), -stanceHeightParameter.get());
   }

   private boolean isMotionExpired()
   {
      double currentTime = robotTime.getDoubleValue();
      return currentTime > trajectoryTimeInterval.getEndTime();
   }
}
