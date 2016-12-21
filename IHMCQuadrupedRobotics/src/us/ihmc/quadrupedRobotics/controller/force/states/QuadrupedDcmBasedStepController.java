package us.ihmc.quadrupedRobotics.controller.force.states;

import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.*;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.robotics.dataStructures.parameter.DoubleArrayParameter;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.quadrupedRobotics.planning.*;
import us.ihmc.quadrupedRobotics.planning.stepStream.QuadrupedStepStream;
import us.ihmc.quadrupedRobotics.planning.trajectory.PiecewiseReverseDcmTrajectory;
import us.ihmc.quadrupedRobotics.planning.trajectory.QuadrupedPiecewiseConstantCopTrajectory;
import us.ihmc.quadrupedRobotics.planning.trajectory.ThreeDoFMinimumJerkTrajectory;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPostureInputProviderInterface;
import us.ihmc.quadrupedRobotics.util.YoPreallocatedList;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.OrientationFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedDcmBasedStepController implements QuadrupedController, QuadrupedStepTransitionCallback
{
   private static int STEP_SEQUENCE_CAPACITY = 100;
   private final QuadrupedPostureInputProviderInterface postureProvider;
   private final QuadrupedStepStream stepStream;
   private final DoubleYoVariable robotTimestamp;
   private final double controlDT;
   private final double gravity;
   private final double mass;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   // parameters
   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleArrayParameter bodyOrientationProportionalGainsParameter = parameterFactory
         .createDoubleArray("bodyOrientationProportionalGains", 5000, 5000, 5000);
   private final DoubleArrayParameter bodyOrientationDerivativeGainsParameter = parameterFactory
         .createDoubleArray("bodyOrientationDerivativeGains", 750, 750, 750);
   private final DoubleArrayParameter bodyOrientationIntegralGainsParameter = parameterFactory.createDoubleArray("bodyOrientationIntegralGains", 0, 0, 0);
   private final DoubleParameter bodyOrientationMaxIntegralErrorParameter = parameterFactory.createDouble("bodyOrientationMaxIntegralError", 0);
   private final DoubleArrayParameter comPositionProportionalGainsParameter = parameterFactory.createDoubleArray("comPositionProportionalGains", 0, 0, 5000);
   private final DoubleArrayParameter comPositionDerivativeGainsParameter = parameterFactory.createDoubleArray("comPositionDerivativeGains", 0, 0, 750);
   private final DoubleArrayParameter comPositionIntegralGainsParameter = parameterFactory.createDoubleArray("comPositionIntegralGains", 0, 0, 0);
   private final DoubleParameter comPositionMaxIntegralErrorParameter = parameterFactory.createDouble("comPositionMaxIntegralError", 0);
   private final DoubleParameter comPositionGravityCompensationParameter = parameterFactory.createDouble("comPositionGravityCompensation", 1);
   private final DoubleArrayParameter comForceCommandWeightsParameter = parameterFactory.createDoubleArray("comForceCommandWeights", 1, 1, 1);
   private final DoubleArrayParameter comTorqueCommandWeightsParameter = parameterFactory.createDoubleArray("comTorqueCommandWeights", 1, 1, 1);
   private final DoubleArrayParameter dcmPositionProportionalGainsParameter = parameterFactory.createDoubleArray("dcmPositionProportionalGains", 1, 1, 0);
   private final DoubleArrayParameter dcmPositionDerivativeGainsParameter = parameterFactory.createDoubleArray("dcmPositionDerivativeGains", 0, 0, 0);
   private final DoubleArrayParameter dcmPositionIntegralGainsParameter = parameterFactory.createDoubleArray("dcmPositionIntegralGains", 0, 0, 0);
   private final DoubleParameter dcmPositionMaxIntegralErrorParameter = parameterFactory.createDouble("dcmPositionMaxIntegralError", 0);
   private final DoubleParameter dcmPositionStepAdjustmentGainParameter = parameterFactory.createDouble("dcmPositionStepAdjustmentGain", 1.5);
   private final DoubleParameter vrpPositionRateLimitParameter = parameterFactory.createDouble("vrpPositionRateLimit", Double.MAX_VALUE);
   private final DoubleParameter jointDampingParameter = parameterFactory.createDouble("jointDamping", 1);
   private final DoubleParameter jointPositionLimitDampingParameter = parameterFactory.createDouble("jointPositionLimitDamping", 10);
   private final DoubleParameter jointPositionLimitStiffnessParameter = parameterFactory.createDouble("jointPositionLimitStiffness", 100);
   private final DoubleParameter initialTransitionDurationParameter = parameterFactory.createDouble("initialTransitionDuration", 0.5);
   private final DoubleParameter haltTransitionDurationParameter = parameterFactory.createDouble("haltTransitionDuration", 1.0);
   private final DoubleParameter minimumStepClearanceParameter = parameterFactory.createDouble("minimumStepClearance", 0.075);
   private final DoubleParameter maximumStepStrideParameter = parameterFactory.createDouble("maximumStepStride", 1.0);
   private final DoubleParameter coefficientOfFrictionParameter = parameterFactory.createDouble("coefficientOfFriction", 0.5);

   // frames
   private final ReferenceFrame supportFrame;
   private final ReferenceFrame worldFrame;

   // model
   private final LinearInvertedPendulumModel lipModel;

   // feedback controllers
   private final FramePoint dcmPositionEstimate;
   private final DivergentComponentOfMotionEstimator dcmPositionEstimator;
   private final DivergentComponentOfMotionController.Setpoints dcmPositionControllerSetpoints;
   private final DivergentComponentOfMotionController dcmPositionController;
   private final QuadrupedComPositionController.Setpoints comPositionControllerSetpoints;
   private final QuadrupedComPositionController comPositionController;
   private final QuadrupedBodyOrientationController.Setpoints bodyOrientationControllerSetpoints;
   private final QuadrupedBodyOrientationController bodyOrientationController;
   private final QuadrantDependentList<QuadrupedFootStateMachine> footStateMachine;

   // task space controller
   private final QuadrupedTaskSpaceEstimator.Estimates taskSpaceEstimates;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final QuadrupedTaskSpaceController.Commands taskSpaceControllerCommands;
   private final QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings;
   private final QuadrupedTaskSpaceController taskSpaceController;

   // step planner
   private static int MAXIMUM_STEP_QUEUE_SIZE = 100;
   private final GroundPlaneEstimator groundPlaneEstimator;
   private final QuadrantDependentList<YoFramePoint> groundPlanePositions;
   private final QuadrupedTimedContactSequence timedContactSequence;
   private final QuadrupedPiecewiseConstantCopTrajectory piecewiseConstanceCopTrajectory;
   private final PiecewiseReverseDcmTrajectory dcmTrajectory;
   private final ThreeDoFMinimumJerkTrajectory dcmTransitionTrajectory;
   private final FramePoint dcmPositionWaypoint;
   private final YoFrameVector instantaneousStepAdjustment;
   private final YoFrameVector accumulatedStepAdjustment;
   private final QuadrupedStepCrossoverProjection crossoverProjection;
   private final FrameOrientation bodyOrientationReference;
   private final OrientationFrame bodyOrientationReferenceFrame;
   private final FramePoint stepGoalPosition;
   private final YoPreallocatedList<YoQuadrupedTimedStep> stepSequence;

   // inputs
   private final DoubleYoVariable haltTime = new DoubleYoVariable("haltTime", registry);
   private final BooleanYoVariable haltFlag = new BooleanYoVariable("haltFlag", registry);
   private final BooleanYoVariable onLiftOffTriggered = new BooleanYoVariable("onLiftOffTriggered", registry);
   private final BooleanYoVariable onTouchDownTriggered = new BooleanYoVariable("onTouchDownTriggered", registry);

   public QuadrupedDcmBasedStepController(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedForceControllerToolbox controllerToolbox,
         QuadrupedPostureInputProviderInterface postureProvider, QuadrupedStepStream stepStream)
   {
      this.postureProvider = postureProvider;
      this.stepStream = stepStream;
      this.robotTimestamp = runtimeEnvironment.getRobotTimestamp();
      this.controlDT = runtimeEnvironment.getControlDT();
      this.gravity = 9.81;
      this.mass = runtimeEnvironment.getFullRobotModel().getTotalMass();

      // frames
      QuadrupedReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      worldFrame = ReferenceFrame.getWorldFrame();

      // model
      lipModel = controllerToolbox.getLinearInvertedPendulumModel();

      // feedback controllers
      dcmPositionEstimate = new FramePoint();
      dcmPositionEstimator = controllerToolbox.getDcmPositionEstimator();
      dcmPositionControllerSetpoints = new DivergentComponentOfMotionController.Setpoints();
      dcmPositionController = controllerToolbox.getDcmPositionController();
      comPositionControllerSetpoints = new QuadrupedComPositionController.Setpoints();
      comPositionController = controllerToolbox.getComPositionController();
      bodyOrientationControllerSetpoints = new QuadrupedBodyOrientationController.Setpoints();
      bodyOrientationController = controllerToolbox.getBodyOrientationController();
      footStateMachine = controllerToolbox.getFootStateMachine();

      // task space controllers
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimator.Estimates();
      taskSpaceEstimator = controllerToolbox.getTaskSpaceEstimator();
      taskSpaceControllerCommands = new QuadrupedTaskSpaceController.Commands();
      taskSpaceControllerSettings = new QuadrupedTaskSpaceController.Settings();
      taskSpaceController = controllerToolbox.getTaskSpaceController();

      // step planner
      groundPlaneEstimator = controllerToolbox.getGroundPlaneEstimator();
      groundPlanePositions = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         groundPlanePositions.set(robotQuadrant, new YoFramePoint(robotQuadrant.getCamelCaseName() + "GroundPlanePosition", worldFrame, registry));
      }
      timedContactSequence = new QuadrupedTimedContactSequence(4, 2 * STEP_SEQUENCE_CAPACITY);
      piecewiseConstanceCopTrajectory = new QuadrupedPiecewiseConstantCopTrajectory(timedContactSequence.capacity());
      dcmTrajectory = new PiecewiseReverseDcmTrajectory(STEP_SEQUENCE_CAPACITY, gravity, postureProvider.getComPositionInput().getZ());
      dcmTransitionTrajectory = new ThreeDoFMinimumJerkTrajectory();
      dcmPositionWaypoint = new FramePoint();
      instantaneousStepAdjustment = new YoFrameVector("instantaneousStepAdjustment", worldFrame, registry);
      accumulatedStepAdjustment = new YoFrameVector("accumulatedStepAdjustment", worldFrame, registry);
      crossoverProjection = new QuadrupedStepCrossoverProjection(referenceFrames.getBodyZUpFrame(), minimumStepClearanceParameter.get(),
            maximumStepStrideParameter.get());
      bodyOrientationReference = new FrameOrientation();
      bodyOrientationReferenceFrame = new OrientationFrame(bodyOrientationReference);
      stepGoalPosition = new FramePoint();
      stepSequence = new YoPreallocatedList<>("stepSequence", registry, MAXIMUM_STEP_QUEUE_SIZE,
            new YoPreallocatedList.DefaultElementFactory<YoQuadrupedTimedStep>()
            {
               @Override
               public YoQuadrupedTimedStep createDefaultElement(String prefix, YoVariableRegistry registry)
               {
                  return new YoQuadrupedTimedStep(prefix, registry);
               }
            });

      runtimeEnvironment.getParentRegistry().addChild(registry);
   }

   private void updateGains()
   {
      dcmPositionController.setVrpPositionRateLimit(vrpPositionRateLimitParameter.get());
      dcmPositionController.getGains().setProportionalGains(dcmPositionProportionalGainsParameter.get());
      dcmPositionController.getGains().setIntegralGains(dcmPositionIntegralGainsParameter.get(), dcmPositionMaxIntegralErrorParameter.get());
      dcmPositionController.getGains().setDerivativeGains(dcmPositionDerivativeGainsParameter.get());
      comPositionController.getGains().setProportionalGains(comPositionProportionalGainsParameter.get());
      comPositionController.getGains().setIntegralGains(comPositionIntegralGainsParameter.get(), comPositionMaxIntegralErrorParameter.get());
      comPositionController.getGains().setDerivativeGains(comPositionDerivativeGainsParameter.get());
      bodyOrientationController.getGains().setProportionalGains(bodyOrientationProportionalGainsParameter.get());
      bodyOrientationController.getGains().setIntegralGains(bodyOrientationIntegralGainsParameter.get(), bodyOrientationMaxIntegralErrorParameter.get());
      bodyOrientationController.getGains().setDerivativeGains(bodyOrientationDerivativeGainsParameter.get());
      taskSpaceControllerSettings.getContactForceLimits().setCoefficientOfFriction(coefficientOfFrictionParameter.get());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointDamping(jointDampingParameter.get());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitDamping(jointPositionLimitDampingParameter.get());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitStiffness(jointPositionLimitStiffnessParameter.get());
      taskSpaceControllerSettings.getContactForceOptimizationSettings().setComForceCommandWeights(comForceCommandWeightsParameter.get());
      taskSpaceControllerSettings.getContactForceOptimizationSettings().setComTorqueCommandWeights(comTorqueCommandWeightsParameter.get());
   }

   private void updateEstimates()
   {
      // update model
      lipModel.setComHeight(postureProvider.getComPositionInput().getZ());

      // update task space estimates
      taskSpaceEstimator.compute(taskSpaceEstimates);

      // update dcm estimate
      dcmPositionEstimator.compute(dcmPositionEstimate, taskSpaceEstimates.getComVelocity());
   }

   private void updateSetpoints()
   {
      // trigger step events
      triggerStepEvents();

      // update ground plane estimate
      groundPlaneEstimator.clearContactPoints();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         groundPlaneEstimator.addContactPoint(groundPlanePositions.get(robotQuadrant).getFrameTuple().getPoint());
      }
      groundPlaneEstimator.compute();

      // update desired horizontal com forces
      computeDcmSetpoints();
      dcmPositionController.compute(taskSpaceControllerCommands.getComForce(), dcmPositionControllerSetpoints, dcmPositionEstimate);
      taskSpaceControllerCommands.getComForce().changeFrame(supportFrame);

      // update desired com position, velocity, and vertical force
      comPositionControllerSetpoints.getComPosition().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComPosition().set(postureProvider.getComPositionInput());
      comPositionControllerSetpoints.getComVelocity().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComVelocity().set(postureProvider.getComVelocityInput());
      comPositionControllerSetpoints.getComForceFeedforward().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComForceFeedforward().set(taskSpaceControllerCommands.getComForce());
      comPositionControllerSetpoints.getComForceFeedforward().setZ(comPositionGravityCompensationParameter.get() * mass * gravity);
      comPositionController.compute(taskSpaceControllerCommands.getComForce(), comPositionControllerSetpoints, taskSpaceEstimates);

      // update desired body orientation, angular velocity, and torque
      stepStream.getBodyOrientation(bodyOrientationReference);
      bodyOrientationReference.changeFrame(bodyOrientationReferenceFrame.getParent());
      bodyOrientationReferenceFrame.setOrientationAndUpdate(bodyOrientationReference);
      bodyOrientationControllerSetpoints.getBodyOrientation().changeFrame(bodyOrientationReferenceFrame);
      bodyOrientationControllerSetpoints.getBodyOrientation().set(postureProvider.getBodyOrientationInput());
      bodyOrientationControllerSetpoints.getBodyOrientation().changeFrame(worldFrame);
      double bodyOrientationYaw = bodyOrientationControllerSetpoints.getBodyOrientation().getYaw();
      double bodyOrientationPitch = bodyOrientationControllerSetpoints.getBodyOrientation().getPitch();
      double bodyOrientationRoll = bodyOrientationControllerSetpoints.getBodyOrientation().getRoll();
      bodyOrientationControllerSetpoints.getBodyOrientation()
            .setYawPitchRoll(bodyOrientationYaw, bodyOrientationPitch + groundPlaneEstimator.getPitch(bodyOrientationYaw), bodyOrientationRoll);
      bodyOrientationControllerSetpoints.getBodyAngularVelocity().setToZero();
      bodyOrientationControllerSetpoints.getComTorqueFeedforward().setToZero();
      bodyOrientationController.compute(taskSpaceControllerCommands.getComTorque(), bodyOrientationControllerSetpoints, taskSpaceEstimates);

      // update desired contact state and sole forces
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         footStateMachine.get(robotQuadrant).compute(taskSpaceControllerCommands.getSoleForce(robotQuadrant), taskSpaceEstimates);
         taskSpaceControllerSettings.setContactState(robotQuadrant, footStateMachine.get(robotQuadrant).getContactState());
      }

      // update joint setpoints
      taskSpaceController.compute(taskSpaceControllerSettings, taskSpaceControllerCommands);

      // update step plan
      computeStepSequence();

      // update step adjustment
      computeStepAdjustment();

      // update dcm trajectory
      computeDcmTrajectory();

      // update accumulated step adjustment
      if (onLiftOffTriggered.getBooleanValue())
      {
         onLiftOffTriggered.set(false);
      }
      if (onTouchDownTriggered.getBooleanValue())
      {
         onTouchDownTriggered.set(false);
         accumulatedStepAdjustment.add(instantaneousStepAdjustment.getFrameTuple().getVector());
         accumulatedStepAdjustment.setZ(0);
      }
   }

   private void computeDcmTrajectory()
   {
      // compute piecewise constant center of pressure plan
      double currentTime = robotTimestamp.getDoubleValue();
      QuadrantDependentList<FramePoint> currentSolePosition = taskSpaceEstimates.getSolePosition();
      QuadrantDependentList<ContactState> currentContactState = taskSpaceControllerSettings.getContactState();
      timedContactSequence.update(stepSequence, currentSolePosition, currentContactState, currentTime);
      piecewiseConstanceCopTrajectory.initializeTrajectory(timedContactSequence);

      // compute dcm trajectory with final boundary constraint
      int numberOfIntervals = piecewiseConstanceCopTrajectory.getNumberOfIntervals();
      dcmPositionWaypoint.setIncludingFrame(piecewiseConstanceCopTrajectory.getCopPositionAtStartOfInterval(numberOfIntervals - 1));
      dcmPositionWaypoint.changeFrame(ReferenceFrame.getWorldFrame());
      dcmPositionWaypoint.add(0, 0, lipModel.getComHeight());
      dcmTrajectory.setComHeight(lipModel.getComHeight());
      dcmTrajectory.initializeTrajectory(numberOfIntervals, piecewiseConstanceCopTrajectory.getTimeAtStartOfInterval(),
            piecewiseConstanceCopTrajectory.getCopPositionAtStartOfInterval(),
            piecewiseConstanceCopTrajectory.getTimeAtStartOfInterval(numberOfIntervals - 1), dcmPositionWaypoint);
   }

   private void computeDcmSetpoints()
   {
      if (robotTimestamp.getDoubleValue() <= dcmTransitionTrajectory.getEndTime())
      {
         dcmTransitionTrajectory.computeTrajectory(robotTimestamp.getDoubleValue());
         dcmTransitionTrajectory.getPosition(dcmPositionControllerSetpoints.getDcmPosition());
         dcmTransitionTrajectory.getVelocity(dcmPositionControllerSetpoints.getDcmVelocity());
      }
      else
      {
         dcmTrajectory.computeTrajectory(robotTimestamp.getDoubleValue());
         dcmTrajectory.getPosition(dcmPositionControllerSetpoints.getDcmPosition());
         dcmTrajectory.getVelocity(dcmPositionControllerSetpoints.getDcmVelocity());
      }
   }

   private void triggerStepEvents()
   {
      for (int i = 0; i < stepSequence.size(); i++)
      {
         RobotQuadrant robotQuadrant = stepSequence.get(i).getRobotQuadrant();
         double currentTime = robotTimestamp.getDoubleValue();
         double startTime = stepSequence.get(i).getTimeInterval().getStartTime();
         double endTime = stepSequence.get(i).getTimeInterval().getEndTime();
         if (startTime < currentTime && currentTime < endTime)
         {
            footStateMachine.get(robotQuadrant).triggerStep(stepSequence.get(i));
         }
      }
   }

   private void computeStepSequence()
   {
      // update step plan
      stepSequence.clear();
      for (int i = 0; i < stepStream.getSteps().size(); i++)
      {
         if (!haltFlag.getBooleanValue() || stepStream.getSteps().get(i).getTimeInterval().getEndTime() < haltTime.getDoubleValue())
         {
            stepSequence.add();
            stepSequence.get(stepSequence.size() - 1).set(stepStream.getSteps().get(i));
            stepSequence.get(stepSequence.size() - 1).getGoalPosition(stepGoalPosition);
            stepGoalPosition.changeFrame(worldFrame);
            stepGoalPosition.add(accumulatedStepAdjustment.getFrameTuple());
            stepSequence.get(stepSequence.size() - 1).setGoalPosition(stepGoalPosition);
         }
      }
   }

   private void computeStepAdjustment()
   {
      if (robotTimestamp.getDoubleValue() > dcmTransitionTrajectory.getEndTime())
      {
         // compute step adjustment for ongoing steps (proportional to dcm tracking error)
         FramePoint dcmPositionSetpoint = dcmPositionControllerSetpoints.getDcmPosition();
         dcmPositionSetpoint.changeFrame(instantaneousStepAdjustment.getReferenceFrame());
         dcmPositionEstimate.changeFrame(instantaneousStepAdjustment.getReferenceFrame());
         instantaneousStepAdjustment.set(dcmPositionEstimate);
         instantaneousStepAdjustment.sub(dcmPositionSetpoint);
         instantaneousStepAdjustment.scale(dcmPositionStepAdjustmentGainParameter.get());
         instantaneousStepAdjustment.setZ(0);

         // adjust nominal step goal positions in foot state machine
         for (int i = 0; i < stepSequence.size(); i++)
         {
            RobotQuadrant robotQuadrant = stepSequence.get(i).getRobotQuadrant();
            double currentTime = robotTimestamp.getDoubleValue();
            double startTime = stepSequence.get(i).getTimeInterval().getStartTime();
            double endTime = stepSequence.get(i).getTimeInterval().getEndTime();
            if (startTime < currentTime && currentTime < endTime)
            {
               stepSequence.get(i).getGoalPosition(stepGoalPosition);
               stepGoalPosition.changeFrame(worldFrame);
               stepGoalPosition.add(instantaneousStepAdjustment.getFrameTuple());
               crossoverProjection.project(stepGoalPosition, taskSpaceEstimates.getSolePosition(), robotQuadrant);
               groundPlaneEstimator.projectZ(stepGoalPosition.getPoint());
               footStateMachine.get(robotQuadrant).adjustStep(stepGoalPosition);
            }
         }
      }
   }

   @Override
   public void onLiftOff(RobotQuadrant thisStepQuadrant)
   {
      // update ground plane estimate
      groundPlanePositions.get(thisStepQuadrant).setAndMatchFrame(taskSpaceEstimates.getSolePosition(thisStepQuadrant));
      onLiftOffTriggered.set(true);
   }

   @Override
   public void onTouchDown(RobotQuadrant thisStepQuadrant)
   {
      onTouchDownTriggered.set(true);
   }

   @Override
   public void onEntry()
   {
      // initialize step stream
      stepStream.onEntry();

      // initialize state
      haltFlag.set(false);
      onLiftOffTriggered.set(false);
      onTouchDownTriggered.set(false);
      accumulatedStepAdjustment.setToZero();

      // initialize estimates
      lipModel.setComHeight(postureProvider.getComPositionInput().getZ());
      updateEstimates();

      // initialize feedback controllers
      dcmPositionControllerSetpoints.initialize(dcmPositionEstimate);
      dcmPositionController.reset();
      comPositionControllerSetpoints.initialize(taskSpaceEstimates);
      comPositionController.reset();
      bodyOrientationControllerSetpoints.initialize(taskSpaceEstimates);
      bodyOrientationController.reset();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         footStateMachine.get(robotQuadrant).registerStepTransitionCallback(this);
         footStateMachine.get(robotQuadrant).reset();
      }

      // initialize task space controller
      taskSpaceControllerSettings.initialize();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         taskSpaceControllerSettings.getContactForceOptimizationSettings().setContactForceCommandWeights(robotQuadrant, 0.0, 0.0, 0.0);
         taskSpaceControllerSettings.setContactState(robotQuadrant, ContactState.IN_CONTACT);
      }
      taskSpaceController.reset();

      // initialize ground plane
      groundPlaneEstimator.clearContactPoints();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         groundPlanePositions.get(robotQuadrant).setAndMatchFrame(taskSpaceEstimates.getSolePosition(robotQuadrant));
         groundPlaneEstimator.addContactPoint(groundPlanePositions.get(robotQuadrant).getFrameTuple().getPoint());
      }
      groundPlaneEstimator.compute();

      // initialize timed contact sequence
      timedContactSequence.initialize();

      // compute step plan
      computeStepSequence();

      // compute step adjustment
      computeStepAdjustment();

      double currentTime = robotTimestamp.getDoubleValue();
      if (stepSequence.size() > 0 && stepSequence.get(stepSequence.size() - 1).getTimeInterval().getEndTime() > currentTime)
      {
         // compute dcm trajectory
         computeDcmTrajectory();
         double transitionEndTime = piecewiseConstanceCopTrajectory.getTimeAtStartOfInterval(1);
         double transitionStartTime = Math.max(currentTime, transitionEndTime - initialTransitionDurationParameter.get());
         dcmTrajectory.computeTrajectory(transitionEndTime);
         dcmTrajectory.getPosition(dcmPositionWaypoint);
         dcmTransitionTrajectory.initializeTrajectory(dcmPositionEstimate, dcmPositionWaypoint, transitionStartTime, transitionEndTime);
         computeDcmSetpoints();
      }
   }

   @Override
   public ControllerEvent process()
   {
      double currentTime = robotTimestamp.getDoubleValue();
      if (stepSequence.size() == 0 || stepSequence.get(stepSequence.size() - 1).getTimeInterval().getEndTime() < currentTime)
      {
         return ControllerEvent.DONE;
      }
      stepStream.process();
      updateGains();
      updateEstimates();
      updateSetpoints();
      return null;
   }

   @Override
   public void onExit()
   {
      // clean up step stream
      stepStream.onExit();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         footStateMachine.get(robotQuadrant).registerStepTransitionCallback(null);
      }
   }

   public void halt()
   {
      if (haltFlag.getBooleanValue() == false)
      {
         haltFlag.set(true);
         haltTime.set(robotTimestamp.getDoubleValue() + haltTransitionDurationParameter.get());
      }
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }
}
