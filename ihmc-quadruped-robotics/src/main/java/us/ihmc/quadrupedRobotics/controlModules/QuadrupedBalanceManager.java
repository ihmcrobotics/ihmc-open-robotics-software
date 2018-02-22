package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.*;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.quadrupedRobotics.planning.*;
import us.ihmc.quadrupedRobotics.planning.trajectory.PiecewiseReverseDcmTrajectory;
import us.ihmc.quadrupedRobotics.planning.trajectory.QuadrupedPiecewiseConstantCopTrajectory;
import us.ihmc.quadrupedRobotics.planning.trajectory.ThreeDoFMinimumJerkTrajectory;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPostureInputProviderInterface;
import us.ihmc.robotics.dataStructures.parameter.DoubleArrayParameter;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.robotics.lists.GenericTypeBuilder;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

public class QuadrupedBalanceManager
{
   private static final int STEP_SEQUENCE_CAPACITY = 100;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble robotTimestamp;
   private final double gravity;
   private final double mass;

   private final ReferenceFrame supportFrame;

   private final LinearInvertedPendulumModel linearInvertedPendulumModel;
   private final DivergentComponentOfMotionEstimator dcmPositionEstimator;
   private final DivergentComponentOfMotionController dcmPositionController;
   private final QuadrupedComPositionController comPositionController;
   private final QuadrupedComPositionController.Setpoints comPositionControllerSetpoints;

   private final QuadrupedPostureInputProviderInterface postureProvider;

   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleArrayParameter comPositionProportionalGainsParameter = parameterFactory.createDoubleArray("comPositionProportionalGains", 0, 0, 5000);
   private final DoubleArrayParameter comPositionDerivativeGainsParameter = parameterFactory.createDoubleArray("comPositionDerivativeGains", 0, 0, 750);
   private final DoubleArrayParameter comPositionIntegralGainsParameter = parameterFactory.createDoubleArray("comPositionIntegralGains", 0, 0, 0);
   private final DoubleParameter comPositionMaxIntegralErrorParameter = parameterFactory.createDouble("comPositionMaxIntegralError", 0);
   private final DoubleArrayParameter dcmPositionProportionalGainsParameter = parameterFactory.createDoubleArray("dcmPositionProportionalGains", 1, 1, 0);
   private final DoubleArrayParameter dcmPositionDerivativeGainsParameter = parameterFactory.createDoubleArray("dcmPositionDerivativeGains", 0, 0, 0);
   private final DoubleArrayParameter dcmPositionIntegralGainsParameter = parameterFactory.createDoubleArray("dcmPositionIntegralGains", 0, 0, 0);
   private final DoubleParameter dcmPositionMaxIntegralErrorParameter = parameterFactory.createDouble("dcmPositionMaxIntegralError", 0);
   private final DoubleParameter vrpPositionRateLimitParameter = parameterFactory.createDouble("vrpPositionRateLimit", Double.MAX_VALUE);
   private final DoubleParameter comPositionGravityCompensationParameter = parameterFactory.createDouble("comPositionGravityCompensation", 1);
   private final DoubleParameter dcmPositionStepAdjustmentGainParameter = parameterFactory.createDouble("dcmPositionStepAdjustmentGain", 1.5);
   private final DoubleParameter minimumStepClearanceParameter = parameterFactory.createDouble("minimumStepClearance", 0.075);
   private final DoubleParameter maximumStepStrideParameter = parameterFactory.createDouble("maximumStepStride", 1.0);
   private final DoubleParameter initialTransitionDurationParameter = parameterFactory.createDouble("initialTransitionDuration", 0.5);

   private final YoFrameVector instantaneousStepAdjustment = new YoFrameVector("instantaneousStepAdjustment", worldFrame, registry);
   private final YoFrameVector accumulatedStepAdjustment = new YoFrameVector("accumulatedStepAdjustment", worldFrame, registry);

   private final FramePoint3D dcmPositionEstimate = new FramePoint3D();
   private final FramePoint3D dcmPositionWaypoint = new FramePoint3D();

   private final QuadrupedTimedContactSequence timedContactSequence = new QuadrupedTimedContactSequence(4, 2 * STEP_SEQUENCE_CAPACITY);
   private final QuadrupedPiecewiseConstantCopTrajectory piecewiseConstanceCopTrajectory;
   private final PiecewiseReverseDcmTrajectory dcmTrajectory;
   private final ThreeDoFMinimumJerkTrajectory dcmTransitionTrajectory;

   private final QuadrupedStepCrossoverProjection crossoverProjection;
   private final GroundPlaneEstimator groundPlaneEstimator;

   private final List<QuadrupedTimedStep> stepSequence = new ArrayList<>();
   private final RecyclingArrayList<QuadrupedStep> adjustedActiveSteps;

   private final FramePoint3D tempPoint = new FramePoint3D();

   public QuadrupedBalanceManager(QuadrupedForceControllerToolbox toolbox,
                                  QuadrupedPostureInputProviderInterface postureProvider, YoVariableRegistry parentRegistry)
   {
      this.postureProvider = postureProvider;
      this.dcmTransitionTrajectory = new ThreeDoFMinimumJerkTrajectory();

      robotTimestamp = toolbox.getRuntimeEnvironment().getRobotTimestamp();
      gravity = 9.81;
      mass = toolbox.getRuntimeEnvironment().getFullRobotModel().getTotalMass();

      dcmTrajectory = new PiecewiseReverseDcmTrajectory(STEP_SEQUENCE_CAPACITY, gravity, postureProvider.getComPositionInput().getZ());
      piecewiseConstanceCopTrajectory = new QuadrupedPiecewiseConstantCopTrajectory(timedContactSequence.capacity());
      groundPlaneEstimator = toolbox.getGroundPlaneEstimator();


      supportFrame = toolbox.getReferenceFrames().getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();

      linearInvertedPendulumModel = toolbox.getLinearInvertedPendulumModel();

      dcmPositionEstimator = toolbox.getDcmPositionEstimator();
      dcmPositionController = toolbox.getDcmPositionController();
      comPositionController = toolbox.getComPositionController();
      comPositionControllerSetpoints = new QuadrupedComPositionController.Setpoints();

      crossoverProjection = new QuadrupedStepCrossoverProjection(toolbox.getReferenceFrames().getBodyZUpFrame(), minimumStepClearanceParameter.get(),
                                                                 maximumStepStrideParameter.get());

      adjustedActiveSteps = new RecyclingArrayList<>(10, new GenericTypeBuilder<QuadrupedStep>()
      {
         @Override
         public QuadrupedStep newInstance()
         {
            return new QuadrupedStep();
         }
      });

      parentRegistry.addChild(registry);
   }

   public void clearStepSequence()
   {
      stepSequence.clear();
   }

   public void addStepToSequence(QuadrupedTimedStep step)
   {
      stepSequence.add(step);
   }

   public void addStepsToSequence(List<? extends QuadrupedTimedStep> steps)
   {
      for (int i = 0; i < steps.size(); i++)
         addStepToSequence(steps.get(i));
   }

   public void initialize(QuadrupedTaskSpaceEstimates taskSpaceEstimates)
   {
      // update model
      linearInvertedPendulumModel.setComHeight(postureProvider.getComPositionInput().getZ());

      // update dcm estimate
      dcmPositionEstimator.compute(dcmPositionEstimate, taskSpaceEstimates.getComVelocity());

      dcmPositionController.initializeSetpoint(dcmPositionEstimate);
      dcmPositionController.reset();
      comPositionControllerSetpoints.initialize(taskSpaceEstimates);
      comPositionController.reset();

      // initialize timed contact sequence
      timedContactSequence.initialize();

      accumulatedStepAdjustment.setToZero();
   }

   public void initializeDcmSetpoints(QuadrupedTaskSpaceEstimates taskSpaceEstimates, QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings)
   {
      double currentTime = robotTimestamp.getDoubleValue();
      if (!stepSequence.isEmpty() && stepSequence.get(stepSequence.size() - 1).getTimeInterval().getEndTime() > currentTime)
      {
         // compute dcm trajectory
         computeDcmTrajectory(taskSpaceEstimates, taskSpaceControllerSettings);
         double transitionEndTime = piecewiseConstanceCopTrajectory.getTimeAtStartOfInterval(1);
         double transitionStartTime = Math.max(currentTime, transitionEndTime - initialTransitionDurationParameter.get());
         dcmTrajectory.computeTrajectory(transitionEndTime);
         dcmTrajectory.getPosition(dcmPositionWaypoint);
         dcmTransitionTrajectory.initializeTrajectory(dcmPositionEstimate, dcmPositionWaypoint, transitionStartTime, transitionEndTime);
         computeDcmSetpoints();
      }
   }

   private void computeDcmTrajectory(QuadrupedTaskSpaceEstimates taskSpaceEstimates, QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings)
   {
      if (!stepSequence.isEmpty())
      {
         // compute piecewise constant center of pressure plan
         double currentTime = robotTimestamp.getDoubleValue();
         QuadrantDependentList<FramePoint3D> currentSolePosition = taskSpaceEstimates.getSolePosition();
         QuadrantDependentList<ContactState> currentContactState = taskSpaceControllerSettings.getContactState();
         timedContactSequence.update(stepSequence, currentSolePosition, currentContactState, currentTime);
         piecewiseConstanceCopTrajectory.initializeTrajectory(timedContactSequence);

         // compute dcm trajectory with final boundary constraint
         int numberOfIntervals = piecewiseConstanceCopTrajectory.getNumberOfIntervals();
         dcmPositionWaypoint.setIncludingFrame(piecewiseConstanceCopTrajectory.getCopPositionAtStartOfInterval(numberOfIntervals - 1));
         dcmPositionWaypoint.changeFrame(ReferenceFrame.getWorldFrame());
         dcmPositionWaypoint.add(0, 0, linearInvertedPendulumModel.getComHeight());
         dcmTrajectory.setComHeight(linearInvertedPendulumModel.getComHeight());
         dcmTrajectory.initializeTrajectory(numberOfIntervals, piecewiseConstanceCopTrajectory.getTimeAtStartOfInterval(), piecewiseConstanceCopTrajectory.getCopPositionAtStartOfInterval(),
                                            piecewiseConstanceCopTrajectory.getTimeAtStartOfInterval(numberOfIntervals - 1), dcmPositionWaypoint);
      }
   }

   private void computeDcmSetpoints()
   {
      if (stepSequence.isEmpty())
      {
         // update desired dcm position
         FramePoint3D dcmPositionSetpoint = dcmPositionController.getDCMPositionSetpoint();
         FrameVector3D dcmVelocitySetpoint = dcmPositionController.getDCMVelocitySetpoint();
         dcmPositionSetpoint.changeFrame(supportFrame);
         dcmPositionSetpoint.set(postureProvider.getComVelocityInput());
         dcmPositionSetpoint.scale(1.0 / linearInvertedPendulumModel.getNaturalFrequency());
         dcmPositionSetpoint.add(postureProvider.getComPositionInput());
         dcmVelocitySetpoint.setToZero(supportFrame);
      }
      else if (robotTimestamp.getDoubleValue() <= dcmTransitionTrajectory.getEndTime())
      {
         dcmTransitionTrajectory.computeTrajectory(robotTimestamp.getDoubleValue());
         dcmTransitionTrajectory.getPosition(dcmPositionController.getDCMPositionSetpoint());
         dcmTransitionTrajectory.getVelocity(dcmPositionController.getDCMVelocitySetpoint());
      }
      else
      {
         dcmTrajectory.computeTrajectory(robotTimestamp.getDoubleValue());
         dcmTrajectory.getPosition(dcmPositionController.getDCMPositionSetpoint());
         dcmTrajectory.getVelocity(dcmPositionController.getDCMVelocitySetpoint());
      }
   }


   public void compute(FrameVector3D linearMomentumRateOfChangeToPack, QuadrupedTaskSpaceEstimates taskSpaceEstimates, QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings)
   {
      updateGains();

      // update model
      linearInvertedPendulumModel.setComHeight(postureProvider.getComPositionInput().getZ());

      // update dcm estimate
      dcmPositionEstimator.compute(dcmPositionEstimate, taskSpaceEstimates.getComVelocity());

      computeDcmTrajectory(taskSpaceEstimates, taskSpaceControllerSettings);

      // update desired horizontal com forces
      computeDcmSetpoints();
      dcmPositionController.compute(linearMomentumRateOfChangeToPack, dcmPositionEstimate, dcmPositionController.getDCMPositionSetpoint(), dcmPositionController.getDCMVelocitySetpoint());
      linearMomentumRateOfChangeToPack.changeFrame(supportFrame);

      // update desired com position, velocity, and vertical force
      comPositionControllerSetpoints.getComPosition().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComPosition().set(postureProvider.getComPositionInput());
      comPositionControllerSetpoints.getComVelocity().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComVelocity().set(postureProvider.getComVelocityInput());
      comPositionControllerSetpoints.getComForceFeedforward().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComForceFeedforward().set(linearMomentumRateOfChangeToPack);
      comPositionControllerSetpoints.getComForceFeedforward().setZ(comPositionGravityCompensationParameter.get() * mass * gravity);
      comPositionController.compute(linearMomentumRateOfChangeToPack, comPositionControllerSetpoints, taskSpaceEstimates);
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
   }

   public void completedStep()
   {
      accumulatedStepAdjustment.add(instantaneousStepAdjustment);
      accumulatedStepAdjustment.setZ(0);
   }

   public RecyclingArrayList<QuadrupedStep> computeStepAdjustment(ArrayList<YoQuadrupedTimedStep> activeSteps, QuadrupedTaskSpaceEstimates taskSpaceEstimates)
   {
      adjustedActiveSteps.clear();
      if (robotTimestamp.getDoubleValue() > dcmTransitionTrajectory.getEndTime())
      {
         // compute step adjustment for ongoing steps (proportional to dcm tracking error)
         FramePoint3D dcmPositionSetpoint = dcmPositionController.getDCMPositionSetpoint();
         dcmPositionSetpoint.changeFrame(instantaneousStepAdjustment.getReferenceFrame());
         dcmPositionEstimate.changeFrame(instantaneousStepAdjustment.getReferenceFrame());

         instantaneousStepAdjustment.sub(dcmPositionEstimate, dcmPositionSetpoint);
         instantaneousStepAdjustment.scale(dcmPositionStepAdjustmentGainParameter.get());
         instantaneousStepAdjustment.setZ(0);

         // adjust nominal step goal positions in foot state machine
         for (int i = 0; i < activeSteps.size(); i++)
         {
            YoQuadrupedTimedStep activeStep = activeSteps.get(i);
            QuadrupedStep adjustedStep = adjustedActiveSteps.add();
            adjustedStep.set(activeStep);

            RobotQuadrant robotQuadrant = activeStep.getRobotQuadrant();
            activeStep.getGoalPosition(tempPoint);
            tempPoint.changeFrame(worldFrame);
            tempPoint.add(instantaneousStepAdjustment);
            crossoverProjection.project(tempPoint, taskSpaceEstimates.getSolePosition(), robotQuadrant);
            groundPlaneEstimator.projectZ(tempPoint);
            adjustedStep.setGoalPosition(tempPoint);
         }
      }

      return adjustedActiveSteps;
   }

   public FrameVector3DReadOnly getAccumulatedStepAdjustment()
   {
      return accumulatedStepAdjustment;
   }
}
