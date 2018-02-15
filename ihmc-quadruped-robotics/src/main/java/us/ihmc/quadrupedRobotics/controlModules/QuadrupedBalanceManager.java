package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.*;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedStepCrossoverProjection;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedContactSequence;
import us.ihmc.quadrupedRobotics.planning.YoQuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.trajectory.PiecewiseReverseDcmTrajectory;
import us.ihmc.quadrupedRobotics.planning.trajectory.QuadrupedPiecewiseConstantCopTrajectory;
import us.ihmc.quadrupedRobotics.planning.trajectory.ThreeDoFMinimumJerkTrajectory;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPostureInputProviderInterface;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

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
   private final DoubleParameter comPositionGravityCompensationParameter = parameterFactory.createDouble("comPositionGravityCompensation", 1);
   private final DoubleParameter dcmPositionStepAdjustmentGainParameter = parameterFactory.createDouble("dcmPositionStepAdjustmentGain", 1.5);
   private final DoubleParameter minimumStepClearanceParameter = parameterFactory.createDouble("minimumStepClearance", 0.075);
   private final DoubleParameter maximumStepStrideParameter = parameterFactory.createDouble("maximumStepStride", 1.0);

   private final FramePoint3D dcmPositionEstimate = new FramePoint3D();
   private final FramePoint3D dcmPositionWaypoint = new FramePoint3D();

   private final GroundPlaneEstimator groundPlaneEstimator;
   private final QuadrupedTimedContactSequence timedContactSequence;
   private final QuadrupedPiecewiseConstantCopTrajectory piecewiseConstanceCopTrajectory;
   private final PiecewiseReverseDcmTrajectory dcmTrajectory;
   private final ThreeDoFMinimumJerkTrajectory dcmTransitionTrajectory;
   private final QuadrupedStepCrossoverProjection crossoverProjection;

   private final YoFrameVector instantaneousStepAdjustment;
   private final YoFrameVector accumulatedStepAdjustment;

   private final RecyclingArrayList<YoQuadrupedTimedStep> stepSequence;

   public QuadrupedBalanceManager(QuadrupedForceControllerToolbox toolbox, RecyclingArrayList<YoQuadrupedTimedStep> stepSequence,
                                  QuadrupedPostureInputProviderInterface postureProvider, YoVariableRegistry parentRegistry,
                                  YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.postureProvider = postureProvider;
      this.stepSequence = stepSequence;

      robotTimestamp = toolbox.getRuntimeEnvironment().getRobotTimestamp();
      this.gravity = 9.81;
      this.mass = toolbox.getRuntimeEnvironment().getFullRobotModel().getTotalMass();
      supportFrame = toolbox.getReferenceFrames().getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();

      double gravity = 9.81;
      double mass = toolbox.getRuntimeEnvironment().getFullRobotModel().getTotalMass();
      ReferenceFrame comZUpFrame = toolbox.getReferenceFrames().getCenterOfMassZUpFrame();
      double controlDT = toolbox.getRuntimeEnvironment().getControlDT();

      linearInvertedPendulumModel = new LinearInvertedPendulumModel(comZUpFrame, mass, gravity, 1.0, registry);
      dcmPositionEstimator = new DivergentComponentOfMotionEstimator(comZUpFrame, linearInvertedPendulumModel, registry, yoGraphicsListRegistry);
      dcmPositionController = new DivergentComponentOfMotionController(comZUpFrame, controlDT, linearInvertedPendulumModel, registry, yoGraphicsListRegistry);
      comPositionController = new QuadrupedComPositionController(comZUpFrame, controlDT, registry);
      comPositionControllerSetpoints = new QuadrupedComPositionController.Setpoints();

      timedContactSequence = new QuadrupedTimedContactSequence(4, 2 * STEP_SEQUENCE_CAPACITY);
      piecewiseConstanceCopTrajectory = new QuadrupedPiecewiseConstantCopTrajectory(timedContactSequence.capacity());
      dcmTrajectory = new PiecewiseReverseDcmTrajectory(STEP_SEQUENCE_CAPACITY, gravity, postureProvider.getComPositionInput().getZ());
      dcmTransitionTrajectory = new ThreeDoFMinimumJerkTrajectory();

      groundPlaneEstimator = toolbox.getGroundPlaneEstimator();

      crossoverProjection = new QuadrupedStepCrossoverProjection(toolbox.getReferenceFrames().getBodyZUpFrame(), minimumStepClearanceParameter.get(),
                                                                 maximumStepStrideParameter.get());

      instantaneousStepAdjustment = new YoFrameVector("instantaneousStepAdjustment", worldFrame, registry);
      accumulatedStepAdjustment = new YoFrameVector("accumulatedStepAdjustment", worldFrame, registry);

      parentRegistry.addChild(registry);
   }

   public void compute(FrameVector3D linearMomentumRateOfChangeToPack, QuadrupedTaskSpaceEstimates taskSpaceEstimates)
   {
      // update estimates
      linearInvertedPendulumModel.setComHeight(postureProvider.getComPositionInput().getZ());

      dcmPositionEstimator.compute(dcmPositionEstimate, taskSpaceEstimates.getComVelocity());

      // update setpoints
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

   private void computeDcmSetpoints()
   {
      if (robotTimestamp.getDoubleValue() <= dcmTransitionTrajectory.getEndTime())
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

   public void computeDcmTrajectory(QuadrupedTaskSpaceEstimates taskSpaceEstimates, QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings)
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
      dcmTrajectory.initializeTrajectory(numberOfIntervals, piecewiseConstanceCopTrajectory.getTimeAtStartOfInterval(),
                                         piecewiseConstanceCopTrajectory.getCopPositionAtStartOfInterval(),
                                         piecewiseConstanceCopTrajectory.getTimeAtStartOfInterval(numberOfIntervals - 1), dcmPositionWaypoint);
   }


   public void computeStepAdjustment()
   {
      // compute step adjustment for ongoing steps (proportional to dcm tracking error)
      FramePoint3D dcmPositionSetpoint = dcmPositionController.getDCMPositionSetpoint();
      dcmPositionSetpoint.changeFrame(instantaneousStepAdjustment.getReferenceFrame());
      dcmPositionEstimate.changeFrame(instantaneousStepAdjustment.getReferenceFrame());
      instantaneousStepAdjustment.set(dcmPositionEstimate);
      instantaneousStepAdjustment.sub(dcmPositionSetpoint);
      instantaneousStepAdjustment.scale(dcmPositionStepAdjustmentGainParameter.get());
      instantaneousStepAdjustment.setZ(0);
   }

   public void adjustStepPosition(FramePoint3D stepGoalPositionToPack, QuadrupedTaskSpaceEstimates taskSpaceEstimates)
   {
      // adjust nominal step goal positions in foot state machine
      for (int i = 0; i < stepSequence.size(); i++)
      {
         RobotQuadrant robotQuadrant = stepSequence.get(i).getRobotQuadrant();
         double currentTime = robotTimestamp.getDoubleValue();
         double startTime = stepSequence.get(i).getTimeInterval().getStartTime();
         double endTime = stepSequence.get(i).getTimeInterval().getEndTime();
         if (startTime < currentTime && currentTime < endTime)
         {
            stepSequence.get(i).getGoalPosition(stepGoalPositionToPack);
            stepGoalPositionToPack.changeFrame(worldFrame);
            stepGoalPositionToPack.add(instantaneousStepAdjustment);
            crossoverProjection.project(stepGoalPositionToPack, taskSpaceEstimates.getSolePosition(), robotQuadrant);
            groundPlaneEstimator.projectZ(stepGoalPositionToPack);
         }
      }
   }
}
