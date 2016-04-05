package us.ihmc.aware.controller.force;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.aware.params.DoubleArrayParameter;
import us.ihmc.aware.params.DoubleParameter;
import us.ihmc.aware.params.ParameterFactory;
import us.ihmc.aware.controller.common.DivergentComponentOfMotionController;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceCommands;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceController;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceControllerSettings;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceEstimates;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceEstimator;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceSetpoints;
import us.ihmc.aware.parameters.QuadrupedRuntimeEnvironment;
import us.ihmc.aware.planning.PiecewiseCopPlanner;
import us.ihmc.aware.planning.PiecewiseReverseDcmTrajectory;
import us.ihmc.aware.planning.ThreeDoFSwingFootTrajectory;
import us.ihmc.aware.planning.XGaitStepPlanner;
import us.ihmc.aware.state.StateMachine;
import us.ihmc.aware.state.StateMachineBuilder;
import us.ihmc.aware.state.StateMachineState;
import us.ihmc.aware.util.ContactState;
import us.ihmc.aware.util.PreallocatedQueue;
import us.ihmc.aware.util.QuadrupedTimedStep;
import us.ihmc.aware.util.TimeInterval;
import us.ihmc.quadrupedRobotics.dataProviders.QuadrupedControllerInputProviderInterface;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class QuadrupedVirtualModelBasedStepController implements QuadrupedForceController
{
   private final SDFFullRobotModel fullRobotModel;
   private final DoubleYoVariable robotTimestamp;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final double controlDT;
   private final double gravity;
   private final double mass;
   private final QuadrupedControllerInputProviderInterface inputProvider;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   // parameters
   private final ParameterFactory parameterFactory = new ParameterFactory(QuadrupedVirtualModelBasedStepController.class.getName());
   private final DoubleParameter jointDampingParameter = parameterFactory.createDouble("jointDamping", 1);
   private final DoubleArrayParameter bodyOrientationProportionalGainsParameter = parameterFactory.createDoubleArray("bodyOrientationProportionalGains", 5000, 5000, 5000);
   private final DoubleArrayParameter bodyOrientationDerivativeGainsParameter = parameterFactory.createDoubleArray("bodyOrientationDerivativeGains", 750, 750, 750);
   private final DoubleArrayParameter bodyOrientationIntegralGainsParameter = parameterFactory.createDoubleArray("bodyOrientationIntegralGains", 0, 0, 0);
   private final DoubleParameter bodyOrientationMaxIntegralErrorParameter = parameterFactory.createDouble("bodyOrientationMaxIntegralError", 0);
   private final DoubleArrayParameter comPositionProportionalGainsParameter = parameterFactory.createDoubleArray("comPositionProportionalGains", 0, 0, 5000);
   private final DoubleArrayParameter comPositionDerivativeGainsParameter = parameterFactory.createDoubleArray("comPositionDerivativeGains", 0, 0, 750);
   private final DoubleArrayParameter comPositionIntegralGainsParameter = parameterFactory.createDoubleArray("comPositionIntegralGains", 0, 0, 0);
   private final DoubleParameter comPositionMaxIntegralErrorParameter = parameterFactory.createDouble("comPositionMaxIntegralError", 0);
   private final DoubleArrayParameter dcmPositionProportionalGainsParameter = parameterFactory.createDoubleArray("dcmPositionProportionalGains", 1, 1, 0);
   private final DoubleArrayParameter dcmPositionDerivativeGainsParameter = parameterFactory.createDoubleArray("dcmPositionDerivativeGains", 0, 0, 0);
   private final DoubleArrayParameter dcmPositionIntegralGainsParameter = parameterFactory.createDoubleArray("dcmPositionIntegralGains", 0, 0, 0);
   private final DoubleParameter dcmPositionMaxIntegralErrorParameter = parameterFactory.createDouble("dcmPositionMaxIntegralError", 0);
   private final DoubleArrayParameter swingPositionProportionalGainsParameter = parameterFactory.createDoubleArray("swingPositionProportionalGains", 50000, 50000, 100000);
   private final DoubleArrayParameter swingPositionDerivativeGainsParameter = parameterFactory.createDoubleArray("swingPositionDerivativeGains", 500, 500, 500);
   private final DoubleArrayParameter swingPositionIntegralGainsParameter = parameterFactory.createDoubleArray("swingPositionIntegralGains", 0, 0, 0);
   private final DoubleParameter swingPositionMaxIntegralErrorParameter = parameterFactory.createDouble("swingPositionMaxIntegralError", 0);
   private final DoubleParameter swingTrajectoryGroundClearanceParameter = parameterFactory.createDouble("swingTrajectoryGroundClearance", 0.1);
   private final DoubleParameter noContactPressureLimitParameter = parameterFactory.createDouble("noContactPressureLimit", 75);

   // frames
   private final ReferenceFrame supportFrame;
   private final ReferenceFrame worldFrame;

   // dcm controller
   private final FramePoint dcmPositionEstimate;
   private final FramePoint dcmPositionSetpoint;
   private final FrameVector dcmVelocitySetpoint;
   private final DivergentComponentOfMotionController dcmPositionController;

   // task space controller
   private final QuadrupedTaskSpaceCommands taskSpaceCommands;
   private final QuadrupedTaskSpaceSetpoints taskSpaceSetpoints;
   private final QuadrupedTaskSpaceEstimates taskSpaceEstimates;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final QuadrupedTaskSpaceController taskSpaceController;
   private final QuadrupedTaskSpaceControllerSettings taskSpaceControllerSettings;

   // planning
   private static int STEP_QUEUE_CAPACITY = 60;
   private final FramePoint dcmPositionWaypoint;
   private final PiecewiseReverseDcmTrajectory dcmTrajectory;
   private final PiecewiseCopPlanner copPlanner;
   private final XGaitStepPlanner footstepPlanner;
   private final PreallocatedQueue<QuadrupedTimedStep> stepQueue;
   private final QuadrantDependentList<QuadrupedTimedStep> stepCache;
   private final QuadrantDependentList<ThreeDoFSwingFootTrajectory> swingFootTrajectory;

   // state machine
   public enum FootState
   {
      SUPPORT, TRANSFER, SWING
   }
   public enum FootEvent
   {
      TRANSFER, LIFT_OFF, TOUCH_DOWN
   }
   private final QuadrantDependentList<StateMachine<FootState, FootEvent>> footStateMachine;

   public QuadrupedVirtualModelBasedStepController(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedControllerInputProviderInterface inputProvider,
         QuadrupedForceControllerContext controllerContext)
   {
      this.fullRobotModel = runtimeEnvironment.getFullRobotModel();
      this.robotTimestamp = runtimeEnvironment.getRobotTimestamp();
      this.yoGraphicsListRegistry = runtimeEnvironment.getGraphicsListRegistry();
      this.controlDT = runtimeEnvironment.getControlDT();
      this.gravity = 9.81;
      this.mass = fullRobotModel.getTotalMass();
      this.inputProvider = inputProvider;

      // utilities
      QuadrupedReferenceFrames referenceFrames = controllerContext.getReferenceFrames();
      supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      worldFrame = ReferenceFrame.getWorldFrame();

      // dcm controller
      dcmPositionEstimate = new FramePoint();
      dcmPositionSetpoint = new FramePoint();
      dcmVelocitySetpoint = new FrameVector();
      dcmPositionController = controllerContext.getDcmPositionController();

      // task space controller
      taskSpaceCommands = new QuadrupedTaskSpaceCommands();
      taskSpaceSetpoints = new QuadrupedTaskSpaceSetpoints();
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimates();
      taskSpaceControllerSettings = new QuadrupedTaskSpaceControllerSettings();
      taskSpaceEstimator = controllerContext.getTaskSpaceEstimator();
      taskSpaceController = controllerContext.getTaskSpaceController();

      // planning
      copPlanner = new PiecewiseCopPlanner(2 * STEP_QUEUE_CAPACITY);
      dcmPositionWaypoint = new FramePoint(worldFrame);
      dcmTrajectory = new PiecewiseReverseDcmTrajectory(2 * STEP_QUEUE_CAPACITY, gravity, inputProvider.getComPositionInput().getZ(), registry);
      footstepPlanner = new XGaitStepPlanner(registry, yoGraphicsListRegistry, referenceFrames);
      swingFootTrajectory = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         swingFootTrajectory.set(robotQuadrant, new ThreeDoFSwingFootTrajectory());
      }
      stepQueue = new PreallocatedQueue<>(QuadrupedTimedStep.class, STEP_QUEUE_CAPACITY);
      stepCache = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         stepCache.set(robotQuadrant, new QuadrupedTimedStep(robotQuadrant));
      }

      // state machines
      footStateMachine = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();
         StateMachineBuilder<FootState, FootEvent> stateMachineBuilder = new StateMachineBuilder<>(FootState.class, prefix + "FootState", registry);
         stateMachineBuilder.addState(FootState.SUPPORT, new SupportState(robotQuadrant));
         stateMachineBuilder.addState(FootState.TRANSFER, new TransferState(robotQuadrant));
         stateMachineBuilder.addState(FootState.SWING, new SwingState(robotQuadrant));
         stateMachineBuilder.addTransition(FootEvent.TRANSFER, FootState.SUPPORT, FootState.TRANSFER);
         stateMachineBuilder.addTransition(FootEvent.LIFT_OFF, FootState.TRANSFER, FootState.SWING);
         stateMachineBuilder.addTransition(FootEvent.TOUCH_DOWN, FootState.SWING, FootState.SUPPORT);
         footStateMachine.set(robotQuadrant, stateMachineBuilder.build(FootState.SUPPORT));
      }

      runtimeEnvironment.getParentRegistry().addChild(registry);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public boolean addStep(QuadrupedTimedStep quadrupedTimedStep)
   {
      if ((quadrupedTimedStep.getTimeInterval().getStartTime() > robotTimestamp.getDoubleValue()) && stepQueue.enqueue())
      {
         stepQueue.getTail().set(quadrupedTimedStep);
         return true;
      }
      return false;
   }

   public void removeSteps()
   {
      for (int i = 0; i < stepQueue.size(); i++)
      {
         // keep ongoing steps in the queue
         QuadrupedTimedStep step = stepQueue.getHead();
         if (step.getTimeInterval().getStartTime() < robotTimestamp.getDoubleValue())
         {
            stepQueue.enqueue();
            stepQueue.getTail().set(step);
         }
         // remove future steps from the queue
         stepQueue.dequeue();
      }
   }

   public int getStepQueueSize()
   {
      return stepQueue.size();
   }

   private void handleStepEvents()
   {
      double currentTime = robotTimestamp.getDoubleValue();

      while ((stepQueue.size() > 0) && (stepQueue.getHead().getTimeInterval().getEndTime() < currentTime))
      {
         stepQueue.dequeue();
      }

      for (int i = 0; i < stepQueue.size(); i++)
      {
         QuadrupedTimedStep step = stepQueue.get(i);
         if (step.getTimeInterval().getStartTime() <= currentTime)
         {
            footStateMachine.get(step.getRobotQuadrant()).trigger(FootEvent.TRANSFER);
            stepCache.get(step.getRobotQuadrant()).set(step);
         }
      }
   }

   private void computeDcmPositionAndVelocitySetpoints()
   {
      if (stepQueue.size() > 0)
      {
         double startTime = stepQueue.getHead().getTimeInterval().getStartTime();
         double currentTime = robotTimestamp.getDoubleValue();
         if (currentTime >= startTime - 0.5)
         {
            int nTransitions = copPlanner.compute(taskSpaceEstimates.getSolePosition(), taskSpaceControllerSettings.getContactState(), stepQueue);
            dcmPositionWaypoint.setIncludingFrame(copPlanner.getCopAtTransition(nTransitions - 1));
            dcmPositionWaypoint.changeFrame(worldFrame);
            dcmPositionWaypoint.add(0, 0, inputProvider.getComPositionInput().getZ());
            dcmTrajectory.setComHeight(dcmPositionController.getComHeight());
            dcmTrajectory.initializeTrajectory(nTransitions, copPlanner.getTimeAtTransitions(), copPlanner.getCopAtTransitions(),
                  copPlanner.getTimeAtTransition(nTransitions - 1), dcmPositionWaypoint);
            dcmTrajectory.computeTrajectory(currentTime);
            if (currentTime >= startTime)
            {
               // compute dcm trajectory while stepping
               dcmTrajectory.getPosition(dcmPositionSetpoint);
               dcmTrajectory.getVelocity(dcmVelocitySetpoint);
            }
            else
            {
               // compute dcm trajectory to transition from standing to stepping
               double deltaTime = Math.max(startTime - currentTime, 0.001);
               dcmPositionSetpoint.changeFrame(worldFrame);
               dcmVelocitySetpoint.changeFrame(worldFrame);
               dcmTrajectory.getPosition(dcmPositionWaypoint);
               dcmPositionWaypoint.changeFrame(worldFrame);
               dcmPositionWaypoint.sub(dcmPositionSetpoint);
               dcmPositionWaypoint.scale(1 / deltaTime);
               dcmVelocitySetpoint.set(dcmPositionWaypoint);
               dcmPositionWaypoint.scale(controlDT);
               dcmPositionSetpoint.add(dcmPositionWaypoint);
            }
         }
      }
      else
      {
         // compute dcm trajectory while standing
         dcmPositionSetpoint.setToZero(supportFrame);
         dcmPositionSetpoint.add(0, 0, dcmPositionController.getComHeight());
         dcmVelocitySetpoint.setToZero(supportFrame);
      }
   }

   private void updateEstimates()
   {
      // update task space estimates
      taskSpaceEstimator.compute(taskSpaceEstimates);

      // update dcm estimate
      taskSpaceEstimates.getComPosition().changeFrame(worldFrame);
      taskSpaceEstimates.getComVelocity().changeFrame(worldFrame);
      dcmPositionEstimate.changeFrame(worldFrame);
      dcmPositionEstimate.set(taskSpaceEstimates.getComVelocity());
      dcmPositionEstimate.scale(1.0 / dcmPositionController.getNaturalFrequency());
      dcmPositionEstimate.add(taskSpaceEstimates.getComPosition());
   }

   private void updateSetpoints()
   {
      // compute virtual foot forces
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         footStateMachine.get(robotQuadrant).process();
      }

      // update desired horizontal com forces
      computeDcmPositionAndVelocitySetpoints();
      dcmPositionController.compute(taskSpaceSetpoints.getComForceFeedforward(), dcmPositionSetpoint, dcmVelocitySetpoint, dcmPositionEstimate);

      // update desired com position, velocity, and vertical force
      taskSpaceSetpoints.getComPosition().changeFrame(supportFrame);
      taskSpaceSetpoints.getComPosition().set(inputProvider.getComPositionInput());
      taskSpaceSetpoints.getComVelocity().setToZero();
      taskSpaceSetpoints.getComForceFeedforward().changeFrame(worldFrame);
      taskSpaceSetpoints.getComForceFeedforward().setZ(mass * gravity);

      // update desired body orientation, angular velocity, and torque
      taskSpaceSetpoints.getBodyOrientation().changeFrame(supportFrame);
      taskSpaceSetpoints.getBodyOrientation().set(inputProvider.getBodyOrientationInput());
      taskSpaceSetpoints.getBodyAngularVelocity().setToZero();
      taskSpaceSetpoints.getComTorqueFeedforward().setToZero();

      // update joint setpoints
      taskSpaceController.compute(taskSpaceControllerSettings, taskSpaceSetpoints, taskSpaceEstimates, taskSpaceCommands);
      taskSpaceSetpoints.getBodyOrientation().changeFrame(worldFrame);
   }

   @Override public QuadrupedForceControllerEvent process()
   {
      dcmPositionController.setComHeight(inputProvider.getComPositionInput().getZ());
      handleStepEvents();
      updateEstimates();
      updateSetpoints();
      return null;
   }

   @Override public void onEntry()
   {
      // initialize dcm controller
      dcmPositionController.setGains(
            dcmPositionProportionalGainsParameter.get(),
            dcmPositionDerivativeGainsParameter.get(),
            dcmPositionIntegralGainsParameter.get(),
            dcmPositionMaxIntegralErrorParameter.get()
      );
      dcmPositionController.reset();

      // initialize task space controller
      taskSpaceEstimator.compute(taskSpaceEstimates);
      taskSpaceSetpoints.initialize(taskSpaceEstimates);
      taskSpaceControllerSettings.initialize();
      taskSpaceControllerSettings.setJointDamping(jointDampingParameter.get());
      taskSpaceControllerSettings.setComForceCommandWeights(1.0, 1.0, 1.0);
      taskSpaceControllerSettings.setComTorqueCommandWeights(1.0, 1.0, 1.0);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         taskSpaceControllerSettings.setSoleForceCommandWeights(robotQuadrant, 0.0, 0.0, 0.0);
         taskSpaceControllerSettings.setContactState(robotQuadrant, ContactState.IN_CONTACT);
         taskSpaceControllerSettings.setSolePositionFeedbackGainsToZero(robotQuadrant);
      }
      taskSpaceControllerSettings.setBodyOrientationFeedbackGains(
            bodyOrientationProportionalGainsParameter.get(),
            bodyOrientationDerivativeGainsParameter.get(),
            bodyOrientationIntegralGainsParameter.get(),
            bodyOrientationMaxIntegralErrorParameter.get()
      );
      taskSpaceControllerSettings.setComPositionFeedbackGains(
            comPositionProportionalGainsParameter.get(),
            comPositionDerivativeGainsParameter.get(),
            comPositionIntegralGainsParameter.get(),
            comPositionMaxIntegralErrorParameter.get()
      );
      taskSpaceController.reset();

      // initialize dcm setpoints
      updateEstimates();
      dcmPositionSetpoint.setIncludingFrame(dcmPositionEstimate);
      dcmVelocitySetpoint.setToZero();

      // initialize step queue
      footstepPlanner.plan(stepQueue, robotTimestamp.getDoubleValue() + 2.0, true);

      // initialize state machine
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         footStateMachine.get(robotQuadrant).reset();
      }
   }

   @Override public void onExit()
   {
      // remove remaining steps from the queue
      removeSteps();
   }

   private class SupportState implements StateMachineState<FootEvent>
   {
      private final RobotQuadrant robotQuadrant;

      public SupportState(RobotQuadrant robotQuadrant)
      {
         this.robotQuadrant = robotQuadrant;
      }

      @Override public void onEntry()
      {
         // initialize contact state
         taskSpaceControllerSettings.setContactState(robotQuadrant, ContactState.IN_CONTACT);
         taskSpaceControllerSettings.setPressureUpperLimit(robotQuadrant, Double.MAX_VALUE);

         // disable sole position feedback
         taskSpaceControllerSettings.setSolePositionFeedbackGainsToZero(robotQuadrant);
      }

      @Override public FootEvent process()
      {
         return null;
      }

      @Override public void onExit()
      {
      }
   }

   private class TransferState implements StateMachineState<FootEvent>
   {
      private final RobotQuadrant robotQuadrant;

      public TransferState(RobotQuadrant robotQuadrant)
      {
         this.robotQuadrant = robotQuadrant;
      }

      @Override public void onEntry()
      {
      }

      @Override public FootEvent process()
      {
         double currentTime = robotTimestamp.getDoubleValue();
         double liftOffTime = stepCache.get(robotQuadrant).getTimeInterval().getStartTime();

         // trigger lift off event
         if (currentTime > liftOffTime)
            return FootEvent.LIFT_OFF;
         else
            return null;
      }

      @Override public void onExit()
      {
      }
   }

   private class SwingState implements StateMachineState<FootEvent>
   {
      private final RobotQuadrant robotQuadrant;

      public SwingState(RobotQuadrant robotQuadrant)
      {
         this.robotQuadrant = robotQuadrant;
      }

      @Override public void onEntry()
      {
         // initialize swing foot controller
         TimeInterval timeInterval = stepCache.get(robotQuadrant).getTimeInterval();
         FramePoint goalPosition = stepCache.get(robotQuadrant).getGoalPosition();
         FramePoint solePosition = taskSpaceEstimates.getSolePosition(robotQuadrant);
         solePosition.changeFrame(goalPosition.getReferenceFrame());
         swingFootTrajectory.get(robotQuadrant).initializeTrajectory(
               solePosition, goalPosition, swingTrajectoryGroundClearanceParameter.get(), timeInterval.getDuration());

         // initialize sole position feedback gains
         taskSpaceControllerSettings.setSolePositionFeedbackGains(robotQuadrant,
               swingPositionProportionalGainsParameter.get(),
               swingPositionDerivativeGainsParameter.get(),
               swingPositionIntegralGainsParameter.get(),
               swingPositionMaxIntegralErrorParameter.get()
         );

         // initialize contact state
         taskSpaceControllerSettings.setContactState(robotQuadrant, ContactState.NO_CONTACT);
         taskSpaceControllerSettings.setPressureUpperLimit(robotQuadrant, noContactPressureLimitParameter.get());
      }

      @Override public FootEvent process()
      {
         double currentTime = robotTimestamp.getDoubleValue();
         double liftOffTime = stepCache.get(robotQuadrant).getTimeInterval().getStartTime();
         double touchDownTime = stepCache.get(robotQuadrant).getTimeInterval().getEndTime();

         // compute swing trajectory
         swingFootTrajectory.get(robotQuadrant).computeTrajectory(currentTime - liftOffTime);
         swingFootTrajectory.get(robotQuadrant).getPosition(taskSpaceSetpoints.getSolePosition(robotQuadrant));

         // shift the swing foot trajectory in the direction of the dcm tracking error
         double alpha = 1.5 * Math.sqrt((currentTime - liftOffTime) / (touchDownTime - liftOffTime));
         dcmPositionEstimate.changeFrame(worldFrame);
         dcmPositionSetpoint.changeFrame(worldFrame);
         taskSpaceSetpoints.getSolePosition(robotQuadrant).changeFrame(worldFrame);
         taskSpaceSetpoints.getSolePosition(robotQuadrant).add(alpha * dcmPositionEstimate.getX(), alpha * dcmPositionEstimate.getY(), 0.0);
         taskSpaceSetpoints.getSolePosition(robotQuadrant).sub(alpha * dcmPositionSetpoint.getX(), alpha * dcmPositionSetpoint.getY(), 0.0);

         // trigger touch down event
         if (currentTime > touchDownTime)
            return FootEvent.TOUCH_DOWN;
         else
            return null;
      }

      @Override public void onExit()
      {
      }
   }
}
