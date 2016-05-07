package us.ihmc.quadrupedRobotics.controller.force.states;

import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.*;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.params.DoubleArrayParameter;
import us.ihmc.quadrupedRobotics.params.DoubleParameter;
import us.ihmc.quadrupedRobotics.params.ParameterFactory;
import us.ihmc.quadrupedRobotics.planning.*;
import us.ihmc.quadrupedRobotics.planning.trajectory.PiecewiseForwardDcmTrajectory;
import us.ihmc.quadrupedRobotics.planning.trajectory.PiecewiseReverseDcmTrajectory;
import us.ihmc.quadrupedRobotics.providers.QuadrupedControllerInputProviderInterface;
import us.ihmc.quadrupedRobotics.state.FiniteStateMachine;
import us.ihmc.quadrupedRobotics.state.FiniteStateMachineBuilder;
import us.ihmc.quadrupedRobotics.state.FiniteStateMachineState;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.*;

import javax.vecmath.Point3d;
import java.util.ArrayList;

public class QuadrupedDcmBasedXGaitController implements QuadrupedController
{
   private final QuadrupedControllerInputProviderInterface inputProvider;
   private final DoubleYoVariable robotTimestamp;
   private final double controlDT;
   private final double gravity;
   private final double mass;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   // parameters
   private final ParameterFactory parameterFactory = new ParameterFactory(getClass());
   private final DoubleParameter jointDampingParameter = parameterFactory.createDouble("jointDamping", 2);
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
   private final DoubleParameter initialTransitionDurationParameter = parameterFactory.createDouble("initialTransitionDuration", 1.00);

   // frames
   private final ReferenceFrame supportFrame;
   private final ReferenceFrame worldFrame;

   // feedback controllers
   private final FramePoint dcmPositionEstimate;
   private final DivergentComponentOfMotionController.Setpoints dcmPositionControllerSetpoints;
   private final DivergentComponentOfMotionController dcmPositionController;
   private final QuadrupedComPositionController.Setpoints comPositionControllerSetpoints;
   private final QuadrupedComPositionController comPositionController;
   private final QuadrupedBodyOrientationController.Setpoints bodyOrientationControllerSetpoints;
   private final QuadrupedBodyOrientationController bodyOrientationController;
   private final QuadrupedTimedStepController.Setpoints timedStepControllerSetpoints;
   private final QuadrupedTimedStepController timedStepController;

   // task space controller
   private final QuadrupedTaskSpaceEstimator.Estimates taskSpaceEstimates;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final QuadrupedTaskSpaceController.Commands taskSpaceControllerCommands;
   private final QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings;
   private final QuadrupedTaskSpaceController taskSpaceController;

   // planning
   private double bodyYawSetpoint;
   private final GroundPlaneEstimator groundPlaneEstimator;
   private final QuadrantDependentList<FramePoint> groundPlanePositions;
   private final QuadrupedTimedStepCopPlanner copPlanner;
   private final QuadrupedXGaitSettings xGaitSettings;
   private final QuadrupedXGaitPlanner xGaitStepPlanner;
   private final ArrayList<QuadrupedTimedStep> xGaitPreviewSteps;

   // state machine
   public enum XGaitState
   {
      INITIAL_TRANSITION, FORWARD_XGAIT, REVERSE_XGAIT
   }
   public enum XGaitEvent
   {
      TIMEOUT, FORWARD, REVERSE
   }
   private final FiniteStateMachine<XGaitState, XGaitEvent> xGaitStateMachine;

   public QuadrupedDcmBasedXGaitController(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedForceControllerToolbox controllerToolbox,
         QuadrupedControllerInputProviderInterface inputProvider)
   {
      this.inputProvider = inputProvider;
      this.robotTimestamp = runtimeEnvironment.getRobotTimestamp();
      this.controlDT = runtimeEnvironment.getControlDT();
      this.gravity = 9.81;
      this.mass = runtimeEnvironment.getFullRobotModel().getTotalMass();

      // frames
      QuadrupedReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      worldFrame = ReferenceFrame.getWorldFrame();

      // feedback controllers
      dcmPositionEstimate = new FramePoint();
      dcmPositionControllerSetpoints = new DivergentComponentOfMotionController.Setpoints();
      dcmPositionController = controllerToolbox.getDcmPositionController();
      comPositionControllerSetpoints = new QuadrupedComPositionController.Setpoints();
      comPositionController = controllerToolbox.getComPositionController();
      bodyOrientationControllerSetpoints = new QuadrupedBodyOrientationController.Setpoints();
      bodyOrientationController = controllerToolbox.getBodyOrientationController();
      timedStepControllerSetpoints = new QuadrupedTimedStepController.Setpoints();
      timedStepController = controllerToolbox.getTimedStepController();

      // task space controllers
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimator.Estimates();
      taskSpaceEstimator = controllerToolbox.getTaskSpaceEstimator();
      taskSpaceControllerCommands = new QuadrupedTaskSpaceController.Commands();
      taskSpaceControllerSettings = new QuadrupedTaskSpaceController.Settings();
      taskSpaceController = controllerToolbox.getTaskSpaceController();

      // planning
      groundPlaneEstimator = new GroundPlaneEstimator();
      groundPlanePositions = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         groundPlanePositions.set(robotQuadrant, new FramePoint());
      }
      copPlanner = new QuadrupedTimedStepCopPlanner(32);
      xGaitStepPlanner = new QuadrupedXGaitPlanner();
      xGaitPreviewSteps = new ArrayList<>(32);
      for (int i = 0; i < 32; i++)
      {
         xGaitPreviewSteps.add(new QuadrupedTimedStep());
      }
      xGaitSettings = new QuadrupedXGaitSettings();

      // state machine
      FiniteStateMachineBuilder<XGaitState, XGaitEvent> ambleStateMachineBuilder = new FiniteStateMachineBuilder<>(XGaitState.class, XGaitEvent.class, "XGaitState", registry);
      ambleStateMachineBuilder.addState(XGaitState.INITIAL_TRANSITION, new InitialTransitionState());
      ambleStateMachineBuilder.addState(XGaitState.FORWARD_XGAIT, new ForwardXGaitState());
      ambleStateMachineBuilder.addState(XGaitState.REVERSE_XGAIT, new ReverseXGaitState());
      ambleStateMachineBuilder.addTransition(XGaitEvent.TIMEOUT, XGaitState.INITIAL_TRANSITION, XGaitState.FORWARD_XGAIT);
      ambleStateMachineBuilder.addTransition(XGaitEvent.REVERSE, XGaitState.FORWARD_XGAIT, XGaitState.REVERSE_XGAIT);
      ambleStateMachineBuilder.addTransition(XGaitEvent.FORWARD, XGaitState.REVERSE_XGAIT, XGaitState.FORWARD_XGAIT);
      xGaitStateMachine = ambleStateMachineBuilder.build(XGaitState.INITIAL_TRANSITION);
      runtimeEnvironment.getParentRegistry().addChild(registry);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
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

      // update ground plane estimate
      groundPlaneEstimator.compute(groundPlanePositions);
   }

   private void updateSetpoints()
   {
      // update state machines
      xGaitStateMachine.process();

      // update desired horizontal com forces
      dcmPositionController.compute(taskSpaceControllerCommands.getComForce(), dcmPositionControllerSetpoints, dcmPositionEstimate);
      taskSpaceControllerCommands.getComForce().changeFrame(supportFrame);

      // update desired com position, velocity, and vertical force
      comPositionControllerSetpoints.getComPosition().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComPosition().set(inputProvider.getComPositionInput());
      comPositionControllerSetpoints.getComVelocity().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComVelocity().set(inputProvider.getComVelocityInput());
      comPositionControllerSetpoints.getComForceFeedforward().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComForceFeedforward().set(taskSpaceControllerCommands.getComForce());
      comPositionControllerSetpoints.getComForceFeedforward().setZ(mass * gravity);
      comPositionController.compute(taskSpaceControllerCommands.getComForce(), comPositionControllerSetpoints, taskSpaceEstimates);

      // update desired body orientation, angular velocity, and torque
      if (xGaitStateMachine.getState() != XGaitState.INITIAL_TRANSITION)
      {
         bodyYawSetpoint += inputProvider.getPlanarVelocityInput().getZ() * controlDT;
      }
      bodyOrientationControllerSetpoints.getBodyOrientation().changeFrame(worldFrame);
      bodyOrientationControllerSetpoints.getBodyOrientation().setYawPitchRoll(bodyYawSetpoint,
            RotationTools.computePitch(inputProvider.getBodyOrientationInput()) + groundPlaneEstimator.getPitch(bodyYawSetpoint),
            RotationTools.computeRoll(inputProvider.getBodyOrientationInput()));
      bodyOrientationControllerSetpoints.getBodyAngularVelocity().setToZero();
      bodyOrientationControllerSetpoints.getComTorqueFeedforward().setToZero();
      bodyOrientationController.compute(taskSpaceControllerCommands.getComTorque(), bodyOrientationControllerSetpoints, taskSpaceEstimates);

      // update desired contact state and sole forces
      FramePoint dcmPositionSetpoint = dcmPositionControllerSetpoints.getDcmPosition();
      dcmPositionSetpoint.changeFrame(worldFrame);
      dcmPositionEstimate.changeFrame(worldFrame);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         timedStepControllerSetpoints.getStepAdjustment(robotQuadrant).changeFrame(worldFrame);
         timedStepControllerSetpoints.getStepAdjustment(robotQuadrant).set(dcmPositionEstimate.getX(), dcmPositionEstimate.getY(), 0.0);
         timedStepControllerSetpoints.getStepAdjustment(robotQuadrant).sub(dcmPositionSetpoint.getX(), dcmPositionSetpoint.getY(), 0.0);
      }
      timedStepController.compute(taskSpaceControllerSettings.getContactState(), taskSpaceControllerCommands.getSoleForce(), timedStepControllerSetpoints, taskSpaceEstimates);

      // update joint setpoints
      taskSpaceController.compute(taskSpaceControllerSettings, taskSpaceControllerCommands);
   }

   @Override public ControllerEvent process()
   {
      updateEstimates();
      updateSetpoints();
      return null;
   }

   @Override public void onEntry()
   {
      // initialize estimates
      dcmPositionController.setComHeight(inputProvider.getComPositionInput().getZ());
      updateEstimates();

      // initialize feedback controllers
      dcmPositionControllerSetpoints.initialize(dcmPositionEstimate);
      dcmPositionController.reset();
      dcmPositionController.getGains().setProportionalGains(dcmPositionProportionalGainsParameter.get());
      dcmPositionController.getGains().setIntegralGains(dcmPositionIntegralGainsParameter.get(), dcmPositionMaxIntegralErrorParameter.get());
      dcmPositionController.getGains().setDerivativeGains(dcmPositionDerivativeGainsParameter.get());
      comPositionControllerSetpoints.initialize(taskSpaceEstimates);
      comPositionController.reset();
      comPositionController.getGains().setProportionalGains(comPositionProportionalGainsParameter.get());
      comPositionController.getGains().setIntegralGains(comPositionIntegralGainsParameter.get(), comPositionMaxIntegralErrorParameter.get());
      comPositionController.getGains().setDerivativeGains(comPositionDerivativeGainsParameter.get());
      bodyOrientationControllerSetpoints.initialize(taskSpaceEstimates);
      bodyOrientationController.reset();
      bodyOrientationController.getGains().setProportionalGains(bodyOrientationProportionalGainsParameter.get());
      bodyOrientationController.getGains().setIntegralGains(bodyOrientationIntegralGainsParameter.get(), bodyOrientationMaxIntegralErrorParameter.get());
      bodyOrientationController.getGains().setDerivativeGains(bodyOrientationDerivativeGainsParameter.get());
      timedStepControllerSetpoints.initialize(taskSpaceEstimates);
      timedStepController.reset();

      // initialize task space controller
      taskSpaceControllerSettings.initialize();
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointDamping(jointDampingParameter.get());
      taskSpaceControllerSettings.getContactForceOptimizationSettings().setComForceCommandWeights(1.0, 1.0, 1.0);
      taskSpaceControllerSettings.getContactForceOptimizationSettings().setComTorqueCommandWeights(1.0, 1.0, 1.0);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         taskSpaceControllerSettings.getContactForceOptimizationSettings().setContactForceCommandWeights(robotQuadrant, 0.0, 0.0, 0.0);
         taskSpaceControllerSettings.setContactState(robotQuadrant, ContactState.IN_CONTACT);
      }
      taskSpaceController.reset();

      // initialize body yaw trajectory
      taskSpaceEstimates.getBodyOrientation().changeFrame(worldFrame);
      bodyYawSetpoint = taskSpaceEstimates.getBodyOrientation().getYaw();

      // initialize state machine
      xGaitStateMachine.reset();
   }

   @Override public void onExit()
   {
   }

   private class InitialTransitionState implements FiniteStateMachineState<XGaitEvent>
   {
      double initialTransitionTime;
      private RobotQuadrant initialQuadrant;
      private final FramePoint initialSupportCentroid;

      public InitialTransitionState()
      {
         initialSupportCentroid = new FramePoint();
      }

      @Override public void onEntry()
      {
         initialSupportCentroid.setToZero(supportFrame);
         initialQuadrant = (xGaitSettings.getEndPhaseShift() < 90) ? RobotQuadrant.HIND_LEFT : RobotQuadrant.FRONT_LEFT;
         initialTransitionTime = robotTimestamp.getDoubleValue() + initialTransitionDurationParameter.get();

         // compute initial step plan
         xGaitStepPlanner.computeInitialPlan(xGaitPreviewSteps, inputProvider.getPlanarVelocityInput(),
               initialQuadrant, initialSupportCentroid, initialTransitionTime, bodyYawSetpoint, xGaitSettings);

         for (int i = 0; i < xGaitPreviewSteps.size(); i++)
         {
            xGaitPreviewSteps.get(i).getTimeInterval().shiftInterval(controlDT * i);
         }
         timedStepController.addStep(xGaitPreviewSteps.get(0));
         timedStepController.addStep(xGaitPreviewSteps.get(1));
         timedStepController.addStep(xGaitPreviewSteps.get(2));

         // update dcm height
         dcmPositionController.setComHeight(inputProvider.getComPositionInput().getZ());

         // initialize ground plane points
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            groundPlanePositions.get(robotQuadrant).setIncludingFrame(taskSpaceEstimates.getSolePosition(robotQuadrant));
            groundPlanePositions.get(robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
         }
      }

      @Override public XGaitEvent process()
      {
         if (robotTimestamp.getDoubleValue() > initialTransitionTime - controlDT)
            return XGaitEvent.TIMEOUT;
         else
            return null;
      }

      @Override public void onExit()
      {
      }
   }

   private class ForwardXGaitState implements FiniteStateMachineState<XGaitEvent>, QuadrupedTimedStepTransitionCallback
   {
      private final PiecewiseForwardDcmTrajectory forwardDcmTrajectory;
      private final PiecewiseReverseDcmTrajectory reverseDcmTrajectory;
      private final FramePoint forwardDcmPositionAtEoS;
      private final FramePoint reverseDcmPositionAtEoS;
      private final FramePoint nominalDcmOffsetAtEoS;

      public ForwardXGaitState()
      {
         forwardDcmTrajectory = new PiecewiseForwardDcmTrajectory(2 * xGaitPreviewSteps.size() + 1, gravity, dcmPositionController.getComHeight());
         reverseDcmTrajectory = new PiecewiseReverseDcmTrajectory(2 * xGaitPreviewSteps.size() + 1, gravity, dcmPositionController.getComHeight());
         forwardDcmPositionAtEoS = new FramePoint();
         reverseDcmPositionAtEoS = new FramePoint();
         nominalDcmOffsetAtEoS = new FramePoint();
      }

      @Override public void onEntry()
      {
         double currentTime = robotTimestamp.getDoubleValue();
         timedStepController.registerStepTransitionCallback(this);

         // update dcm height
         dcmPositionController.setComHeight(inputProvider.getComPositionInput().getZ());

         // compute nominal dcm trajectory
         int nIntervals = copPlanner.compute(timedStepController.getQueue(), taskSpaceEstimates.getSolePosition(), taskSpaceControllerSettings.getContactState(), currentTime);
         forwardDcmTrajectory.setComHeight(dcmPositionController.getComHeight());
         forwardDcmTrajectory.initializeTrajectory(nIntervals, copPlanner.getTimeAtStartOfInterval(), copPlanner.getCopAtStartOfInterval(), dcmPositionEstimate);
      }

      @Override public void onLiftOff(RobotQuadrant thisStepQuadrant, QuadrantDependentList<ContactState> contactState)
      {
         double currentTime = robotTimestamp.getDoubleValue();
         RobotQuadrant lastStepQuadrant = thisStepQuadrant.getNextReversedRegularGaitSwingQuadrant();
         RobotQuadrant nextStepQuadrant = thisStepQuadrant.getNextRegularGaitSwingQuadrant();
         QuadrupedTimedStep lastStep = timedStepController.getLatestStep(lastStepQuadrant);
         QuadrupedTimedStep thisStep = timedStepController.getLatestStep(thisStepQuadrant);
         QuadrupedTimedStep nextStep = timedStepController.getLatestStep(nextStepQuadrant);

         if (lastStep == null)
         {
            return;
         }

         xGaitStepPlanner.computeMidStepPlan(xGaitPreviewSteps, lastStep, inputProvider.getPlanarVelocityInput(), currentTime, bodyYawSetpoint, xGaitSettings);
         for (int i = 0; i < xGaitPreviewSteps.size(); i++)
         {
            xGaitPreviewSteps.get(i).getTimeInterval().shiftInterval(controlDT * i);
         }
         thisStep.set(xGaitPreviewSteps.get(0));
         nextStep.set(xGaitPreviewSteps.get(1));
         timedStepController.addStep(xGaitPreviewSteps.get(2));

         // update dcm height
         int nIntervals;
         dcmPositionController.setComHeight(inputProvider.getComPositionInput().getZ());

         // compute forward dcm trajectory
         nIntervals = copPlanner.compute(timedStepController.getQueue(), taskSpaceEstimates.getSolePosition(), contactState, currentTime);
         forwardDcmTrajectory.setComHeight(dcmPositionController.getComHeight());
         forwardDcmTrajectory.initializeTrajectory(nIntervals, copPlanner.getTimeAtStartOfInterval(), copPlanner.getCopAtStartOfInterval(), dcmPositionEstimate);

         // compute reverse dcm trajectory
         nIntervals = copPlanner.compute(xGaitPreviewSteps, taskSpaceEstimates.getSolePosition(), contactState, currentTime);
         reverseDcmTrajectory.setComHeight(dcmPositionController.getComHeight());
         reverseDcmTrajectory.initializeTrajectory(nIntervals, copPlanner.getTimeAtStartOfInterval(), copPlanner.getCopAtStartOfInterval(), copPlanner.getTimeAtStartOfInterval(nIntervals - 1), copPlanner.getCopAtStartOfInterval(nIntervals - 1));

         // adjust current step goal position to compensate for dcm error
         forwardDcmTrajectory.computeTrajectory(thisStep.getTimeInterval().getEndTime());
         reverseDcmTrajectory.computeTrajectory(nextStep.getTimeInterval().getEndTime());
         forwardDcmTrajectory.getPosition(forwardDcmPositionAtEoS);
         reverseDcmTrajectory.getPosition(reverseDcmPositionAtEoS);
         forwardDcmPositionAtEoS.changeFrame(ReferenceFrame.getWorldFrame());
         reverseDcmPositionAtEoS.changeFrame(ReferenceFrame.getWorldFrame());
         nominalDcmOffsetAtEoS.setIncludingFrame(reverseDcmPositionAtEoS);
         nominalDcmOffsetAtEoS.sub(thisStep.getGoalPosition());
         reverseDcmPositionAtEoS.sub(forwardDcmPositionAtEoS);

         double supportTime = nextStep.getTimeInterval().getEndTime() - thisStep.getTimeInterval().getEndTime();
         computeStepGoalPosition(thisStep.getGoalPosition(), lastStep.getGoalPosition(), forwardDcmPositionAtEoS, nominalDcmOffsetAtEoS, supportTime, dcmPositionController.getNaturalFrequency());
         groundPlaneEstimator.projectZ(thisStep.getGoalPosition());
      }

      @Override public void onTouchDown(RobotQuadrant robotQuadrant, QuadrantDependentList<ContactState> contactState)
      {
      }

      @Override public XGaitEvent process()
      {
         // compute nominal dcm trajectory
         forwardDcmTrajectory.computeTrajectory(robotTimestamp.getDoubleValue());
         forwardDcmTrajectory.getPosition(dcmPositionControllerSetpoints.getDcmPosition());
         forwardDcmTrajectory.getVelocity(dcmPositionControllerSetpoints.getDcmVelocity());
         return null;
      }

      @Override public void onExit()
      {
      }
   }

   private class ReverseXGaitState implements FiniteStateMachineState<XGaitEvent>, QuadrupedTimedStepTransitionCallback
   {
      public ReverseXGaitState()
      {
      }

      @Override public void onEntry()
      {
      }

      @Override public void onLiftOff(RobotQuadrant robotQuadrant, QuadrantDependentList<ContactState> contactState)
      {
      }

      @Override public void onTouchDown(RobotQuadrant robotQuadrant, QuadrantDependentList<ContactState> contactState)
      {
      }

      @Override public XGaitEvent process()
      {
         return null;
      }

      @Override public void onExit()
      {
      }
   }

   private void computeStepGoalPosition(Point3d thisGoalPosition, Point3d lastGoalPosition, FramePoint dcmPositionAtEoTS, FramePoint dcmOffsetAtEoNS, double supportTime, double naturalFrequency)
   {
      double exp = Math.exp(naturalFrequency * supportTime);

      dcmPositionAtEoTS.scale(exp);
      dcmPositionAtEoTS.sub(dcmOffsetAtEoNS);
      dcmPositionAtEoTS.scale(2 / (1 + exp));
      thisGoalPosition.set(lastGoalPosition);
      thisGoalPosition.scale((1 - exp) / (1 + exp));
      thisGoalPosition.add(dcmPositionAtEoTS.getPoint());
   }
}
