package us.ihmc.quadrupedRobotics.controller.force.states;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.ConstrainedQPSolver;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.OASESConstrainedQPSolver;
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
import us.ihmc.quadrupedRobotics.providers.QuadrupedControllerInputProviderInterface;
import us.ihmc.quadrupedRobotics.providers.QuadrupedXGaitSettingsProvider;
import us.ihmc.quadrupedRobotics.util.PreallocatedQueue;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.*;
import us.ihmc.tools.exceptions.NoConvergenceException;

import java.util.ArrayList;

public class QuadrupedMpcBasedXGaitController implements QuadrupedController, QuadrupedTimedStepTransitionCallback
{
   private final QuadrupedControllerInputProviderInterface inputProvider;
   private final QuadrupedXGaitSettingsProvider settingsProvider;
   private final DoubleYoVariable robotTimestamp;
   private final double controlDT;
   private final double gravity;
   private final double mass;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   // parameters
   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleParameter jointDampingParameter = parameterFactory.createDouble("jointDamping", 2);
   private final DoubleArrayParameter bodyOrientationProportionalGainsParameter = parameterFactory.createDoubleArray("bodyOrientationProportionalGains", 5000, 5000, 5000);
   private final DoubleArrayParameter bodyOrientationDerivativeGainsParameter = parameterFactory.createDoubleArray("bodyOrientationDerivativeGains", 750, 750, 750);
   private final DoubleArrayParameter bodyOrientationIntegralGainsParameter = parameterFactory.createDoubleArray("bodyOrientationIntegralGains", 0, 0, 0);
   private final DoubleParameter bodyOrientationMaxIntegralErrorParameter = parameterFactory.createDouble("bodyOrientationMaxIntegralError", 0);
   private final DoubleArrayParameter comPositionProportionalGainsParameter = parameterFactory.createDoubleArray("comPositionProportionalGains", 0, 0, 5000);
   private final DoubleArrayParameter comPositionDerivativeGainsParameter = parameterFactory.createDoubleArray("comPositionDerivativeGains", 0, 0, 750);
   private final DoubleArrayParameter comPositionIntegralGainsParameter = parameterFactory.createDoubleArray("comPositionIntegralGains", 0, 0, 0);
   private final DoubleParameter comPositionMaxIntegralErrorParameter = parameterFactory.createDouble("comPositionMaxIntegralError", 0);
   private final DoubleParameter initialTransitionDurationParameter = parameterFactory.createDouble("initialTransitionDuration", 0.25);
   private final DoubleParameter footholdDistanceLowerLimitParameter = parameterFactory.createDouble("footholdDistanceLowerLimit", 0.15);

   // frames
   private final ReferenceFrame supportFrame;
   private final ReferenceFrame worldFrame;

   // model
   private final LinearInvertedPendulumModel lipModel;

   // feedback controllers
   private final FramePoint cmpPositionSetpoint;
   private final FramePoint dcmPositionEstimate;
   private final DivergentComponentOfMotionEstimator dcmPositionEstimator;
   private final QuadrupedComPositionController.Setpoints comPositionControllerSetpoints;
   private final QuadrupedComPositionController comPositionController;
   private final QuadrupedBodyOrientationController.Setpoints bodyOrientationControllerSetpoints;
   private final QuadrupedBodyOrientationController bodyOrientationController;
   private final QuadrupedTimedStepController timedStepController;

   // task space controller
   private final QuadrupedTaskSpaceEstimator.Estimates taskSpaceEstimates;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final QuadrupedTaskSpaceController.Commands taskSpaceControllerCommands;
   private final QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings;
   private final QuadrupedTaskSpaceController taskSpaceController;

   // planning
   private static int NUMBER_OF_PREVIEW_STEPS = 16;
   private double bodyYawSetpoint;
   private final GroundPlaneEstimator groundPlaneEstimator;
   private final QuadrantDependentList<FramePoint> groundPlanePositions;
   private final QuadrupedXGaitSettings xGaitSettings;
   private final QuadrupedXGaitPlanner xGaitStepPlanner;
   private final ArrayList<QuadrupedTimedStep> xGaitPreviewSteps;
   private final QuadrupedTimedStepPressurePlanner timedStepPressurePlanner;
   private final FramePoint supportCentroid;
   private EndDependentList<QuadrupedTimedStep> latestSteps;
   private final FrameVector stepAdjustment;


   public QuadrupedMpcBasedXGaitController(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedForceControllerToolbox controllerToolbox,
         QuadrupedControllerInputProviderInterface inputProvider, QuadrupedXGaitSettingsProvider settingsProvider)
   {
      this.inputProvider = inputProvider;
      this.settingsProvider = settingsProvider;
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
      cmpPositionSetpoint = new FramePoint();
      dcmPositionEstimate = new FramePoint();
      dcmPositionEstimator = controllerToolbox.getDcmPositionEstimator();
      comPositionControllerSetpoints = new QuadrupedComPositionController.Setpoints();
      comPositionController = controllerToolbox.getComPositionController();
      bodyOrientationControllerSetpoints = new QuadrupedBodyOrientationController.Setpoints();
      bodyOrientationController = controllerToolbox.getBodyOrientationController();
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
      xGaitStepPlanner = new QuadrupedXGaitPlanner();
      xGaitPreviewSteps = new ArrayList<>(NUMBER_OF_PREVIEW_STEPS);
      for (int i = 0; i < NUMBER_OF_PREVIEW_STEPS; i++)
      {
         xGaitPreviewSteps.add(new QuadrupedTimedStep());
      }
      xGaitSettings = new QuadrupedXGaitSettings();
      timedStepPressurePlanner = new QuadrupedTimedStepPressurePlanner(NUMBER_OF_PREVIEW_STEPS + 4);
      supportCentroid = new FramePoint();
      stepAdjustment = new FrameVector();
      latestSteps = new EndDependentList<>();
      for (RobotEnd robotEnd : RobotEnd.values)
      {
         latestSteps.set(robotEnd, new QuadrupedTimedStep());
      }

      runtimeEnvironment.getParentRegistry().addChild(registry);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   private void updateEstimates()
   {
      // update model
      lipModel.setComHeight(inputProvider.getComPositionInput().getZ());

      // update task space estimates
      taskSpaceEstimator.compute(taskSpaceEstimates);

      // update dcm estimate
      dcmPositionEstimator.compute(dcmPositionEstimate, taskSpaceEstimates.getComVelocity());
   }

   private void updateSetpoints()
   {
      double currentTime = robotTimestamp.getDoubleValue();

      // update step plan
      xGaitStepPlanner.computeOnlinePlan(xGaitPreviewSteps, latestSteps, inputProvider.getPlanarVelocityInput(), currentTime, bodyYawSetpoint, xGaitSettings);
      timedStepController.removeSteps();
      for (RobotEnd robotEnd : RobotEnd.values)
      {
         timedStepController.addStep(latestSteps.get(robotEnd));
      }
      for (int i = 0; i < xGaitPreviewSteps.size(); i++)
      {
         timedStepController.addStep(xGaitPreviewSteps.get(i));
      }

      // update desired horizontal com forces
      lipModel.computeComForce(taskSpaceControllerCommands.getComForce(), cmpPositionSetpoint);
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
      bodyYawSetpoint += inputProvider.getPlanarVelocityInput().getZ() * controlDT;
      bodyOrientationControllerSetpoints.getBodyOrientation().changeFrame(worldFrame);
      bodyOrientationControllerSetpoints.getBodyOrientation().setYawPitchRoll(
            RotationTools.computeYaw(inputProvider.getBodyOrientationInput()) + bodyYawSetpoint,
            RotationTools.computePitch(inputProvider.getBodyOrientationInput()) + groundPlaneEstimator.getPitch(bodyYawSetpoint),
            RotationTools.computeRoll(inputProvider.getBodyOrientationInput()));
      bodyOrientationControllerSetpoints.getBodyAngularVelocity().setToZero();
      bodyOrientationControllerSetpoints.getComTorqueFeedforward().setToZero();
      bodyOrientationController.compute(taskSpaceControllerCommands.getComTorque(), bodyOrientationControllerSetpoints, taskSpaceEstimates);

      // update desired contact state and sole forces
      timedStepController.compute(taskSpaceControllerSettings.getContactState(), taskSpaceControllerCommands.getSoleForce(), taskSpaceEstimates);

      // update joint setpoints
      taskSpaceController.compute(taskSpaceControllerSettings, taskSpaceControllerCommands);

      // update step plan
      computeStepAdjustmentAndCopSetpoint(stepAdjustment, cmpPositionSetpoint, timedStepController.getQueue(), taskSpaceEstimates.getSolePosition(), taskSpaceControllerSettings.getContactState(), dcmPositionEstimate, currentTime, lipModel.getNaturalFrequency());
      for (RobotEnd robotEnd : RobotEnd.values)
      {
         latestSteps.get(robotEnd).getGoalPosition().add(stepAdjustment.getVector());
         groundPlaneEstimator.projectZ(latestSteps.get(robotEnd).getGoalPosition());
      }
      for (int i = 0; i < timedStepController.getQueue().size(); i++)
      {
         timedStepController.getQueue().get(i).getGoalPosition().add(stepAdjustment.getVector());
         groundPlaneEstimator.projectZ(timedStepController.getQueue().get(i).getGoalPosition());
      }
   }

   private void updateSettings()
   {
      settingsProvider.getXGaitSettings(xGaitSettings);

      // increase stance dimensions to prevent self collisions
      double strideLength = Math.abs(2 * inputProvider.getPlanarVelocityInput().getX() * xGaitSettings.getStepDuration());
      double strideWidth = Math.abs(2 * inputProvider.getPlanarVelocityInput().getY() * xGaitSettings.getStepDuration());
      strideLength += Math.abs(xGaitSettings.getStanceWidth() / 2 * Math.sin(2 * inputProvider.getPlanarVelocityInput().getZ() * xGaitSettings.getStepDuration()));
      strideWidth += Math.abs(xGaitSettings.getStanceLength() / 2 * Math.sin(2 * inputProvider.getPlanarVelocityInput().getZ() * xGaitSettings.getStepDuration()));
      xGaitSettings.setStanceLength(Math.max(xGaitSettings.getStanceLength(), strideLength / 2 + footholdDistanceLowerLimitParameter.get()));
      xGaitSettings.setStanceWidth(Math.max(xGaitSettings.getStanceWidth(), strideWidth / 2 + footholdDistanceLowerLimitParameter.get()));
   }

   @Override public void onLiftOff(RobotQuadrant thisStepQuadrant, QuadrantDependentList<ContactState> thisContactState)
   {
      // update ground plane estimate
      groundPlanePositions.get(thisStepQuadrant).setIncludingFrame(taskSpaceEstimates.getSolePosition(thisStepQuadrant));
      groundPlanePositions.get(thisStepQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
      groundPlaneEstimator.compute(groundPlanePositions);

      // update latest step
      RobotEnd thisStepEnd = thisStepQuadrant.getEnd();
      QuadrupedTimedStep thisStep = timedStepController.getLatestStep(thisStepQuadrant);
      latestSteps.get(thisStepEnd).set(thisStep);
   }

   @Override public void onTouchDown(RobotQuadrant thisStepQuadrant, QuadrantDependentList<ContactState> thisContactState)
   {
      latestSteps.getClass();
   }

   @Override public void onEntry()
   {
      updateSettings();
      updateEstimates();

      // initialize feedback controllers
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
      timedStepController.reset();
      timedStepController.registerStepTransitionCallback(this);

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

      // initialize ground plane
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         groundPlanePositions.get(robotQuadrant).setIncludingFrame(taskSpaceEstimates.getSolePosition(robotQuadrant));
         groundPlanePositions.get(robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
      }
      groundPlaneEstimator.compute(groundPlanePositions);

      // initialize step plan
      supportCentroid.setToZero(supportFrame);
      double initialStepStartTime = robotTimestamp.getDoubleValue() + initialTransitionDurationParameter.get();
      RobotQuadrant initialQuadrant = (xGaitSettings.getEndPhaseShift() < 90) ? RobotQuadrant.HIND_LEFT : RobotQuadrant.FRONT_LEFT;
      xGaitStepPlanner.computeInitialPlan(xGaitPreviewSteps, inputProvider.getPlanarVelocityInput(), initialQuadrant, supportCentroid, initialStepStartTime, bodyYawSetpoint, xGaitSettings);
      for (int i = 0; i < xGaitPreviewSteps.size(); i++)
      {
         timedStepController.addStep(xGaitPreviewSteps.get(i));
      }
      for (RobotEnd robotEnd : RobotEnd.values)
      {
         latestSteps.get(robotEnd).set(timedStepController.getLatestStep(robotEnd));
      }
   }

   @Override public ControllerEvent process()
   {
      updateSettings();
      updateEstimates();
      updateSetpoints();
      return null;
   }

   @Override public void onExit()
   {
      timedStepController.removeSteps();
      timedStepController.registerStepTransitionCallback(null);
   }

   private final DenseMatrix64F x0 = new DenseMatrix64F(100, 1);
   private final DenseMatrix64F y0 = new DenseMatrix64F(2, 1);
   private final DenseMatrix64F B = new DenseMatrix64F(100, 6);
   private final DenseMatrix64F C = new DenseMatrix64F(2, 100);
   private final DenseMatrix64F S = new DenseMatrix64F(2, 100);
   private final DenseMatrix64F CmS = new DenseMatrix64F(2, 100);
   private final DenseMatrix64F CmSB = new DenseMatrix64F(2, 6);
   private final DenseMatrix64F CmSx0py0 = new DenseMatrix64F(2, 1);
   private final DenseMatrix64F qpSolutionVector = new DenseMatrix64F(6, 1);
   private final DenseMatrix64F qpCostVector = new DenseMatrix64F(100, 1);
   private final DenseMatrix64F qpCostMatrix = new DenseMatrix64F(100, 100);
   private final DenseMatrix64F qpEqualityVector = new DenseMatrix64F(100, 1);
   private final DenseMatrix64F qpEqualityMatrix = new DenseMatrix64F(100, 100);
   private final DenseMatrix64F qpInequalityVector = new DenseMatrix64F(100, 1);
   private final DenseMatrix64F qpInequalityMatrix = new DenseMatrix64F(100, 100);
   private final ConstrainedQPSolver qpSolver = new OASESConstrainedQPSolver(null);

   /**
    * Compute optimal step adjustment and center of pressure setpoint using model predictive control.
    * @param stepAdjustment output step adjustment vector
    * @param copPositionSetpoint output center of pressure setpoint
    * @param queuedSteps queue of ongoing and upcoming steps
    * @param currentSolePosition current sole position for each quadrant
    * @param currentContactState current contact state for each quadrant
    * @param currentDcmEstimate current divergent component of motion estimate
    * @param currentTime current time in seconds
    * @param naturalFrequency natural frequency of the divergent component of motion
    */
   private void computeStepAdjustmentAndCopSetpoint(FrameVector stepAdjustment, FramePoint copPositionSetpoint, PreallocatedQueue<QuadrupedTimedStep> queuedSteps, QuadrantDependentList<FramePoint> currentSolePosition, QuadrantDependentList<ContactState> currentContactState, FramePoint currentDcmEstimate, double currentTime, double naturalFrequency)
   {
      // Compute step adjustment and contact pressure by solving the following QP:
      // min_u u'Au
      // s.t
      // (C - S)Bu + (C - S)x0 + y0 = 0
      // u0 + u1 + u2 + u3 - 1 = 0
      // u0 >= 0
      // u1 >=0
      // u2 >= 0
      // u3 >= 0
      // where u = [u0, u1, u2, u3, u4, u5]',
      // u0, u1, u2, u3 are the normalized contact pressures for each quadrant and
      // u4, u5 are the x and y step adjustment in meters

      int rowOffset, columnOffset;
      stepAdjustment.changeFrame(worldFrame);
      currentDcmEstimate.changeFrame(worldFrame);

      int nContacts  = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (currentContactState.get(robotQuadrant) == ContactState.IN_CONTACT)
         {
            nContacts++;
         }
      }

      int nPreviewSteps = MathTools.clipToMinMax((int) (10 /xGaitSettings.getEndDoubleSupportDuration() + xGaitSettings.getStepDuration()), 1, queuedSteps.size());
      int nIntervals = timedStepPressurePlanner.compute(nPreviewSteps, queuedSteps, currentSolePosition, currentContactState, currentTime);
      x0.reshape(2 * nIntervals, 1);             // center of pressure offset
      y0.reshape(2, 1);                          // final divergent component of motion offset
      B.reshape(2 * nIntervals, nContacts + 2);  // center of pressure map
      C.reshape(2, 2 * nIntervals);              // final divergent component of motion map
      S.reshape(2, 2 * nIntervals);              // final interval selection matrix
      B.zero();
      C.zero();
      S.zero();
      rowOffset = 0;
      for (Direction direction : Direction.values2D())
      {
         columnOffset = 0;
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            if (currentContactState.get(robotQuadrant) == ContactState.IN_CONTACT)
            {
               currentSolePosition.get(robotQuadrant).changeFrame(worldFrame);
               B.set(rowOffset, columnOffset, currentSolePosition.get(robotQuadrant).get(direction));
               columnOffset++;
            }
         }

         for (int i = 0; i < nIntervals; i++)
         {
            timedStepPressurePlanner.getCenterOfPressureAtStartOfInterval(i).changeFrame(worldFrame);
            x0.set(i * 2 + rowOffset, 0, timedStepPressurePlanner.getCenterOfPressureAtStartOfInterval(i).get(direction));
            B.set(i * 2 + rowOffset, nContacts + rowOffset, timedStepPressurePlanner.getNormalizedPressureContributedByQueuedSteps(i));
         }
         x0.set(rowOffset, 0, 0);

         for (int i = nIntervals - 2; i >= 0; i--)
         {
            double expn = Math.exp(naturalFrequency * (timedStepPressurePlanner.getTimeAtStartOfInterval(nIntervals - 1) - timedStepPressurePlanner.getTimeAtStartOfInterval(i + 1)));
            double expi = Math.exp(naturalFrequency * (timedStepPressurePlanner.getTimeAtStartOfInterval(i + 1) - timedStepPressurePlanner.getTimeAtStartOfInterval(i)));
            C.set(rowOffset, i * 2 + rowOffset, expn * (1 - expi));
         }
         C.set(rowOffset, 2 * nIntervals - 2 + rowOffset, 0);
         S.set(rowOffset, 2 * nIntervals - 2 + rowOffset, 1);

         double previewTime = timedStepPressurePlanner.getTimeAtStartOfInterval(nIntervals - 1) - timedStepPressurePlanner.getTimeAtStartOfInterval(0);
         y0.set(rowOffset, 0, Math.exp(naturalFrequency * previewTime) * currentDcmEstimate.get(direction));
         rowOffset++;
      }

      CmS.reshape(2, 2 * nIntervals);
      CmSB.reshape(2, nContacts + 2);
      CmSx0py0.reshape(2, 1);
      CommonOps.subtract(C, S, CmS);
      CommonOps.mult(CmS, B, CmSB);
      CmSx0py0.set(y0);
      CommonOps.multAdd(CmS, x0, CmSx0py0);

      DenseMatrix64F u = qpSolutionVector;
      DenseMatrix64F beq = qpEqualityVector;
      DenseMatrix64F Aeq = qpEqualityMatrix;
      DenseMatrix64F bin = qpInequalityVector;
      DenseMatrix64F Ain = qpInequalityMatrix;
      DenseMatrix64F b = qpCostVector;
      DenseMatrix64F A = qpCostMatrix;

      // Initialize solution vector.
      u.reshape(nContacts + 2, 1);

      // Initialize equality constraints. (Aeq u = beq)
      beq.reshape(3, 1);
      beq.zero();
      beq.set(0, 0, -CmSx0py0.get(0, 0));
      beq.set(1, 0, -CmSx0py0.get(1, 0));
      beq.set(2, 0, 1);
      Aeq.reshape(3, nContacts + 2);
      Aeq.zero();
      for (int i = 0; i < nContacts + 2; i++)
      {
         Aeq.set(0, i, CmSB.get(0, i));
         Aeq.set(1, i, CmSB.get(1, i));
      }
      for (int i = 0; i < nContacts; i++)
      {
         Aeq.set(2, i, 1);
      }

      // Initialize inequality constraints. (Ain u <= bin)
      bin.reshape(nContacts, 1);
      bin.zero();
      Ain.reshape(nContacts, nContacts + 2);
      Ain.zero();
      for (int i = 0; i < nContacts; i++)
      {
         Ain.set(i, i, -1);
      }

      // Initialize cost terms. (min_u u'Au + b'u)
      b.reshape(nContacts + 2, 1);
      b.zero();
      A.reshape(nContacts + 2, nContacts + 2);
      A.zero();

      double copRegularization = 1;
      double stepAdjustmentRegularization = 10000;
      for (int i = 0; i < nContacts; i++)
      {
         A.set(i, i, copRegularization);
      }
      for (int i = nContacts; i < nContacts + 2; i++)
      {
         A.set(i, i, stepAdjustmentRegularization);
      }

      // Solve constrained quadratic program.
      try
      {
         qpSolver.solve(A, b, Aeq, beq, Ain, bin, u, false);
      }
      catch (NoConvergenceException e)
      {
         System.err.println("NoConvergenceException: " + e.getMessage());
      }

      rowOffset = 0;
      copPositionSetpoint.setToZero(worldFrame);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (currentContactState.get(robotQuadrant) == ContactState.IN_CONTACT)
         {
            double normalizedContactPressure = u.get(rowOffset++, 0);
            currentSolePosition.get(robotQuadrant).changeFrame(worldFrame);
            addPointWithScaleFactor(copPositionSetpoint, currentSolePosition.get(robotQuadrant), normalizedContactPressure);
         }
      }

      for (Direction direction : Direction.values2D())
      {
         stepAdjustment.set(direction, u.get(rowOffset++, 0));
      }
   }

   private void addPointWithScaleFactor(FramePoint point, FramePoint pointToAdd, double scaleFactor)
   {
      point.checkReferenceFrameMatch(pointToAdd);
      point.add(scaleFactor * pointToAdd.getX(), scaleFactor * pointToAdd.getY(), scaleFactor * pointToAdd.getZ());
   }
}
