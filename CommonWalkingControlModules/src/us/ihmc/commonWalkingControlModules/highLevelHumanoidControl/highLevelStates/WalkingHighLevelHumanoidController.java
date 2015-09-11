package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.SdfLoader.partNames.LimbName;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoFramePoint2dInPolygonCoordinate;
import us.ihmc.commonWalkingControlModules.captureRegion.PushRecoveryControlModule;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.ChestOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.LegSingularityAndKneeCollapseAvoidanceControlModule;
import us.ihmc.commonWalkingControlModules.controllers.roughTerrain.FootExplorationControlModule;
import us.ihmc.commonWalkingControlModules.desiredFootStep.AbortWalkingProvider;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepProvider;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsDataVisualizer;
import us.ihmc.commonWalkingControlModules.desiredFootStep.UpcomingFootstepList;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingManagers;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.CapturePointPlannerAdapter;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlModuleBridge.MomentumControlModuleType;
import us.ihmc.commonWalkingControlModules.packetConsumers.AutomaticManipulationAbortCommunicator;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredFootStateProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.FootPoseProvider;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.HeelSwitch;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.ToeSwitch;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightPartialDerivativesData;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesData;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesSmoother;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.CoMXYTimeDerivativesData;
import us.ihmc.commonWalkingControlModules.trajectories.ContactStatesAndUpcomingFootstepData;
import us.ihmc.commonWalkingControlModules.trajectories.SwingTimeCalculationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.TransferTimeCalculationProvider;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.ConvexPolygonShrinker;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.stateMachines.State;
import us.ihmc.robotics.stateMachines.StateChangedListener;
import us.ihmc.robotics.stateMachines.StateMachine;
import us.ihmc.robotics.stateMachines.StateTransition;
import us.ihmc.robotics.stateMachines.StateTransitionAction;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;
import us.ihmc.yoUtilities.controllers.PDController;
import us.ihmc.yoUtilities.controllers.YoPDGains;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;

public class WalkingHighLevelHumanoidController extends AbstractHighLevelHumanoidControlPattern
{
   private boolean VISUALIZE = true;

   private static final boolean DESIREDICP_FROM_POLYGON_COORDINATE = false;
   private static final boolean CHECK_FOR_OVEREXTENSION_ON_TOE_OFF_LEG = false;

   private final static HighLevelState controllerState = HighLevelState.WALKING;
   private final static MomentumControlModuleType MOMENTUM_CONTROL_MODULE_TO_USE = MomentumControlModuleType.OPT_NULLSPACE;

   private final PushRecoveryControlModule pushRecoveryModule;

   public final static boolean DEBUG = false;

   private final StateMachine<WalkingState> stateMachine;
   private final CenterOfMassJacobian centerOfMassJacobian;

   private final CoMHeightTrajectoryGenerator centerOfMassHeightTrajectoryGenerator;
   private final CoMHeightTimeDerivativesCalculator coMHeightTimeDerivativesCalculator = new CoMHeightTimeDerivativesCalculator();
   private final CoMHeightTimeDerivativesSmoother coMHeightTimeDerivativesSmoother;
   private final DoubleYoVariable desiredCoMHeightFromTrajectory = new DoubleYoVariable("desiredCoMHeightFromTrajectory", registry);
   private final DoubleYoVariable desiredCoMHeightVelocityFromTrajectory = new DoubleYoVariable("desiredCoMHeightVelocityFromTrajectory", registry);
   private final DoubleYoVariable desiredCoMHeightAccelerationFromTrajectory = new DoubleYoVariable("desiredCoMHeightAccelerationFromTrajectory", registry);
   //   private final DoubleYoVariable desiredCoMHeightBeforeSmoothing = new DoubleYoVariable("desiredCoMHeightBeforeSmoothing", registry);
   //   private final DoubleYoVariable desiredCoMHeightVelocityBeforeSmoothing = new DoubleYoVariable("desiredCoMHeightVelocityBeforeSmoothing", registry);
   //   private final DoubleYoVariable desiredCoMHeightAccelerationBeforeSmoothing = new DoubleYoVariable("desiredCoMHeightAccelerationBeforeSmoothing", registry);
   private final DoubleYoVariable desiredCoMHeightCorrected = new DoubleYoVariable("desiredCoMHeightCorrected", registry);
   private final DoubleYoVariable desiredCoMHeightVelocityCorrected = new DoubleYoVariable("desiredCoMHeightVelocityCorrected", registry);
   private final DoubleYoVariable desiredCoMHeightAccelerationCorrected = new DoubleYoVariable("desiredCoMHeightAccelerationCorrected", registry);
   private final DoubleYoVariable desiredCoMHeightAfterSmoothing = new DoubleYoVariable("desiredCoMHeightAfterSmoothing", registry);
   private final DoubleYoVariable desiredCoMHeightVelocityAfterSmoothing = new DoubleYoVariable("desiredCoMHeightVelocityAfterSmoothing", registry);
   private final DoubleYoVariable desiredCoMHeightAccelerationAfterSmoothing = new DoubleYoVariable("desiredCoMHeightAccelerationAfterSmoothing", registry);

   private final PDController centerOfMassHeightController;

   private final YoFramePoint2d transferToFootstep = new YoFramePoint2d("transferToFootstep", worldFrame, registry);

   private final BooleanYoVariable resetIntegratorsAfterSwing = new BooleanYoVariable("resetIntegratorsAfterSwing", registry);
   private final BooleanYoVariable alwaysIntegrateAnkleAcceleration = new BooleanYoVariable("alwaysIntegrateAnkleAcceleration", registry);

   private final DoubleYoVariable stopInDoubleSupporTrajectoryTime = new DoubleYoVariable("stopInDoubleSupporTrajectoryTime", registry);
   private final DoubleYoVariable dwellInSingleSupportDuration = new DoubleYoVariable("dwellInSingleSupportDuration",
         "Amount of time to stay in single support after the ICP trajectory is done if you haven't registered a touchdown yet", registry);

   private final BooleanYoVariable loopControllerForever = new BooleanYoVariable("loopControllerForever", "For checking memory and profiling", registry);
   private final BooleanYoVariable justFall = new BooleanYoVariable("justFall", registry);

   private final BooleanYoVariable controlPelvisHeightInsteadOfCoMHeight = new BooleanYoVariable("controlPelvisHeightInsteadOfCoMHeight", registry);

   private final BooleanYoVariable hasMinimumTimePassed = new BooleanYoVariable("hasMinimumTimePassed", registry);
   private final DoubleYoVariable minimumSwingFraction = new DoubleYoVariable("minimumSwingFraction", registry);

   private final BooleanYoVariable hasICPPlannerFinished = new BooleanYoVariable("hasICPPlannerFinished", registry);
   private final DoubleYoVariable timeThatICPPlannerFinished = new DoubleYoVariable("timeThatICPPlannerFinished", registry);
   private final BooleanYoVariable initializingICPTrajectory = new BooleanYoVariable("initializingICPTrajectory", registry);

   // private final FinalDesiredICPCalculator finalDesiredICPCalculator;

   private final BooleanYoVariable rememberFinalICPFromSingleSupport = new BooleanYoVariable("rememberFinalICPFromSingleSupport", registry);
   private final YoFramePoint2d finalDesiredICPInWorld = new YoFramePoint2d("finalDesiredICPInWorld", "", worldFrame, registry);

   private final SideDependentList<FootSwitchInterface> footSwitches;

   private final DoubleYoVariable maxICPErrorBeforeSingleSupportX = new DoubleYoVariable("maxICPErrorBeforeSingleSupportX", registry);
   private final DoubleYoVariable maxICPErrorBeforeSingleSupportY = new DoubleYoVariable("maxICPErrorBeforeSingleSupportY", registry);

   private final SwingTimeCalculationProvider swingTimeCalculationProvider;
   private final TransferTimeCalculationProvider transferTimeCalculationProvider;

   private final BooleanYoVariable readyToGrabNextFootstep = new BooleanYoVariable("readyToGrabNextFootstep", registry);

   private final EnumYoVariable<RobotSide> previousSupportSide = new EnumYoVariable<RobotSide>("previousSupportSide", registry, RobotSide.class);

   private final YoFrameVector2d icpAdjustment = new YoFrameVector2d("icpAdjustment", null, registry);

   private final CapturePointPlannerAdapter capturePointPlannerAdapter;

   private final BooleanYoVariable icpTrajectoryHasBeenInitialized;

   private final UpcomingFootstepList upcomingFootstepList;
   private final FootPoseProvider footPoseProvider;
   private final DesiredFootStateProvider desiredFootStateProvider;

   private final ICPAndMomentumBasedController icpAndMomentumBasedController;

   private final EnumYoVariable<RobotSide> upcomingSupportLeg;
   private final EnumYoVariable<RobotSide> supportLeg;
   private final BipedSupportPolygons bipedSupportPolygons;
   private final YoFramePoint capturePoint;
   private final YoFramePoint2d desiredICP;
   private final YoFrameVector2d desiredICPVelocity;

   private final FramePoint tmpFramePoint = new FramePoint(worldFrame);
   private final FramePoint2d tempFramePoint2d = new FramePoint2d();

   private final DoubleYoVariable controlledCoMHeightAcceleration;

   private final TransferToAndNextFootstepsDataVisualizer transferToAndNextFootstepsDataVisualizer;

   private final BooleanYoVariable doneFinishingSingleSupportTransfer = new BooleanYoVariable("doneFinishingSingleSupportTransfer", registry);
   private final BooleanYoVariable footstepListHasBeenUpdated = new BooleanYoVariable("footstepListHasBeenUpdated", registry);
   private final BooleanYoVariable stayInTransferWalkingState = new BooleanYoVariable("stayInTransferWalkingState", registry);

   private final BooleanYoVariable ecmpBasedToeOffHasBeenInitialized = new BooleanYoVariable("ecmpBasedToeOffHasBeenInitialized", registry);
   private final YoFramePoint2d desiredECMP = new YoFramePoint2d("desiredECMP", "", worldFrame, registry);
   private final BooleanYoVariable desiredECMPinSupportPolygon = new BooleanYoVariable("desiredECMPinSupportPolygon", registry);
   private final YoFramePoint ecmpViz = new YoFramePoint("ecmpViz", worldFrame, registry);

   private final YoFramePoint2dInPolygonCoordinate doubleSupportDesiredICP;
   
   private final BooleanYoVariable preparingForLocomotion = new BooleanYoVariable("preparingForLocomotion", registry);
   private final DoubleYoVariable timeToGetPreparedForLocomotion = new DoubleYoVariable("timeToGetPreparedForLocomotion", registry);
   private final DoubleYoVariable preparingForLocomotionStartTime = new DoubleYoVariable("preparingForLocomotionStartTime", registry);
   private final BooleanYoVariable doPrepareManipulationForLocomotion = new BooleanYoVariable("doPrepareManipulationForLocomotion", registry);
   private final BooleanYoVariable doPreparePelvisForLocomotion = new BooleanYoVariable("doPreparePelvisForLocomotion", registry);

   private final BooleanYoVariable isInFlamingoStance = new BooleanYoVariable("isInFlamingoStance", registry);

   private final DoubleYoVariable[] unconstrainedDesiredPositions;

   private final FootExplorationControlModule footExplorationControlModule;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final BooleanYoVariable hasWalkingControllerBeenInitialized = new BooleanYoVariable("hasWalkingControllerBeenInitialized", registry);

   private final DoubleYoVariable remainingSwingTimeAccordingToPlan = new DoubleYoVariable("remainingSwingTimeAccordingToPlan", registry);
   private final DoubleYoVariable estimatedRemainingSwingTimeUnderDisturbance = new DoubleYoVariable("estimatedRemainingSwingTimeUnderDisturbance", registry);
   private final DoubleYoVariable icpErrorThresholdToSpeedUpSwing = new DoubleYoVariable("icpErrorThresholdToSpeedUpSwing", registry);

   private final BooleanYoVariable loadFoot = new BooleanYoVariable("loadFoot", registry);
   private final DoubleYoVariable loadFootStartTime = new DoubleYoVariable("loadFootStartTime", registry);
   private final DoubleYoVariable loadFootDuration = new DoubleYoVariable("loadFootDuration", registry);
   private final DoubleYoVariable loadFootTransferDuration = new DoubleYoVariable("loadFootTransferDuration", registry);
   
   private final BooleanYoVariable hasICPPlannerBeenInitializedAtStart = new BooleanYoVariable("hasICPPlannerBeenInitializedAtStart", registry);
   
   @Deprecated
   private final BooleanYoVariable useICPPlannerHackN13 = new BooleanYoVariable("useICPPlannerHackN13", registry);
   @Deprecated
   private final DoubleYoVariable amountToMoveICPAway = new DoubleYoVariable("amountToMoveICPAway", registry);
   @Deprecated
   private final DoubleYoVariable percentOfSwingTimeRemainingForICPMove = new DoubleYoVariable("percentOfSwingTimeRemainingForICPMove", registry);
   @Deprecated
   private final DoubleYoVariable distanceFromLineToOriginalICP = new DoubleYoVariable("distanceFromLineToOriginalICP", registry);
   @Deprecated
   private final DoubleYoVariable swingTimeRemainingForICPMoveViz = new DoubleYoVariable("swingTimeRemainingForICPMoveViz", registry);
   @Deprecated
   private final DoubleYoVariable moveICPAwayDuringSwingDistance = new DoubleYoVariable("moveICPAwayDuringSwingDistance", registry);
   @Deprecated
   private final DoubleYoVariable moveICPAwayAtEndOfSwingDistance = new DoubleYoVariable("moveICPAwayAtEndOfSwingDistance", registry);
   @Deprecated
   private final DoubleYoVariable singleSupportTimeLeftBeforeShift = new DoubleYoVariable("singleSupportTimeLeftBeforeShift", registry);

   private final DoubleYoVariable timeICPPlannerFinishedAt;
   private final DoubleYoVariable desiredICPVelocityReductionDuration;
   private final DoubleYoVariable desiredICPVelocityRedutionFactor;

   private final ConvexPolygonShrinker convexPolygonShrinker = new ConvexPolygonShrinker();
   private final FrameConvexPolygon2d shrunkSupportPolygon = new FrameConvexPolygon2d();
   private final DoubleYoVariable distanceToShrinkSupportPolygonWhenHoldingCurrent = new DoubleYoVariable("distanceToShrinkSupportPolygonWhenHoldingCurrent", registry);
   private final BooleanYoVariable holdICPToCurrentCoMLocationInNextDoubleSupport = new BooleanYoVariable("holdICPToCurrentCoMLocationInNextDoubleSupport", registry);

   private final BooleanYoVariable enablePushRecoveryOnFailure = new BooleanYoVariable("enablePushRecoveryOnFailure", registry);

   private final AutomaticManipulationAbortCommunicator automaticManipulationAbortCommunicator;
   private final BooleanYoVariable isAutomaticManipulationAbortEnabled = new BooleanYoVariable("isAutomaticManipulationAbortEnabled", registry);
   private final BooleanYoVariable hasManipulationBeenAborted = new BooleanYoVariable("hasManipulationBeenAborted", registry);
   private final DoubleYoVariable icpErrorThresholdToAbortManipulation = new DoubleYoVariable("icpErrorThresholdToAbortManipulation", registry);
   private final DoubleYoVariable minimumDurationBetweenTwoManipulationAborts = new DoubleYoVariable("minimumDurationBetweenTwoManipulationAborts", registry);
   private final DoubleYoVariable timeOfLastManipulationAbortRequest = new DoubleYoVariable("timeOfLastManipulationAbortRequest", registry);
   private final DoubleYoVariable manipulationIgnoreInputsDurationAfterAbort = new DoubleYoVariable("manipulationIgnoreInputsDurationAfterAbort", registry);

   public WalkingHighLevelHumanoidController(VariousWalkingProviders variousWalkingProviders, VariousWalkingManagers variousWalkingManagers,
         CoMHeightTrajectoryGenerator centerOfMassHeightTrajectoryGenerator, TransferTimeCalculationProvider transferTimeCalculationProvider,
         SwingTimeCalculationProvider swingTimeCalculationProvider, WalkingControllerParameters walkingControllerParameters,
         CapturePointPlannerAdapter capturePointPlannerAdapter, ICPAndMomentumBasedController icpAndMomentumBasedController,
         MomentumBasedController momentumBasedController)
   {
      super(variousWalkingProviders, variousWalkingManagers, momentumBasedController, walkingControllerParameters, controllerState);

      setupManagers(variousWalkingManagers);

      hasWalkingControllerBeenInitialized.set(false);

      unconstrainedDesiredPositions = new DoubleYoVariable[unconstrainedJoints.length];

      for (int i = 0; i < unconstrainedDesiredPositions.length; i++)
      {
         unconstrainedDesiredPositions[i] = new DoubleYoVariable(unconstrainedJoints[i].getName() + "_unconstrained_q_d", registry);
      }

      timeToGetPreparedForLocomotion.set(walkingControllerParameters.getTimeToGetPreparedForLocomotion());
      icpErrorThresholdToSpeedUpSwing.set(walkingControllerParameters.getICPErrorThresholdToSpeedUpSwing());

      doPrepareManipulationForLocomotion.set(walkingControllerParameters.doPrepareManipulationForLocomotion());
      doPreparePelvisForLocomotion.set(true);

      automaticManipulationAbortCommunicator = variousWalkingProviders.getAutomaticManipulationAbortCommunicator();
      isAutomaticManipulationAbortEnabled.set(walkingControllerParameters.allowAutomaticManipulationAbort());
      icpErrorThresholdToAbortManipulation.set(0.04);
      minimumDurationBetweenTwoManipulationAborts.set(5.0);
      manipulationIgnoreInputsDurationAfterAbort.set(2.0);

      failureDetectionControlModule = new WalkingFailureDetectionControlModule(momentumBasedController.getContactableFeet(), registry);

      if (yoGraphicsListRegistry == null)
      {
         VISUALIZE = false;
      }

      if (VISUALIZE)
      {
         transferToAndNextFootstepsDataVisualizer = new TransferToAndNextFootstepsDataVisualizer(registry, yoGraphicsListRegistry);
      }
      else
      {
         transferToAndNextFootstepsDataVisualizer = null;
      }

      if (VISUALIZE)
      {
         YoGraphicPosition dynamicGraphicPositionECMP = new YoGraphicPosition("ecmpviz", ecmpViz, 0.002, YoAppearance.BlueViolet());
         yoGraphicsListRegistry.registerYoGraphic("ecmpviz", dynamicGraphicPositionECMP);
         yoGraphicsListRegistry.registerArtifact("ecmpviz", dynamicGraphicPositionECMP.createArtifact());
      }

      // Getting parameters from the icpAndMomentumBasedController
      this.icpAndMomentumBasedController = icpAndMomentumBasedController;

      //    contactStates = momentumBasedController.getContactStates();
      upcomingSupportLeg = momentumBasedController.getUpcomingSupportLeg();
      supportLeg = icpAndMomentumBasedController.getYoSupportLeg();
      capturePoint = icpAndMomentumBasedController.getCapturePoint();
      desiredICP = icpAndMomentumBasedController.getDesiredICP();
      desiredICPVelocity = icpAndMomentumBasedController.getDesiredICPVelocity();
      bipedSupportPolygons = icpAndMomentumBasedController.getBipedSupportPolygons();
      controlledCoMHeightAcceleration = icpAndMomentumBasedController.getControlledCoMHeightAcceleration();
      centerOfMassJacobian = momentumBasedController.getCenterOfMassJacobian();

      // this.finalDesiredICPCalculator = finalDesiredICPCalculator;
      this.centerOfMassHeightTrajectoryGenerator = centerOfMassHeightTrajectoryGenerator;

      this.swingTimeCalculationProvider = swingTimeCalculationProvider;
      this.transferTimeCalculationProvider = transferTimeCalculationProvider;

      this.footSwitches = momentumBasedController.getFootSwitches();

      this.capturePointPlannerAdapter = capturePointPlannerAdapter;

      this.timeICPPlannerFinishedAt = new DoubleYoVariable("timeICPPlannerFinishedAt", registry);
      this.desiredICPVelocityReductionDuration = new DoubleYoVariable("desiredICPVelocityReductionDuration", registry);
      desiredICPVelocityReductionDuration.set(walkingControllerParameters.getDurationToCancelOutDesiredICPVelocityWhenStuckInTransfer());
      this.desiredICPVelocityRedutionFactor = new DoubleYoVariable("desiredICPVelocityRedutionFactor", registry);

      // TODO The following is the hack N13, switch to the new planner instead of using that hack.
      useICPPlannerHackN13.set(walkingControllerParameters.useICPPlannerHackN13());
      moveICPAwayDuringSwingDistance.set(0.012); // 0.03);
      moveICPAwayAtEndOfSwingDistance.set(0.04); // 0.08);
      singleSupportTimeLeftBeforeShift.set(0.26);

      FootstepProvider footstepProvider = variousWalkingProviders.getFootstepProvider();
      this.upcomingFootstepList = new UpcomingFootstepList(footstepProvider, registry);
      footPoseProvider = variousWalkingProviders.getDesiredFootPoseProvider();
      desiredFootStateProvider = variousWalkingProviders.getDesiredFootStateProvider();

      YoPDGains comHeightControlGains = walkingControllerParameters.createCoMHeightControlGains(registry);
      DoubleYoVariable kpCoMHeight = comHeightControlGains.getYoKp();
      DoubleYoVariable kdCoMHeight = comHeightControlGains.getYoKd();
      DoubleYoVariable maxCoMHeightAcceleration = comHeightControlGains.getYoMaximumAcceleration();
      DoubleYoVariable maxCoMHeightJerk = comHeightControlGains.getYoMaximumJerk();

      // TODO Need to extract the maximum velocity parameter.
      coMHeightTimeDerivativesSmoother = new CoMHeightTimeDerivativesSmoother(null, maxCoMHeightAcceleration, maxCoMHeightJerk, controlDT, registry);
      this.centerOfMassHeightController = new PDController(kpCoMHeight, kdCoMHeight, "comHeight", registry);

      String namePrefix = "walking";

      this.stateMachine = new StateMachine<WalkingState>(namePrefix + "State", namePrefix + "SwitchTime", WalkingState.class, yoTime, registry); // this is used by name, and it is ugly.

      this.icpTrajectoryHasBeenInitialized = new BooleanYoVariable("icpTrajectoryHasBeenInitialized", registry);

      rememberFinalICPFromSingleSupport.set(false); // true);
      finalDesiredICPInWorld.set(Double.NaN, Double.NaN);

      coefficientOfFriction.set(0.0); // TODO Remove coefficient of friction from the abstract high level stuff and let the EndEffector controlModule deal with it

      this.centerOfMassHeightTrajectoryGenerator.attachWalkOnToesManager(feetManager.getWalkOnTheEdgesManager());

      pushRecoveryModule = new PushRecoveryControlModule(bipedSupportPolygons, momentumBasedController, walkingControllerParameters, registry);

      footExplorationControlModule = new FootExplorationControlModule(registry, momentumBasedController, yoTime, centerOfMassHeightTrajectoryGenerator,
            swingTimeCalculationProvider, feetManager);

      setupStateMachine();
      readyToGrabNextFootstep.set(true);

      dwellInSingleSupportDuration.set(0.0); //0.2);

      loadFoot.set(false);
      loadFootDuration.set(1.2);
      loadFootTransferDuration.set(0.8);
 
      maxICPErrorBeforeSingleSupportX.set(walkingControllerParameters.getMaxICPErrorBeforeSingleSupportX());
      maxICPErrorBeforeSingleSupportY.set(walkingControllerParameters.getMaxICPErrorBeforeSingleSupportY());

      transferTimeCalculationProvider.updateTransferTime();

      stopInDoubleSupporTrajectoryTime.set(0.5);

      minimumSwingFraction.set(0.5); // 0.8);

      upcomingSupportLeg.set(RobotSide.RIGHT); // TODO: stairs hack, so that the following lines use the correct leading leg

      // TODO: Fix low level stuff so that we are truly controlling pelvis height and not CoM height.
      controlPelvisHeightInsteadOfCoMHeight.set(true);

      if (DESIREDICP_FROM_POLYGON_COORDINATE)
      {
         doubleSupportDesiredICP = new YoFramePoint2dInPolygonCoordinate("desiredICP", registry);
      }
      else
      {
         doubleSupportDesiredICP = null;
      }

      resetIntegratorsAfterSwing.set(true);
      alwaysIntegrateAnkleAcceleration.set(true);

      distanceToShrinkSupportPolygonWhenHoldingCurrent.set(0.08);
   }

   private void setupStateMachine()
   {
      DoubleSupportState doubleSupportState = new DoubleSupportState(null);
      SideDependentList<State<WalkingState>> transferStates = new SideDependentList<>();
      SideDependentList<StateTransition<WalkingState>> fromFallingToSingleSupportsTransitions = new SideDependentList<>();

      stateMachine.addState(doubleSupportState);

      ResetICPTrajectoryAction resetICPTrajectoryAction = new ResetICPTrajectoryAction();
      for (RobotSide robotSide : RobotSide.values)
      {
         WalkingState doubleSupportStateEnum = doubleSupportState.getStateEnum();
         WalkingState singleSupportStateEnum = WalkingState.getSingleSupportState(robotSide);
         WalkingState transferStateEnum = WalkingState.getTransferState(robotSide);
         WalkingState oppTransferStateEnum = WalkingState.getTransferState(robotSide.getOppositeSide());

         State<WalkingState> transferState = new DoubleSupportState(robotSide);
         State<WalkingState> singleSupportState = new SingleSupportState(robotSide);

         transferStates.put(robotSide, transferState);

         StopWalkingCondition stopWalkingCondition = new StopWalkingCondition(robotSide, variousWalkingProviders.getAbortProvider());
         DoneWithTransferCondition doneWithTransferCondition = new DoneWithTransferCondition(robotSide);
         SingleSupportToTransferToCondition singleSupportToTransferToOppositeSideCondition = new SingleSupportToTransferToCondition(robotSide);
         SingleSupportToTransferToCondition singleSupportToTransferToSameSideCondition = new SingleSupportToTransferToCondition(robotSide.getOppositeSide());
         StartWalkingCondition startWalkingCondition = new StartWalkingCondition(robotSide);
         FlamingoStanceCondition flamingoStanceCondition = new FlamingoStanceCondition(robotSide);

         StateTransition<WalkingState> toDoubleSupport = new StateTransition<WalkingState>(doubleSupportStateEnum, stopWalkingCondition,
               resetICPTrajectoryAction);
         StateTransition<WalkingState> toSingleSupport = new StateTransition<WalkingState>(singleSupportStateEnum, doneWithTransferCondition);
         //         StateTransition<WalkingState> toDoubleSupport2 = new StateTransition<WalkingState>(doubleSupportStateEnum, stopWalkingCondition, resetICPTrajectoryAction);
         StateTransition<WalkingState> toTransferOppositeSide = new StateTransition<WalkingState>(oppTransferStateEnum,
               singleSupportToTransferToOppositeSideCondition);
         StateTransition<WalkingState> toTransferSameSide = new StateTransition<WalkingState>(transferStateEnum, singleSupportToTransferToSameSideCondition);
         StateTransition<WalkingState> toTransfer = new StateTransition<WalkingState>(transferStateEnum, startWalkingCondition);
         StateTransition<WalkingState> toTransfer2 = new StateTransition<WalkingState>(transferStateEnum, flamingoStanceCondition);

         transferState.addStateTransition(toDoubleSupport);
         transferState.addStateTransition(toSingleSupport);
         //         singleSupportState.addStateTransition(toDoubleSupport2);
         singleSupportState.addStateTransition(toTransferOppositeSide);
         singleSupportState.addStateTransition(toTransferSameSide);
         doubleSupportState.addStateTransition(toTransfer);
         doubleSupportState.addStateTransition(toTransfer2);

         stateMachine.addState(transferState);
         stateMachine.addState(singleSupportState);

         DoubleSupportToSingleSupportConditionForDisturbanceRecovery isFallingFromDoubleSupportCondition = new DoubleSupportToSingleSupportConditionForDisturbanceRecovery(robotSide);
         StateTransition<WalkingState> fromFallingToSingleSupport = new StateTransition<WalkingState>(singleSupportStateEnum, isFallingFromDoubleSupportCondition);
         fromFallingToSingleSupportsTransitions.put(robotSide, fromFallingToSingleSupport);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         doubleSupportState.addStateTransition(fromFallingToSingleSupportsTransitions.get(robotSide));

         for (RobotSide transferSide : RobotSide.values)
         {
            transferStates.get(transferSide).addStateTransition(fromFallingToSingleSupportsTransitions.get(robotSide));
         }
      }

      stateMachine.attachStateChangedListener(new StateChangedListener<WalkingState>()
      {
         @Override
         public void stateChanged(State<WalkingState> oldState, State<WalkingState> newState, double time)
         {
            momentumBasedController.reportControllerStateChangeToListeners(oldState.getStateEnum(), newState.getStateEnum());
         }
      });
   }

   private RigidBody baseForChestOrientationControl;
   private int jacobianForChestOrientationControlId;

   public void setupManagers(VariousWalkingManagers variousWalkingManagers)
   {
      baseForChestOrientationControl = fullRobotModel.getElevator();
      ChestOrientationManager chestOrientationManager = variousWalkingManagers.getChestOrientationManager();
      String[] chestOrientationControlJointNames = walkingControllerParameters.getDefaultChestOrientationControlJointNames();

      if (chestOrientationManager != null)
      {
         jacobianForChestOrientationControlId = chestOrientationManager.createJacobian(fullRobotModel, chestOrientationControlJointNames);
         chestOrientationManager.setUp(baseForChestOrientationControl, jacobianForChestOrientationControlId);
      }
   }

   public void initialize()
   {
      super.initialize();

      for (int i = 0; i < unconstrainedDesiredPositions.length; i++)
      {
         unconstrainedDesiredPositions[i].set(unconstrainedJoints[i].getQ());
      }

      momentumBasedController.setMomentumControlModuleToUse(MOMENTUM_CONTROL_MODULE_TO_USE);

      initializeContacts();

      ChestOrientationManager chestOrientationManager = variousWalkingManagers.getChestOrientationManager();
      if (chestOrientationManager != null)
      {
         chestOrientationManager.setUp(baseForChestOrientationControl, jacobianForChestOrientationControlId);
      }

      pelvisICPBasedTranslationManager.disable();

      double stepTime = swingTimeCalculationProvider.getValue() + transferTimeCalculationProvider.getValue();
      pelvisOrientationManager.setTrajectoryTime(stepTime);

      if (!hasWalkingControllerBeenInitialized.getBooleanValue())
      {
         pelvisOrientationManager.resetOrientationOffset();
         pelvisOrientationManager.setToZeroInSupportFoot(upcomingSupportLeg.getEnumValue());
         hasWalkingControllerBeenInitialized.set(true);
      }
      else
      {
         pelvisOrientationManager.resetOrientationOffset();
         pelvisOrientationManager.setToHoldCurrentInWorldFrame();
      }
      
      if(manipulationControlModule != null)
      {
         manipulationControlModule.holdCurrentArmConfiguration();
      }
      chestOrientationManager.holdCurrentOrientation();

      icpAndMomentumBasedController.initialize();
      desiredICP.setByProjectionOntoXYPlane(capturePoint);
//      requestICPPlannerToHoldCurrent(); // Not sure if we want to do this. Might cause robot to fall. Might just be better to recenter ICP whenever switching to walking.

      // Need to reset it so the planner will be initialized even when restarting the walking controller.
      hasICPPlannerBeenInitializedAtStart.set(false);
      stateMachine.setCurrentState(WalkingState.DOUBLE_SUPPORT);

      hasWalkingControllerBeenInitialized.set(true);
   }

   public void initializeDesiredHeightToCurrent()
   {
      centerOfMassHeightTrajectoryGenerator.initializeDesiredHeightToCurrent();
      coMHeightTimeDerivativesSmoother.reset();
      feetManager.resetHeightCorrectionParametersForSingularityAvoidance();
   }

   public void requestICPPlannerToHoldCurrentCoM()
   {
      tmpFramePoint.setToZero(referenceFrames.getCenterOfMassFrame());

      icpAndMomentumBasedController.updateBipedSupportPolygons();
      FrameConvexPolygon2d supportPolygonInMidFeetZUp = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();
      convexPolygonShrinker.shrinkConstantDistanceInto(supportPolygonInMidFeetZUp, distanceToShrinkSupportPolygonWhenHoldingCurrent.getDoubleValue(), shrunkSupportPolygon);

      tmpFramePoint.changeFrame(shrunkSupportPolygon.getReferenceFrame());
      tempFramePoint2d.setByProjectionOntoXYPlaneIncludingFrame(tmpFramePoint);
      shrunkSupportPolygon.orthogonalProjection(tempFramePoint2d);
      tmpFramePoint.setXY(tempFramePoint2d);

      tmpFramePoint.changeFrame(worldFrame);
      capturePointPlannerAdapter.holdCurrentICP(yoTime.getDoubleValue(), tmpFramePoint);
   }

   private void initializeContacts()
   {
      momentumBasedController.clearContacts();

      for (RobotSide robotSide : RobotSide.values)
      {
         feetManager.setFlatFootContactState(robotSide);
      }
   }

   private final EnumYoVariable<RobotSide> trailingLeg = new EnumYoVariable<RobotSide>("trailingLeg", "", registry, RobotSide.class, true);
   private final EnumYoVariable<RobotSide> lastPlantedLeg = new EnumYoVariable<RobotSide>("lastPlantedLeg", "", registry, RobotSide.class, true);

   private class DoubleSupportState extends State<WalkingState>
   {      
      private final RobotSide transferToSide;
      private final FramePoint2d desiredICPLocal = new FramePoint2d();
      private final FrameVector2d desiredICPVelocityLocal = new FrameVector2d();
      private final FramePoint2d ecmpLocal = new FramePoint2d();
      private final FramePoint2d capturePoint2d = new FramePoint2d();
      private final FramePoint2d stanceFootLocation = new FramePoint2d();

      private boolean isICPPlannerDonePreviousValue = false;

      public DoubleSupportState(RobotSide transferToSide)
      {
         super((transferToSide == null) ? WalkingState.DOUBLE_SUPPORT : WalkingState.getTransferState(transferToSide));
         this.transferToSide = transferToSide;
      }

      @Override
      public void doAction()
      {
         //abort walk and clear if should abort
         if (variousWalkingProviders.getAbortProvider().shouldAbortWalking())
         {
            upcomingFootstepList.clearCurrentFootsteps();
            upcomingFootstepList.requestCancelPlanToProvider();
            variousWalkingProviders.getAbortProvider().walkingAborted();
            readyToGrabNextFootstep.set(true);
         }

         if (!alwaysIntegrateAnkleAcceleration.getBooleanValue())
            doNotIntegrateAnkleAccelerations();

         feetManager.updateContactStatesInDoubleSupport(transferToSide);

         // note: this has to be done before the ICP trajectory generator is initialized, since it is using nextFootstep
         // TODO: Make a LOADING state and clean all of these timing hacks up.
         doneFinishingSingleSupportTransfer.set(capturePointPlannerAdapter.isPerformingICPDoubleSupport());
         double estimatedTimeRemainingForState = capturePointPlannerAdapter.getEstimatedTimeRemainingForState(yoTime.getDoubleValue());

         if (doneFinishingSingleSupportTransfer.getBooleanValue() || (estimatedTimeRemainingForState < 0.02))
         {
            if (readyToGrabNextFootstep.getBooleanValue())
            {
               upcomingFootstepList.checkForFootsteps();
               
               if (upcomingFootstepList.getNextFootstep() != null)
               {
                  upcomingSupportLeg.set(upcomingFootstepList.getNextFootstep().getRobotSide().getOppositeSide());
                  readyToGrabNextFootstep.set(false);
               }
            }
            footstepListHasBeenUpdated.set(true);
         }

         initializeICPPlannerIfNecessary();

         desiredICPLocal.setToZero(desiredICP.getReferenceFrame());
         desiredICPVelocityLocal.setToZero(desiredICPVelocity.getReferenceFrame());
         ecmpLocal.setToZero(worldFrame);
         capturePoint.getFrameTuple2dIncludingFrame(capturePoint2d);

         capturePointPlannerAdapter.getICPPositionAndVelocity(desiredICPLocal, desiredICPVelocityLocal, ecmpLocal, capturePoint2d,
               yoTime.getDoubleValue());
         
         if (transferToSide != null)
         {
            stanceFootLocation.setToZero(referenceFrames.getAnkleZUpFrame(transferToSide));
            moveICPToInsideOfFootAtEndOfSwing(transferToSide.getOppositeSide(), stanceFootLocation, swingTimeCalculationProvider.getValue(), 0.0,
                  desiredICPLocal);
         }
         else
         {
            RobotSide previousSupport = previousSupportSide.getEnumValue();
            if (previousSupport != null)
            {
               stanceFootLocation.setToZero(referenceFrames.getAnkleZUpFrame(previousSupport.getOppositeSide()));
               moveICPToInsideOfFootAtEndOfSwing(previousSupport, stanceFootLocation, swingTimeCalculationProvider.getValue(), 0.0, desiredICPLocal);
            }

            pelvisICPBasedTranslationManager.compute(null, capturePoint2d);
            pelvisICPBasedTranslationManager.addICPOffset(desiredICPLocal, desiredICPVelocityLocal, bipedSupportPolygons.getSupportPolygonInMidFeetZUp());
         }

         desiredICP.set(desiredICPLocal);
         desiredICPVelocity.set(desiredICPVelocityLocal);

         desiredECMP.set(ecmpLocal);

         if (footExplorationControlModule.isControllingSwingFoot() && !pushRecoveryModule.isRecovering() && !isInFlamingoStance.getBooleanValue())
         {
            footExplorationControlModule.masterFullExploration(desiredICP, desiredICPVelocity, capturePoint2d, desiredCMP);
         }

         if (VISUALIZE)
         {
            ecmpViz.set(desiredECMP.getX(), desiredECMP.getY(), 0.0);
         }

         initializeECMPbasedToeOffIfNotInitializedYet();

         boolean isICPPlannerDone = capturePointPlannerAdapter.isDone(yoTime.getDoubleValue());
         if (transferToSide != null && isICPPlannerDone)
         {
            if (!isICPPlannerDonePreviousValue)
               timeICPPlannerFinishedAt.set(yoTime.getDoubleValue());

            isICPPlannerDonePreviousValue = isICPPlannerDone;

            if (!timeICPPlannerFinishedAt.isNaN())
            {
               desiredICPVelocityRedutionFactor.set(1.0 - (yoTime.getDoubleValue() - timeICPPlannerFinishedAt.getDoubleValue())
                     / desiredICPVelocityReductionDuration.getDoubleValue());
               desiredICPVelocityRedutionFactor.set(MathTools.clipToMinMax(desiredICPVelocityRedutionFactor.getDoubleValue(), 0.0, 1.0));

               desiredICPVelocity.scale(desiredICPVelocityRedutionFactor.getDoubleValue());
            }
         }

         if (pushRecoveryModule.isEnabled())
         {
            capturePoint.getFrameTuple2dIncludingFrame(capturePoint2d);
            pushRecoveryModule.updateForDoubleSupport(desiredICPLocal, capturePoint2d, icpAndMomentumBasedController.getOmega0());
         }

         // Do it only when standing
         if (transferToSide == null)
            handleAutomaticManipulationAbortOnICPError();
      }

      private void handleAutomaticManipulationAbortOnICPError()
      {
         if (manipulationControlModule == null)
         {
            return;
         }

         if (automaticManipulationAbortCommunicator != null && automaticManipulationAbortCommunicator.checkForNewInformation())
         {
            isAutomaticManipulationAbortEnabled.set(automaticManipulationAbortCommunicator.isAutomaticManipulationAbortRequested());
         }

         if (!isAutomaticManipulationAbortEnabled.getBooleanValue())
         {
            return;
         }

         if (yoTime.getDoubleValue() - timeOfLastManipulationAbortRequest.getDoubleValue() < minimumDurationBetweenTwoManipulationAborts.getDoubleValue())
            return;

         if (capturePoint2d.distance(desiredICPLocal) > icpErrorThresholdToAbortManipulation.getDoubleValue())
         {
            hasManipulationBeenAborted.set(true);
            manipulationControlModule.freeze();
            manipulationControlModule.ignoreInputsForGivenDuration(manipulationIgnoreInputsDurationAfterAbort.getDoubleValue());
            timeOfLastManipulationAbortRequest.set(yoTime.getDoubleValue());

            if (automaticManipulationAbortCommunicator != null)
            {
               automaticManipulationAbortCommunicator.reportManipulationAborted();
            }
         }
         else
         {
            hasManipulationBeenAborted.set(false);
         }
      }

      public void initializeICPPlannerIfNecessary()
      {
         if (!icpTrajectoryHasBeenInitialized.getBooleanValue() && capturePointPlannerAdapter.isDone(yoTime.getDoubleValue()))
         {
            initializingICPTrajectory.set(true);

            ImmutablePair<FramePoint2d, Double> finalDesiredICPAndTrajectoryTime = computeFinalDesiredICPAndTrajectoryTime();

            if (transferToSide == null && !hasICPPlannerBeenInitializedAtStart.getBooleanValue())
            {
               FramePoint2d finalDesiredICP = finalDesiredICPAndTrajectoryTime.getLeft();
               finalDesiredICP.changeFrame(desiredICP.getReferenceFrame());

               desiredICP.set(finalDesiredICP);

               updateICPPlannerTimesAndOmega0();
               capturePointPlannerAdapter.clear();
               Footstep nextNextFootstep = upcomingFootstepList.getNextNextFootstep();
               capturePointPlannerAdapter.addFootstep(nextNextFootstep);
               capturePointPlannerAdapter.addFootstep(upcomingFootstepList.getNextNextNextFootstep());
               RobotSide upcomingTransferToside = null;
               if (nextNextFootstep != null)
                  upcomingTransferToside = nextNextFootstep.getRobotSide().getOppositeSide();
               capturePointPlannerAdapter.initializeDoubleSupport(desiredICP, desiredICPVelocity, yoTime.getDoubleValue(), upcomingTransferToside);

               hasICPPlannerBeenInitializedAtStart.set(true);
            }

            icpAndMomentumBasedController.updateBipedSupportPolygons(); // need to always update biped support polygons after a change to the contact states
            icpTrajectoryHasBeenInitialized.set(true);
         }
         else
         {
            initializingICPTrajectory.set(false);
         }
      }

      private final FramePoint2d desiredCMP = new FramePoint2d();

      public void initializeECMPbasedToeOffIfNotInitializedYet()
      {
         // the only case left for determining the contact state of the trailing foot
         if (!ecmpBasedToeOffHasBeenInitialized.getBooleanValue() && transferToSide != null)
         {
            RobotSide trailingLeg = transferToSide.getOppositeSide();
            icpAndMomentumBasedController.getDesiredCMP(desiredCMP);
            
            double predictedToeOffDuration = capturePointPlannerAdapter.getEstimatedTimeRemainingForState(yoTime.getDoubleValue());
            desiredICP.getFrameTuple2dIncludingFrame(desiredICPLocal);
            feetManager.requestToeOff(trailingLeg, desiredCMP, desiredICPLocal, capturePoint2d, predictedToeOffDuration);
            
            // If trailing leg is doing toe off, then use front leg for reference frames for com height trajectory.
            centerOfMassHeightTrajectoryGenerator.setSupportLeg(trailingLeg.getOppositeSide());


            if (feetManager.doToeOff())
            {
               icpAndMomentumBasedController.updateBipedSupportPolygons(); // need to always update biped support polygons after a change to the contact states
               ecmpBasedToeOffHasBeenInitialized.set(true);
            }
         }
         else
         {
            // Always do this so that when a foot slips or is loaded in the air, the height
            // gets adjusted.
            if (transferToSide != null)
               centerOfMassHeightTrajectoryGenerator.setSupportLeg(transferToSide);
            else
               centerOfMassHeightTrajectoryGenerator.setSupportLeg(lastPlantedLeg.getEnumValue());
         }
      }

      private final FramePoint2d tempFramePoint2d = new FramePoint2d();
      
      private ImmutablePair<FramePoint2d, Double> computeFinalDesiredICPAndTrajectoryTime()
      {
         ImmutablePair<FramePoint2d, Double> finalDesiredICPAndTrajectoryTime;

         if (transferToSide == null)
         {
            FramePoint2d finalDesiredICP = getDoubleSupportFinalDesiredICPForDoubleSupportStance();
            double trajectoryTime = stopInDoubleSupporTrajectoryTime.getDoubleValue();

            finalDesiredICPInWorld.set(Double.NaN, Double.NaN);

            finalDesiredICPAndTrajectoryTime = new ImmutablePair<FramePoint2d, Double>(finalDesiredICP, trajectoryTime);
         }

         else if (rememberFinalICPFromSingleSupport.getBooleanValue() && !finalDesiredICPInWorld.containsNaN())
         {
            FramePoint2d finalDesiredICP = finalDesiredICPInWorld.getFramePoint2dCopy();
            double trajectoryTime = transferTimeCalculationProvider.getValue();

            finalDesiredICPAndTrajectoryTime = new ImmutablePair<FramePoint2d, Double>(finalDesiredICP, trajectoryTime);
         }

         else
         {
            updateICPPlannerTimesAndOmega0();
            capturePointPlannerAdapter.clear();
            if (footPoseProvider != null && footPoseProvider.checkForNewPose() != null)
            {
               capturePointPlannerAdapter.setSingleSupportTime(Double.POSITIVE_INFINITY);
               capturePointPlannerAdapter.addFootstep(createFootstepAtCurrentLocation(transferToSide.getOppositeSide()));
            }
            else
            {
               capturePointPlannerAdapter.addFootstep(upcomingFootstepList.getNextFootstep());
               capturePointPlannerAdapter.addFootstep(upcomingFootstepList.getNextNextFootstep());
            }

            // When going from double support to transfer state.
            capturePointPlannerAdapter.initializeDoubleSupport(desiredICP, desiredICPVelocity, yoTime.getDoubleValue(), transferToSide);

            FramePoint2d finalDesiredICP = capturePointPlannerAdapter.getFinalDesiredICP();
            double trajectoryTime = transferTimeCalculationProvider.getValue();

            tempFramePoint2d.setIncludingFrame(finalDesiredICP);
            tempFramePoint2d.changeFrame(worldFrame);
            finalDesiredICPInWorld.set(tempFramePoint2d);
            finalDesiredICPAndTrajectoryTime = new ImmutablePair<FramePoint2d, Double>(finalDesiredICP, trajectoryTime);
         }

         return finalDesiredICPAndTrajectoryTime;
      }

      private TransferToAndNextFootstepsData createTransferToAndNextFootstepDataForDoubleSupport(RobotSide transferToSide, boolean inInitialize)
      {
         Footstep transferFromFootstep = createFootstepAtCurrentLocation(transferToSide.getOppositeSide());
         Footstep transferToFootstep = createFootstepAtCurrentLocation(transferToSide);

         Footstep nextFootstep, nextNextFootstep;

         if (inInitialize)
         {
            // Haven't popped the footstep off yet...
            nextFootstep = upcomingFootstepList.getNextNextFootstep();
            nextNextFootstep = upcomingFootstepList.getNextNextNextFootstep();
         }
         else
         {
            nextFootstep = upcomingFootstepList.getNextFootstep();
            nextNextFootstep = upcomingFootstepList.getNextNextFootstep();
         }

         failureDetectionControlModule.setNextFootstep(nextFootstep);

         TransferToAndNextFootstepsData transferToAndNextFootstepsData = new TransferToAndNextFootstepsData();
         transferToAndNextFootstepsData.setTransferFromFootstep(transferFromFootstep);
         transferToAndNextFootstepsData.setTransferToFootstep(transferToFootstep);
         transferToAndNextFootstepsData.setTransferToSide(transferToSide);
         transferToAndNextFootstepsData.setNextFootstep(nextFootstep);
         transferToAndNextFootstepsData.setNextNextFootstep(nextNextFootstep);

         if (VISUALIZE)
         {
            transferToAndNextFootstepsDataVisualizer.visualizeFootsteps(transferToAndNextFootstepsData);
         }

         return transferToAndNextFootstepsData;
      }

      @Override
      public void doTransitionIntoAction()
      {
         preparingForLocomotion.set(false);

         pelvisICPBasedTranslationManager.enable();
         capturePointPlannerAdapter.clear();
         isICPPlannerDonePreviousValue = false;
         timeICPPlannerFinishedAt.set(Double.NaN);
         desiredICPVelocityRedutionFactor.set(Double.NaN);

         boolean isInDoubleSupport = supportLeg.getEnumValue() == null;
         if (isInDoubleSupport && !upcomingFootstepList.hasNextFootsteps() && !upcomingFootstepList.isPaused())
         {
            PrintTools.debug(DEBUG, this, "WALKING COMPLETE");
            upcomingFootstepList.notifyWalkingComplete();
         }
         
         desiredECMPinSupportPolygon.set(false);
         ecmpBasedToeOffHasBeenInitialized.set(false);
         trailingLeg.set(transferToSide); // FIXME

         icpTrajectoryHasBeenInitialized.set(false);
         if (DEBUG)
            System.out.println("WalkingHighLevelHumanoidController: enteringDoubleSupportState");
         supportLeg.set(null); // TODO: check if necessary

         feetManager.initializeContactStatesForDoubleSupport(transferToSide);

         if (transferToSide != null)
         {
            feetManager.enableAnkleLimitAvoidanceInSupportState(transferToSide, false);
            feetManager.enableAnkleLimitAvoidanceInSupportState(transferToSide.getOppositeSide(), true);
         }
         else
         {
            failureDetectionControlModule.setNextFootstep(null);
            momentumBasedController.reportChangeOfRobotMotionStatus(RobotMotionStatus.STANDING);
            feetManager.enableAnkleLimitAvoidanceInSupportState(false);

            // Do something smart here when going to DoubleSupport state.
            //            instantaneousCapturePointPlanner.initializeForStoppingInDoubleSupport(yoTime.getDoubleValue());
         }

         icpAndMomentumBasedController.updateBipedSupportPolygons(); // need to always update biped support polygons after a change to the contact states

         if (DESIREDICP_FROM_POLYGON_COORDINATE)
         {
            doubleSupportDesiredICP.updatePointAndPolygon(bipedSupportPolygons.getSupportPolygonInMidFeetZUp(), desiredICP.getFramePoint2dCopy());
         }

         RobotSide transferToSideToUseInFootstepData = transferToSide;
         if (transferToSideToUseInFootstepData == null)
            transferToSideToUseInFootstepData = lastPlantedLeg.getEnumValue();

         if (footPoseProvider.checkForNewPose() == null && !centerOfMassHeightTrajectoryGenerator.hasBeenInitializedWithNextStep())
         {
            boolean inInitialize = true;
            TransferToAndNextFootstepsData transferToAndNextFootstepsDataForDoubleSupport = createTransferToAndNextFootstepDataForDoubleSupport(
                  transferToSideToUseInFootstepData, inInitialize);

            centerOfMassHeightTrajectoryGenerator.initialize(transferToAndNextFootstepsDataForDoubleSupport,
                  transferToAndNextFootstepsDataForDoubleSupport.getTransferToSide(), null, getContactStatesList());
         }

         if (pushRecoveryModule.isEnabled())
         {
            pushRecoveryModule.reset();
         }

         double transferTime;

         boolean isPreviousStateDoubleSupport = getPreviousState().getStateEnum() == WalkingState.DOUBLE_SUPPORT;
         if (isPreviousStateDoubleSupport)
         {
            transferTime = capturePointPlannerAdapter.getInitialTransferDuration();
         }
         else
         {
            transferTime = transferTimeCalculationProvider.getValue();
         }

         pelvisOrientationManager.setTrajectoryTime(transferTime);

         // Just standing in double support, do nothing
         if (transferToSide == null)
            pelvisOrientationManager.setToHoldCurrentDesiredInMidFeetZUpFrame();
         // Transferring to execute a foot pose, hold current desired in upcoming support foot in case it slips
         else if (footPoseProvider.checkForNewPose() != null)
            pelvisOrientationManager.setToHoldCurrentDesiredInSupportFoot(transferToSide);
         // Transfer for taking the first step, need to ensure a safe pelvis orientation
         else if (upcomingFootstepList.hasNextFootsteps() && isPreviousStateDoubleSupport)
            pelvisOrientationManager.moveToAverageInSupportFoot(transferToSide);
         // In middle of walking or leaving foot pose, pelvis is good leave it like that.
         else
            pelvisOrientationManager.setToHoldCurrentDesiredInSupportFoot(transferToSide);

         footExplorationControlModule.setSwingIsFinished(true);
      }

      @Override
      public void doTransitionOutOfAction()
      {
         preparingForLocomotion.set(false);
         pelvisICPBasedTranslationManager.disable();
         footstepListHasBeenUpdated.set(false);
         desiredECMPinSupportPolygon.set(false);
         feetManager.reset();
         ecmpBasedToeOffHasBeenInitialized.set(false);

         if (transferToSide == null)
         {
            momentumBasedController.reportChangeOfRobotMotionStatus(RobotMotionStatus.IN_MOTION);

            if (!pushRecoveryModule.isRecoveringFromDoubleSupportFall())
            {
               swingTimeCalculationProvider.updateSwingTime();
               transferTimeCalculationProvider.updateTransferTime();
            }
         }

         if (DEBUG)
            System.out.println("WalkingHighLevelHumanoidController: leavingDoubleSupportState");

         desiredICPVelocity.set(0.0, 0.0);

         footExplorationControlModule.reset();
      }
   }

   private Footstep previousDesiredFootstep;
   private class SingleSupportState extends State<WalkingState>
   {
      private final RobotSide swingSide;

      private final FramePoint2d desiredICPLocal = new FramePoint2d();
      private final FrameVector2d desiredICPVelocityLocal = new FrameVector2d();
      private final FramePoint2d ecmpLocal = new FramePoint2d();
      private final FramePoint2d capturePoint2d = new FramePoint2d();
      private final FramePoint2d desiredCMP = new FramePoint2d();

      private final FramePoint2d transferToFootstepLocation = new FramePoint2d();
      
      private Footstep nextFootstep;
      private double captureTime;

      private final FramePose actualFootPoseInWorld;

      public SingleSupportState(RobotSide supportSide)
      {
         super(WalkingState.getSingleSupportState(supportSide));
         this.swingSide = supportSide.getOppositeSide();
         actualFootPoseInWorld = new FramePose(worldFrame);
      }

      @Override
      public void doAction()
      {
         integrateAnkleAccelerationsOnSwingLeg(swingSide);

         desiredICPLocal.setToZero(desiredICP.getReferenceFrame());
         desiredICPVelocityLocal.setToZero(desiredICPVelocity.getReferenceFrame());
         ecmpLocal.setToZero(worldFrame);

         capturePoint.getFrameTuple2dIncludingFrame(capturePoint2d);

         capturePointPlannerAdapter.getICPPositionAndVelocity(desiredICPLocal, desiredICPVelocityLocal, ecmpLocal, capturePoint2d, yoTime.getDoubleValue());

         if (isInFlamingoStance.getBooleanValue() && footPoseProvider.checkForNewPose(swingSide))
            feetManager.requestMoveStraight(swingSide, footPoseProvider.getDesiredFootPose(swingSide), footPoseProvider.getTrajectoryTime());

         RobotSide supportSide = swingSide.getOppositeSide();
         transferToFootstep.packFramePoint2d(transferToFootstepLocation);

         boolean icpErrorIsTooLarge = capturePoint2d.distance(desiredICPLocal) > icpErrorThresholdToSpeedUpSwing.getDoubleValue();

         if (isInFlamingoStance.getBooleanValue())
         {
            if (icpErrorIsTooLarge)
            {
               FrameConvexPolygon2d supportPolygonInWorld = bipedSupportPolygons.getSupportPolygonInWorld();
               FrameConvexPolygon2d combinedFootPolygon = failureDetectionControlModule.getCombinedFootPolygon();
               if (!supportPolygonInWorld.isPointInside(capturePoint2d, 2.0e-2) && combinedFootPolygon.isPointInside(capturePoint2d))
               {
                  feetManager.requestMoveStraightTouchdownForDisturbanceRecovery(swingSide);
                  initiateFootLoadingProcedure(swingSide);
                  holdICPToCurrentCoMLocationInNextDoubleSupport.set(true);
               }
            }
         }
         else if (pushRecoveryModule.isEnabled())
         {
            capturePoint.getFrameTuple2dIncludingFrame(capturePoint2d);
            pushRecoveryModule.updateForSingleSupport(desiredICPLocal, capturePoint2d, icpAndMomentumBasedController.getOmega0());

            double estimatedTimeRemaining = capturePointPlannerAdapter.getEstimatedTimeRemainingForState(yoTime.getDoubleValue());
            boolean footstepHasBeenAdjusted = pushRecoveryModule.checkAndUpdateFootstep(estimatedTimeRemaining, nextFootstep);
            if (footstepHasBeenAdjusted)
            {
               failureDetectionControlModule.setNextFootstep(nextFootstep);
               updateFootstepParameters();

               captureTime = stateMachine.timeInCurrentState();
               feetManager.replanSwingTrajectory(swingSide, nextFootstep);

               updateICPPlannerTimesAndOmega0();

               upcomingFootstepList.requestCancelPlanToProvider();
               upcomingFootstepList.clearCurrentFootsteps();

               capturePointPlannerAdapter.clear();
               capturePointPlannerAdapter.addFootstep(nextFootstep);
               capturePoint.getFrameTupleIncludingFrame(tmpFramePoint);

               capturePointPlannerAdapter.updatePlanForSingleSupportDisturbances(tmpFramePoint, yoTime.getDoubleValue(), supportSide);
               // Force the final ICP to be in the planned footstep so the smart CMP projection will be way more effective
               finalDesiredCapturePoint2d.setToZero(nextFootstep.getSoleReferenceFrame());
               finalDesiredICPInWorld.setAndMatchFrame(finalDesiredCapturePoint2d);
               holdICPToCurrentCoMLocationInNextDoubleSupport.set(true);
            }
         }

         if (icpErrorIsTooLarge || pushRecoveryModule.isRecovering())
         {
            requestSwingSpeedUpIfNeeded();
         }

         if (isInFlamingoStance.getBooleanValue())
         {
            pelvisICPBasedTranslationManager.compute(supportSide, capturePoint2d);
            pelvisICPBasedTranslationManager.addICPOffset(desiredICPLocal, desiredICPVelocityLocal, bipedSupportPolygons.getFootPolygonInAnkleZUp(supportSide));
         }
         else
         {
            double swingTimeRemaining = swingTimeCalculationProvider.getValue() - stateMachine.timeInCurrentState();
            moveICPToInsideOfFootAtEndOfSwing(supportSide, transferToFootstepLocation, swingTimeCalculationProvider.getValue(), swingTimeRemaining, desiredICPLocal);
            pelvisICPBasedTranslationManager.clearProvider();
            pelvisOrientationManager.clearProvider();
            footPoseProvider.clear();
         }

         desiredICP.set(desiredICPLocal);
         desiredICPVelocity.set(desiredICPVelocityLocal);

         desiredECMP.set(ecmpLocal);

         if (footExplorationControlModule.isControllingSwingFoot() && !pushRecoveryModule.isRecovering() && !isInFlamingoStance.getBooleanValue())
         {
            icpAndMomentumBasedController.getDesiredCMP(desiredCMP);
            footExplorationControlModule.masterFullExploration(desiredICP, desiredICPVelocity, capturePoint2d, desiredCMP);
         }

         if (VISUALIZE)
         {
            ecmpViz.set(desiredECMP.getX(), desiredECMP.getY(), 0.0);
         }

         if ((stateMachine.timeInCurrentState() - captureTime < 0.5 * swingTimeCalculationProvider.getValue())
               && feetManager.isInSingularityNeighborhood(swingSide))
         {
            feetManager.doSingularityEscape(swingSide);
         }
      }

      private void requestSwingSpeedUpIfNeeded()
      {
         remainingSwingTimeAccordingToPlan.set(capturePointPlannerAdapter.getEstimatedTimeRemainingForState(yoTime.getDoubleValue()));
         tmpFramePoint.set(capturePoint.getX(), capturePoint.getY(), 0.0);
         estimatedRemainingSwingTimeUnderDisturbance.set(capturePointPlannerAdapter.estimateTimeRemainingForStateUnderDisturbance(yoTime.getDoubleValue(), tmpFramePoint));

         if (estimatedRemainingSwingTimeUnderDisturbance.getDoubleValue() > 1.0e-3)
         {
            double swingSpeedUpFactor = remainingSwingTimeAccordingToPlan.getDoubleValue() / estimatedRemainingSwingTimeUnderDisturbance.getDoubleValue();
            feetManager.requestSwingSpeedUp(swingSide, swingSpeedUpFactor);
         }
         else
         {
            feetManager.requestSwingSpeedUp(swingSide, Double.POSITIVE_INFINITY);
         }
      }

      @SuppressWarnings("unused")
      private Footstep createSquareUpFootstep(RobotSide footstepSide)
      {
         Footstep squareUpFootstep = createFootstepAtCurrentLocation(footstepSide);
         FramePose squareUpFootstepPose = new FramePose();
         
         squareUpFootstepPose.setToZero(referenceFrames.getFootFrame(footstepSide.getOppositeSide()));
         squareUpFootstepPose.translate(0.0, footstepSide.negateIfRightSide(walkingControllerParameters.getInPlaceWidth()), 0.0);
         
         
         squareUpFootstepPose.changeFrame(worldFrame);
         squareUpFootstep.setPose(squareUpFootstepPose);
         
         return squareUpFootstep;
      }

      @Override
      public void doTransitionIntoAction()
      {
         lastPlantedLeg.set(swingSide.getOppositeSide());
         captureTime = 0.0;
         hasICPPlannerFinished.set(false);
         trailingLeg.set(null);

         RobotSide supportSide = swingSide.getOppositeSide();
         supportLeg.set(supportSide);
         capturePointPlannerAdapter.clear();

         footSwitches.get(swingSide).reset();
         
         if (pushRecoveryModule.isEnabled() && pushRecoveryModule.isRecoveringFromDoubleSupportFall())
         {
            nextFootstep = pushRecoveryModule.createFootstepForRecoveringFromDisturbance(swingSide, swingTimeCalculationProvider.getValue());
            nextFootstep.setTrajectoryType(TrajectoryType.PUSH_RECOVERY);
            upcomingFootstepList.requestCancelPlanToProvider();
            upcomingFootstepList.clearCurrentFootsteps();
         }
         else
         {
            swingTimeCalculationProvider.updateSwingTime();
            nextFootstep = upcomingFootstepList.getNextFootstep();
         }

         if (footExplorationControlModule.isActive())
         {
            FrameConvexPolygon2d footPolygon = computeFootPolygon(supportSide, referenceFrames.getAnkleZUpFrame(supportSide));
            footExplorationControlModule.initialize(nextFootstep, footPolygon, swingSide);
         }

         if (nextFootstep != null)
         {
            feetManager.requestSwing(swingSide, nextFootstep);
         }
         else if (footPoseProvider != null && footPoseProvider.checkForNewPose(swingSide))
         {
            FramePose nextFootPose = footPoseProvider.getDesiredFootPose(swingSide);
            feetManager.requestMoveStraight(swingSide, nextFootPose, footPoseProvider.getTrajectoryTime());
            isInFlamingoStance.set(true);
            pelvisICPBasedTranslationManager.enable();
         }

         feetManager.enableAnkleLimitAvoidanceInSupportState(false);

         if (DEBUG)
            System.out.println("WalkingHighLevelHumanoidController: enteringSingleSupportState");

         transferTimeCalculationProvider.updateTransferTime();

         if (nextFootstep != null)
         {
            updateFootstepParameters();

            updateICPPlannerTimesAndOmega0();
            capturePointPlannerAdapter.addFootstep(nextFootstep);
            capturePointPlannerAdapter.addFootstep(upcomingFootstepList.getNextNextFootstep());
            capturePointPlannerAdapter.addFootstep(upcomingFootstepList.getNextNextNextFootstep());
            capturePointPlannerAdapter.initializeSingleSupport(yoTime.getDoubleValue(), supportSide);
            finalDesiredICPInWorld.setAndMatchFrame(capturePointPlannerAdapter.getFinalDesiredICP());

            if (pushRecoveryModule.isEnabled() && pushRecoveryModule.isRecoveringFromDoubleSupportFall())
            {
               capturePoint.getFrameTupleIncludingFrame(tmpFramePoint);
               capturePointPlannerAdapter.updatePlanForSingleSupportDisturbances(tmpFramePoint, yoTime.getDoubleValue(), supportSide);
               holdICPToCurrentCoMLocationInNextDoubleSupport.set(true);
               // Force the final ICP to be in the planned footstep so the smart CMP projection will be way more effective
               finalDesiredCapturePoint2d.setToZero(nextFootstep.getSoleReferenceFrame());
               finalDesiredICPInWorld.setAndMatchFrame(finalDesiredCapturePoint2d);
            }
         }
         else
         {
            pelvisOrientationManager.setToHoldCurrentDesiredInSupportFoot(supportSide);
            centerOfMassHeightTrajectoryGenerator.setSupportLeg(supportSide);
         }

         if (desiredFootStateProvider != null)
         {
            desiredFootStateProvider.checkForNewLoadBearingRequest(swingSide);
            loadFoot.set(false);
         }
      }

      private void updateFootstepParameters()
      {
         FramePoint2d nextFootstepPosition = new FramePoint2d();
         nextFootstep.getPosition2d(nextFootstepPosition);
         transferToFootstep.set(nextFootstepPosition);
         RobotSide supportSide = swingSide.getOppositeSide();

         pelvisOrientationManager.setTrajectoryTime(swingTimeCalculationProvider.getValue());
         pelvisOrientationManager.setWithUpcomingFootstep(nextFootstep);

         FramePoint centerOfMass = new FramePoint(referenceFrames.getCenterOfMassFrame());
         centerOfMass.changeFrame(worldFrame);
         //ContactablePlaneBody supportFoot = feet.get(supportSide);
         //Transform3D supportFootToWorldTransform = footToWorldTransform.get(supportSide);
         //double footHeight = DesiredFootstepCalculatorTools.computeMinZPointInFrame(supportFootToWorldTransform, supportFoot, worldFrame).getZ();
         //double comHeight = centerOfMass.getZ() - footHeight;

         TransferToAndNextFootstepsData transferToAndNextFootstepsData = createTransferToAndNextFootstepDataForSingleSupport(nextFootstep, swingSide);
         transferToAndNextFootstepsData.setTransferFromDesiredFootstep(previousDesiredFootstep);
         //FramePoint2d finalDesiredICP = getSingleSupportFinalDesiredICPForWalking(transferToAndNextFootstepsData, swingSide);

         supportLeg.set(supportSide);

         // Update the contact states based on the footstep. If the footstep doesn't have any predicted contact points, then use the default ones in the ContactablePlaneBodys.
         momentumBasedController.updateContactPointsForUpcomingFootstep(nextFootstep);

         icpAndMomentumBasedController.updateBipedSupportPolygons();

         // Shouldn't have to do this init anymore since it's done above...
         // icpTrajectoryGenerator.initialize(desiredICP.getFramePoint2dCopy(), finalDesiredICP, swingTimeCalculationProvider.getValue(), omega0,
         // amountToBeInsideSingleSupport.getDoubleValue(), getSupportLeg(), yoTime.getDoubleValue());

         centerOfMassHeightTrajectoryGenerator.initialize(transferToAndNextFootstepsData, supportLeg.getEnumValue(), nextFootstep, getContactStatesList());

         if (DEBUG)
            System.out.println("WalkingHighLevelHumanoidController: nextFootstep will change now!");
         readyToGrabNextFootstep.set(true);
      }

      @Override
      public void doTransitionOutOfAction()
      {
         finalDesiredICPInWorld.setToNaN();

         if (DEBUG)
            System.out.println("WalkingHighLevelController: leavingDoubleSupportState");

         if (!isInFlamingoStance.getBooleanValue())
         {
            actualFootPoseInWorld.setToZero(fullRobotModel.getEndEffectorFrame(swingSide, LimbName.LEG)); // changed Here Nicolas
            actualFootPoseInWorld.changeFrame(worldFrame);
            upcomingFootstepList.notifyComplete(actualFootPoseInWorld);
         }
         else
         {
            pelvisICPBasedTranslationManager.disable();
            isInFlamingoStance.set(false);
         }

         if (pushRecoveryModule.isEnabled())
         {
            captureTime = 0.0;
            pushRecoveryModule.reset();
         }

         previousSupportSide.set(swingSide.getOppositeSide());
         resetLoadedLegIntegrators(swingSide);
         
         previousDesiredFootstep = nextFootstep;
      }
   }

   private void initiateFootLoadingProcedure(RobotSide swingSide)
   {
      loadFoot.set(true);
      loadFootStartTime.set(yoTime.getDoubleValue());
      capturePointPlannerAdapter.setSingleSupportTime(loadFootDuration.getDoubleValue());
      capturePointPlannerAdapter.setDoubleSupportTime(loadFootTransferDuration.getDoubleValue());
      capturePointPlannerAdapter.clear();
      capturePointPlannerAdapter.addFootstep(createFootstepAtCurrentLocation(swingSide));
      capturePointPlannerAdapter.initializeSingleSupport(yoTime.getDoubleValue(), supportLeg.getEnumValue());
      
      pelvisICPBasedTranslationManager.freeze();
   }

   private class DoubleSupportToSingleSupportConditionForDisturbanceRecovery implements StateTransitionCondition
   {
      private final RobotSide transferToSide;

      public DoubleSupportToSingleSupportConditionForDisturbanceRecovery(RobotSide robotSide)
      {
         this.transferToSide = robotSide;
      }

      @Override
      public boolean checkCondition()
      {
         if (!pushRecoveryModule.isEnabled())
            return false;

         RobotSide suggestedSwingSide = pushRecoveryModule.isRobotFallingFromDoubleSupport(stateMachine.timeInCurrentState());
         boolean isRobotFalling = suggestedSwingSide != null;

         if (!isRobotFalling)
            return false;

         boolean switchToSingleSupport = transferToSide != suggestedSwingSide;

         if (switchToSingleSupport)
            pushRecoveryModule.initializeParametersForDoubleSupportPushRecovery();

         return switchToSingleSupport;
      }
   }

   public class StartWalkingCondition implements StateTransitionCondition
   {
      private final RobotSide transferToSide;

      public StartWalkingCondition(RobotSide robotSide)
      {
         this.transferToSide = robotSide;
      }

      public boolean checkCondition()
      {
         if (readyToGrabNextFootstep.getBooleanValue())
            return false;

         boolean doubleSupportTimeHasPassed = stateMachine.timeInCurrentState() > transferTimeCalculationProvider.getValue();
         boolean transferringToThisRobotSide = transferToSide == upcomingSupportLeg.getEnumValue();

         boolean shouldStartWalking = transferringToThisRobotSide && doubleSupportTimeHasPassed;
         boolean isPreparedToWalk = false;

         if (shouldStartWalking)
         {
            if (!preparingForLocomotion.getBooleanValue())
            {
               preparingForLocomotion.set(true);
               preparingForLocomotionStartTime.set(yoTime.getDoubleValue());

               if (manipulationControlModule != null && doPrepareManipulationForLocomotion.getBooleanValue())
                  manipulationControlModule.prepareForLocomotion();

               if (pelvisOrientationManager != null && doPreparePelvisForLocomotion.getBooleanValue())
                  pelvisOrientationManager.prepareForLocomotion();

               RobotSide upcomingSwingSide = upcomingSupportLeg.getEnumValue().getOppositeSide();
               feetManager.enableAnkleLimitAvoidanceInSupportState(upcomingSwingSide, true);
            }

            isPreparedToWalk = yoTime.getDoubleValue() - preparingForLocomotionStartTime.getDoubleValue() > timeToGetPreparedForLocomotion.getDoubleValue();
         }

         return shouldStartWalking && isPreparedToWalk;
      }
   }

   public class FlamingoStanceCondition implements StateTransitionCondition
   {
      private final RobotSide transferToSide;

      public FlamingoStanceCondition(RobotSide robotSide)
      {
         this.transferToSide = robotSide;
      }

      public boolean checkCondition()
      {
         if (!readyToGrabNextFootstep.getBooleanValue())
            return false;

         boolean doubleSupportTimeHasPassed = stateMachine.timeInCurrentState() > transferTimeCalculationProvider.getValue();
         boolean transferringToThisRobotSide = footPoseProvider != null && footPoseProvider.checkForNewPose(transferToSide.getOppositeSide());

         if (transferringToThisRobotSide && doubleSupportTimeHasPassed)
            upcomingSupportLeg.set(transferToSide);

         return transferringToThisRobotSide && doubleSupportTimeHasPassed;
      }
   }

   public class DoneWithTransferCondition implements StateTransitionCondition
   {
      private final RobotSide robotSide;
      private final FramePoint2d capturePoint2d = new FramePoint2d();
      private final FramePoint2d desiredICP2d = new FramePoint2d();
      private final FramePoint2d icpError2d = new FramePoint2d();

      public DoneWithTransferCondition(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      public boolean checkCondition()
      {
         if (footExplorationControlModule.isControllingSwingFoot() && !stayInTransferWalkingState.getBooleanValue())
            return footExplorationControlModule.isDone();

         if (stayInTransferWalkingState.getBooleanValue())
            return false;

         if (!feetManager.isEdgeTouchDownDone(robotSide))
            return false;

         boolean icpTrajectoryIsDone = icpTrajectoryHasBeenInitialized.getBooleanValue() && capturePointPlannerAdapter.isDone(yoTime.getDoubleValue());

         if (!icpTrajectoryIsDone)
            return false;

         if (CHECK_FOR_OVEREXTENSION_ON_TOE_OFF_LEG)
         {
            if (feetManager.isLegDoingToeOffAndAtLimit(robotSide.getOppositeSide()))
               return true;
         }

         capturePoint.getFrameTuple2dIncludingFrame(capturePoint2d);
         desiredICP.getFrameTuple2dIncludingFrame(desiredICP2d);

         capturePoint2d.changeFrame(referenceFrames.getAnkleZUpFrame(robotSide));
         desiredICP2d.changeFrame(referenceFrames.getAnkleZUpFrame(robotSide));

         icpError2d.setIncludingFrame(desiredICP2d);
         icpError2d.sub(capturePoint2d);

         double ellipticErrorSquared = MathTools.square(icpError2d.getX() / maxICPErrorBeforeSingleSupportX.getDoubleValue())
               + MathTools.square(icpError2d.getY() / maxICPErrorBeforeSingleSupportY.getDoubleValue());
         boolean closeEnough = ellipticErrorSquared < 1.0;

         return closeEnough;
      }
   }

   private class SingleSupportToTransferToCondition extends DoneWithSingleSupportCondition
   {
      private final RobotSide robotSide;

      public SingleSupportToTransferToCondition(RobotSide robotSide)
      {
         super();

         this.robotSide = robotSide;
      }

      public boolean checkCondition()
      {
         Footstep nextFootstep = upcomingFootstepList.getNextNextFootstep();
         if (nextFootstep == null)
            return super.checkCondition();

         if (this.robotSide != nextFootstep.getRobotSide())
            return false;

         boolean condition = super.checkCondition();

         return condition;
      }
   }

   private class DoneWithSingleSupportCondition implements StateTransitionCondition
   {
      private boolean footSwitchActivated;

      public DoneWithSingleSupportCondition()
      {
      }

      public boolean checkCondition()
      {
         RobotSide swingSide = supportLeg.getEnumValue().getOppositeSide();
         hasMinimumTimePassed.set(hasMinimumTimePassed());

         if (!hasICPPlannerFinished.getBooleanValue())
         {
            hasICPPlannerFinished.set(capturePointPlannerAdapter.isDone(yoTime.getDoubleValue()));

            if (hasICPPlannerFinished.getBooleanValue())
            {
               timeThatICPPlannerFinished.set(yoTime.getDoubleValue());
            }
         }

         FootSwitchInterface footSwitch = footSwitches.get(swingSide);

         if (feetManager.willLandOnToes())
         {
            if (!(footSwitch instanceof ToeSwitch))
            {
               throw new RuntimeException("toe touchdown should not be used if Robot is not using a ToeSwitch.");
            }

            ToeSwitch toeSwitch = (ToeSwitch) footSwitch;
            footSwitchActivated = toeSwitch.hasToeHitGround();
         }
         else if (feetManager.willLandOnHeel())
         {
            if (!(footSwitch instanceof HeelSwitch))
            {
               throw new RuntimeException("landOnHeels should not be set to true if Robot is not using a HeelSwitch.");
            }

            HeelSwitch heelSwitch = (HeelSwitch) footSwitch;
            footSwitchActivated = heelSwitch.hasHeelHitGround();
         }
         else
         {
            footSwitchActivated = footSwitch.hasFootHitGround();
         }

         if (hasMinimumTimePassed.getBooleanValue() && justFall.getBooleanValue() && !footExplorationControlModule.isControllingSwingFoot())
            return true;

         boolean finishSingleSupportWhenICPPlannerIsDone = walkingControllerParameters.finishSingleSupportWhenICPPlannerIsDone();// || pushRecoveryModule.isRecovering();

         if (finishSingleSupportWhenICPPlannerIsDone && !isInFlamingoStance.getBooleanValue())
         {
            if (!footExplorationControlModule.isControllingSwingFoot())
            {
               if (hasICPPlannerFinished.getBooleanValue()
                     && (yoTime.getDoubleValue() > timeThatICPPlannerFinished.getDoubleValue() + dwellInSingleSupportDuration.getDoubleValue()))
                  return true;
            }
         }
         
         if (desiredFootStateProvider != null)
         {
            if (isInFlamingoStance.getBooleanValue() && desiredFootStateProvider.checkForNewLoadBearingRequest(swingSide))
            {
               initiateFootLoadingProcedure(swingSide);
            }
            
            if (loadFoot.getBooleanValue() && (yoTime.getDoubleValue() > loadFootStartTime.getDoubleValue() + loadFootDuration.getDoubleValue()))
            {
               loadFoot.set(false);
               return true;
            }
         }

         return hasMinimumTimePassed.getBooleanValue() && footSwitchActivated;
      }

      private boolean hasMinimumTimePassed()
      {
         double minimumSwingTime;
         if (pushRecoveryModule.isEnabled() && pushRecoveryModule.isRecoveringFromDoubleSupportFall())
            minimumSwingTime = 0.15;
         else
            minimumSwingTime = swingTimeCalculationProvider.getValue() * minimumSwingFraction.getDoubleValue();

         return stateMachine.timeInCurrentState() > minimumSwingTime;
      }
   }

   private class StopWalkingCondition extends DoneWithSingleSupportCondition
   {
      private final RobotSide robotSide;
      private final AbortWalkingProvider abortState;

      public StopWalkingCondition(RobotSide robotSide, AbortWalkingProvider abortVariable)
      {
         super();

         this.robotSide = robotSide;
         this.abortState = abortVariable;
      }

      public boolean checkCondition()
      {
         if (abortState.shouldAbortWalking()){
            return true;
         }

         boolean isNotExploringFoothold = !footExplorationControlModule.isControllingSwingFoot();
         boolean isNextFootstepNull = upcomingFootstepList.getNextFootstep() == null;
         // This is to fix a bug occuring for instance when doing transfer to right side and receiving a right footstep the walking would do a left footstep instead.
         boolean isNextFootstepForThisSide = true;
         if (footstepListHasBeenUpdated.getBooleanValue())
         {
            isNextFootstepForThisSide = isNextFootstepNull || upcomingFootstepList.getNextFootstep().getRobotSide() != robotSide;
         }
         else
         {
            boolean isNextNextFootstepNull = upcomingFootstepList.getNextNextFootstep() == null;
            isNextFootstepForThisSide = isNextNextFootstepNull || upcomingFootstepList.getNextNextFootstep().getRobotSide() != robotSide;
         }
         boolean isSupportLegNull = supportLeg.getEnumValue() == null;
         boolean noMoreFootstepsForThisSide = upcomingFootstepList.isFootstepProviderEmpty() && isNextFootstepNull || !isNextFootstepForThisSide;
         boolean noMoreFootPoses = footPoseProvider == null || !footPoseProvider.checkForNewPose(robotSide.getOppositeSide());
         boolean readyToStopWalking = noMoreFootstepsForThisSide && noMoreFootPoses && (isSupportLegNull || super.checkCondition()) && isNotExploringFoothold;
         return readyToStopWalking;
      }
   }

   public class ResetICPTrajectoryAction implements StateTransitionAction
   {
      public void doTransitionAction()
      {
         if (holdICPToCurrentCoMLocationInNextDoubleSupport.getBooleanValue())
         {
            requestICPPlannerToHoldCurrentCoM();
            holdICPToCurrentCoMLocationInNextDoubleSupport.set(false);
         }

         capturePointPlannerAdapter.reset(yoTime.getDoubleValue());
      }
   }

   private void updateICPPlannerTimesAndOmega0()
   {
      capturePointPlannerAdapter.setOmega0(icpAndMomentumBasedController.getOmega0());
      capturePointPlannerAdapter.setDoubleSupportTime(transferTimeCalculationProvider.getValue());
      capturePointPlannerAdapter.setSingleSupportTime(swingTimeCalculationProvider.getValue());
   }

   private FramePoint2d getDoubleSupportFinalDesiredICPForDoubleSupportStance()
   {
      FramePoint2d ret = new FramePoint2d(worldFrame);
      double trailingFootToLeadingFootFactor = 0.5; // 0.25;
      for (RobotSide robotSide : RobotSide.values)
      {
         FramePoint2d centroid = new FramePoint2d(ret.getReferenceFrame());
         FrameConvexPolygon2d footPolygon = computeFootPolygon(robotSide, referenceFrames.getAnkleZUpFrame(robotSide));
         footPolygon.getCentroid(centroid);
         centroid.changeFrame(ret.getReferenceFrame());
         if (robotSide == upcomingSupportLeg.getEnumValue())
            centroid.scale(trailingFootToLeadingFootFactor);
         else
            centroid.scale(1.0 - trailingFootToLeadingFootFactor);
         ret.add(centroid);
      }

      return ret;
   }

   public TransferToAndNextFootstepsData createTransferToAndNextFootstepDataForSingleSupport(Footstep transferToFootstep, RobotSide swingSide)
   {
      TransferToAndNextFootstepsData transferToAndNextFootstepsData = new TransferToAndNextFootstepsData();

      Footstep transferFromFootstep = createFootstepAtCurrentLocation(swingSide.getOppositeSide());

      transferToAndNextFootstepsData.setTransferFromFootstep(transferFromFootstep);
      transferToAndNextFootstepsData.setTransferToFootstep(transferToFootstep);

      transferToAndNextFootstepsData.setTransferToSide(swingSide);
      transferToAndNextFootstepsData.setNextFootstep(upcomingFootstepList.getNextNextFootstep());
      transferToAndNextFootstepsData.setNextNextFootstep(upcomingFootstepList.getNextNextNextFootstep());

      if (VISUALIZE)
      {
         transferToAndNextFootstepsDataVisualizer.visualizeFootsteps(transferToAndNextFootstepsData);
      }

      return transferToAndNextFootstepsData;
   }

   public void doMotionControl()
   {
      if (loopControllerForever.getBooleanValue())
      {
         while (true)
         {
            doMotionControlInternal();
         }
      }
      else
      {
         doMotionControlInternal();
      }
   }

   private final FrameVector2d desiredICPVelocityAsFrameVector = new FrameVector2d();

   // FIXME: don't override
   private void doMotionControlInternal()
   {
      failureDetectionControlModule.checkIfRobotIsFalling(capturePoint, desiredICP);
      if (failureDetectionControlModule.isRobotFalling())
      {
         if (enablePushRecoveryOnFailure.getBooleanValue() && !pushRecoveryModule.isEnabled())
         {
            pushRecoveryModule.setIsEnabled(true);
         }
         else if (!pushRecoveryModule.isEnabled() || pushRecoveryModule.isCaptureRegionEmpty())
         {
            momentumBasedController.reportControllerFailureToListeners(failureDetectionControlModule.getFallingDirection());
         }
      }

      if (enablePushRecoveryOnFailure.getBooleanValue())
      {
         if (pushRecoveryModule.isEnabled() && pushRecoveryModule.isRobotBackToSafeState())
            pushRecoveryModule.setIsEnabled(false);
      }

      momentumBasedController.doPrioritaryControl();
      super.callUpdatables();

      icpAndMomentumBasedController.update();

      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
      
      desiredICPVelocity.packFrameVector2d(desiredICPVelocityAsFrameVector);
      controlledCoMHeightAcceleration.set(computeDesiredCoMHeightAcceleration(desiredICPVelocityAsFrameVector));

      doFootControl();
      doArmControl();
      doHeadControl();

      //    doCoMControl(); //TODO: Should we be doing this too?
      doChestControl();
      doCapturePointBasedControl();
      doPelvisControl();
      doUnconstrainedJointControl();

      setTorqueControlJointsToZeroDersiredAcceleration();

      momentumBasedController.doSecondaryControl();

      momentumBasedController.doPassiveKneeControl();
      momentumBasedController.doProportionalControlOnCoP();
      momentumBasedController.smoothDesiredCoPIfNeeded();
   }

   private final FramePoint2d finalDesiredCapturePoint2d = new FramePoint2d();

   private void doCapturePointBasedControl()
   {
      boolean keepCMPInsideSupportPolygon = true;
      if ((manipulationControlModule != null) && (manipulationControlModule.isAtLeastOneHandLoadBearing()))
         keepCMPInsideSupportPolygon = false;

      finalDesiredICPInWorld.getFrameTuple2dIncludingFrame(finalDesiredCapturePoint2d);
      icpAndMomentumBasedController.computeAndSubmitDesiredRateOfChangeOfMomentum(finalDesiredCapturePoint2d, keepCMPInsideSupportPolygon);
   }

   @Override
   protected void doUnconstrainedJointControl()
   {
      for (int i = 0; i < unconstrainedDesiredPositions.length; i++)
      {
         unconstrainedJoints[i].setqDesired(unconstrainedDesiredPositions[i].getDoubleValue());
      }
      super.doUnconstrainedJointControl();
   }

   // Temporary objects to reduce garbage collection.
   private final CoMHeightPartialDerivativesData coMHeightPartialDerivatives = new CoMHeightPartialDerivativesData();
   private final ContactStatesAndUpcomingFootstepData centerOfMassHeightInputData = new ContactStatesAndUpcomingFootstepData();
   private final FramePoint comPosition = new FramePoint();
   private final FrameVector comVelocity = new FrameVector(worldFrame);
   private final FrameVector2d comXYVelocity = new FrameVector2d();
   private final FrameVector2d comXYAcceleration = new FrameVector2d();
   private final CoMHeightTimeDerivativesData comHeightDataBeforeSmoothing = new CoMHeightTimeDerivativesData();
   private final CoMHeightTimeDerivativesData comHeightDataAfterSmoothing = new CoMHeightTimeDerivativesData();
   private final CoMXYTimeDerivativesData comXYTimeDerivatives = new CoMXYTimeDerivativesData();
   private final FramePoint desiredCenterOfMassHeightPoint = new FramePoint(worldFrame);
   private final FramePoint pelvisPosition = new FramePoint();
   private final FramePoint2d comPositionAsFramePoint2d = new FramePoint2d();

   private final Twist currentPelvisTwist = new Twist();

   private double computeDesiredCoMHeightAcceleration(FrameVector2d desiredICPVelocity)
   {
      centerOfMassHeightInputData
            .setCenterOfMassAndPelvisZUpFrames(momentumBasedController.getCenterOfMassFrame(), momentumBasedController.getPelvisZUpFrame());

      List<? extends PlaneContactState> contactStatesList = getContactStatesList();

      centerOfMassHeightInputData.setContactStates(contactStatesList);

      centerOfMassHeightInputData.setSupportLeg(supportLeg.getEnumValue());

      Footstep nextFootstep;
      if (pushRecoveryModule.isEnabled() && pushRecoveryModule.isRecoveringFromDoubleSupportFall())
      {
         nextFootstep = null;
      }
      else
      {
         nextFootstep = upcomingFootstepList.getNextFootstep();
      }

      centerOfMassHeightInputData.setUpcomingFootstep(nextFootstep);

      centerOfMassHeightTrajectoryGenerator.solve(coMHeightPartialDerivatives, centerOfMassHeightInputData);

      comPosition.setToZero(referenceFrames.getCenterOfMassFrame());
      centerOfMassJacobian.packCenterOfMassVelocity(comVelocity);
      comPosition.changeFrame(worldFrame);
      comVelocity.changeFrame(worldFrame);

      double zCurrent = comPosition.getZ();
      double zdCurrent = comVelocity.getZ();

      if (controlPelvisHeightInsteadOfCoMHeight.getBooleanValue())
      {
         pelvisPosition.setToZero(referenceFrames.getPelvisFrame());
         pelvisPosition.changeFrame(worldFrame);
         zCurrent = pelvisPosition.getZ();
         twistCalculator.packTwistOfBody(currentPelvisTwist, fullRobotModel.getPelvis());
         currentPelvisTwist.changeFrame(worldFrame);
         zdCurrent = comVelocity.getZ(); // Just use com velocity for now for damping...
      }

      
      // TODO: use current omega0 instead of previous
      comXYVelocity.setIncludingFrame(comVelocity.getReferenceFrame(), comVelocity.getX(), comVelocity.getY());
      if (desiredICPVelocity.containsNaN())
      {
         System.err.println("Desired ICP velocity contains NaN");
         comXYAcceleration.setToZero(desiredICPVelocity.getReferenceFrame());
      }
      else
      {
         comXYAcceleration.setIncludingFrame(desiredICPVelocity);
      }
      comXYAcceleration.sub(comXYVelocity);
      comXYAcceleration.scale(icpAndMomentumBasedController.getOmega0()); // MathTools.square(omega0.getDoubleValue()) * (com.getX() - copX);

      // FrameVector2d comd2dSquared = new FrameVector2d(comXYVelocity.getReferenceFrame(), comXYVelocity.getX() * comXYVelocity.getX(), comXYVelocity.getY() * comXYVelocity.getY());
      comPosition.getFramePoint2d(comPositionAsFramePoint2d);
      comXYTimeDerivatives.setCoMXYPosition(comPositionAsFramePoint2d);
      comXYTimeDerivatives.setCoMXYVelocity(comXYVelocity);
      comXYTimeDerivatives.setCoMXYAcceleration(comXYAcceleration);

      coMHeightTimeDerivativesCalculator.computeCoMHeightTimeDerivatives(comHeightDataBeforeSmoothing, comXYTimeDerivatives, coMHeightPartialDerivatives);

      comHeightDataBeforeSmoothing.getComHeight(desiredCenterOfMassHeightPoint);
      desiredCoMHeightFromTrajectory.set(desiredCenterOfMassHeightPoint.getZ());
      desiredCoMHeightVelocityFromTrajectory.set(comHeightDataBeforeSmoothing.getComHeightVelocity());
      desiredCoMHeightAccelerationFromTrajectory.set(comHeightDataBeforeSmoothing.getComHeightAcceleration());

      //    correctCoMHeight(desiredICPVelocity, zCurrent, comHeightDataBeforeSmoothing, false, false);

      //    comHeightDataBeforeSmoothing.getComHeight(desiredCenterOfMassHeightPoint);
      //    desiredCoMHeightBeforeSmoothing.set(desiredCenterOfMassHeightPoint.getZ());
      //    desiredCoMHeightVelocityBeforeSmoothing.set(comHeightDataBeforeSmoothing.getComHeightVelocity());
      //    desiredCoMHeightAccelerationBeforeSmoothing.set(comHeightDataBeforeSmoothing.getComHeightAcceleration());

      coMHeightTimeDerivativesSmoother.smooth(comHeightDataAfterSmoothing, comHeightDataBeforeSmoothing);

      comHeightDataAfterSmoothing.getComHeight(desiredCenterOfMassHeightPoint);
      desiredCoMHeightAfterSmoothing.set(desiredCenterOfMassHeightPoint.getZ());
      desiredCoMHeightVelocityAfterSmoothing.set(comHeightDataAfterSmoothing.getComHeightVelocity());
      desiredCoMHeightAccelerationAfterSmoothing.set(comHeightDataAfterSmoothing.getComHeightAcceleration());

      feetManager.correctCoMHeight(trailingLeg.getEnumValue(), desiredICPVelocity, zCurrent, comHeightDataAfterSmoothing);

      comHeightDataAfterSmoothing.getComHeight(desiredCenterOfMassHeightPoint);
      desiredCoMHeightCorrected.set(desiredCenterOfMassHeightPoint.getZ());
      desiredCoMHeightVelocityCorrected.set(comHeightDataAfterSmoothing.getComHeightVelocity());
      desiredCoMHeightAccelerationCorrected.set(comHeightDataAfterSmoothing.getComHeightAcceleration());

      comHeightDataAfterSmoothing.getComHeight(desiredCenterOfMassHeightPoint);

      double zDesired = desiredCenterOfMassHeightPoint.getZ();
      double zdDesired = comHeightDataAfterSmoothing.getComHeightVelocity();
      double zddFeedForward = comHeightDataAfterSmoothing.getComHeightAcceleration();

      double zddDesired = centerOfMassHeightController.compute(zCurrent, zDesired, zdCurrent, zdDesired) + zddFeedForward;

      for (RobotSide robotSide : RobotSide.values)
      {
         if (feetManager.isInFlatSupportState(robotSide) && feetManager.isInSingularityNeighborhood(robotSide))
         {
            // Ignore the desired height acceleration only if EndEffectorControlModule is not taking care of singularity during support
            if (!LegSingularityAndKneeCollapseAvoidanceControlModule.USE_SINGULARITY_AVOIDANCE_SUPPORT)
               zddDesired = 0.0;

            double zTreshold = 0.01;

            if (zDesired >= zCurrent - zTreshold)
            {
               // Can't achieve the desired height, just lock the knee
               feetManager.lockKnee(robotSide);
            }
            else
            {
               // Do the singularity escape before trying to achieve the desired height
               feetManager.doSupportSingularityEscape(robotSide);
            }
         }
      }

      // In a recovering context, accelerating upwards is just gonna make the robot fall faster. We should even consider letting the robot fall a bit.
      if (pushRecoveryModule.isEnabled() && pushRecoveryModule.isRecovering())
         zddDesired = Math.min(0.0, zddDesired);

      double epsilon = 1e-12;
      zddDesired = MathTools.clipToMinMax(zddDesired, -gravity + epsilon, Double.POSITIVE_INFINITY);

      return zddDesired;
   }

   private List<PlaneContactState> getContactStatesList()
   {
      List<PlaneContactState> contactStatesList = new ArrayList<PlaneContactState>();

      for (RobotSide robotSide : RobotSide.values)
      {
         PlaneContactState contactState = momentumBasedController.getContactState(feet.get(robotSide));

         if (contactState.inContact())
            contactStatesList.add(contactState);
      }

      return contactStatesList;
   }

   private final List<FramePoint> tempContactPoints = new ArrayList<FramePoint>();
   private final FrameConvexPolygon2d tempFootPolygon = new FrameConvexPolygon2d(worldFrame);

   // TODO: should probably precompute this somewhere else
   private FrameConvexPolygon2d computeFootPolygon(RobotSide robotSide, ReferenceFrame referenceFrame)
   {
      momentumBasedController.getContactPoints(feet.get(robotSide), tempContactPoints);
      tempFootPolygon.setIncludingFrameByProjectionOntoXYPlaneAndUpdate(referenceFrame, tempContactPoints);

      return tempFootPolygon;
   }

   private Footstep createFootstepAtCurrentLocation(RobotSide robotSide)
   {
      ContactablePlaneBody foot = feet.get(robotSide);
      ReferenceFrame footReferenceFrame = foot.getRigidBody().getParentJoint().getFrameAfterJoint();
      FramePose framePose = new FramePose(footReferenceFrame);
      framePose.changeFrame(worldFrame);

      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("poseReferenceFrame", framePose);

      boolean trustHeight = true;
      Footstep footstep = new Footstep(foot.getRigidBody(), robotSide, foot.getSoleFrame(), poseReferenceFrame, trustHeight);

      momentumBasedController.setFootstepsContactPointsBasedOnFootContactStatePoints(footstep);

      return footstep;
   }

   public void integrateAnkleAccelerationsOnSwingLeg(RobotSide swingSide)
   {
      fullRobotModel.getLegJoint(swingSide, LegJointName.ANKLE_PITCH).setIntegrateDesiredAccelerations(true);
      fullRobotModel.getLegJoint(swingSide, LegJointName.ANKLE_ROLL).setIntegrateDesiredAccelerations(true);
      fullRobotModel.getLegJoint(swingSide.getOppositeSide(), LegJointName.ANKLE_PITCH).setIntegrateDesiredAccelerations(false);
      fullRobotModel.getLegJoint(swingSide.getOppositeSide(), LegJointName.ANKLE_ROLL).setIntegrateDesiredAccelerations(false);
   }

   private void doNotIntegrateAnkleAccelerations()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         fullRobotModel.getLegJoint(robotSide, LegJointName.ANKLE_PITCH).setIntegrateDesiredAccelerations(false);
         fullRobotModel.getLegJoint(robotSide, LegJointName.ANKLE_ROLL).setIntegrateDesiredAccelerations(false);
      }
   }

   private void resetLoadedLegIntegrators(RobotSide robotSide)
   {
      if (resetIntegratorsAfterSwing.getBooleanValue())
      {
         for (LegJointName jointName : fullRobotModel.getRobotSpecificJointNames().getLegJointNames())
            fullRobotModel.getLegJoint(robotSide, jointName).resetDesiredAccelerationIntegrator();
      }
   }

   private final FrameVector2d stanceToOriginalICPVector = new FrameVector2d();

   private final FramePoint2d stanceToSwingPoint = new FramePoint2d();
   private final FrameVector2d stanceToSwingVector = new FrameVector2d();
   private final FramePoint2d stanceContactCentroid2d = new FramePoint2d();
   private final FramePoint stanceContactCentroid = new FramePoint();

   /**
    * Terrible hack that needs to be gone by switching to the new ICP planner and let it do its job.
    */
   @Deprecated
   private void moveICPToInsideOfFootAtEndOfSwing(RobotSide supportSide, FramePoint2d upcomingFootstepLocation, double swingTime, double swingTimeRemaining,
         FramePoint2d desiredICPToMove)
   {
      if (useICPPlannerHackN13.getBooleanValue())
      {
         ReferenceFrame supportAnkleFrame = referenceFrames.getAnkleZUpFrame(supportSide);

         momentumBasedController.getCenterOfFootContactPoints(supportSide, stanceContactCentroid2d);
         if (stanceContactCentroid2d.containsNaN())
         {
            stanceContactCentroid2d.setToZero(momentumBasedController.getContactableFeet().get(supportSide).getSoleFrame());
         }

         stanceContactCentroid.setToZero(stanceContactCentroid2d.getReferenceFrame());
         stanceContactCentroid.setXY(stanceContactCentroid2d);

         stanceContactCentroid.changeFrame(supportAnkleFrame);

         stanceContactCentroid2d.setIncludingFrame(supportAnkleFrame, stanceContactCentroid.getX(), stanceContactCentroid.getY());

         swingTimeRemainingForICPMoveViz.set(swingTimeRemaining);

         desiredICPToMove.changeFrame(supportAnkleFrame);

         stanceToOriginalICPVector.setIncludingFrame(desiredICPToMove);
         stanceToOriginalICPVector.sub(stanceContactCentroid2d);

         stanceToSwingPoint.setIncludingFrame(upcomingFootstepLocation);
         stanceToSwingPoint.changeFrame(supportAnkleFrame);

         stanceToSwingVector.setIncludingFrame(stanceToSwingPoint);
         stanceToSwingVector.sub(stanceContactCentroid2d);

         double stanceToSwingDistance = stanceToSwingVector.length();
         if (stanceToSwingDistance < 0.001)
         {
            desiredICPToMove.changeFrame(desiredICP.getReferenceFrame());

            return;
         }

         stanceToSwingVector.normalize();

         distanceFromLineToOriginalICP.set(stanceToSwingVector.dot(stanceToOriginalICPVector));
         double timeToUseBeforeShift = singleSupportTimeLeftBeforeShift.getDoubleValue();
         if (timeToUseBeforeShift < 0.01)
            timeToUseBeforeShift = 0.01;

         double deltaTime = swingTime - timeToUseBeforeShift;
         double percent;
         if (deltaTime <= 1e-7)
            percent = 1.0;
         else
         {
            percent = (swingTime - swingTimeRemaining) / (deltaTime);
         }

         percent = MathTools.clipToMinMax(percent, 0.0, 1.0);
         percentOfSwingTimeRemainingForICPMove.set(percent);

         double maxDistanceToMove = moveICPAwayDuringSwingDistance.getDoubleValue();
         maxDistanceToMove = MathTools.clipToMinMax(maxDistanceToMove, 0.0, stanceToSwingDistance / 2.0);
         double duringSwingDistance = (percent * maxDistanceToMove);

         if (swingTimeRemaining > timeToUseBeforeShift)
         {
            amountToMoveICPAway.set(duringSwingDistance);
         }
         else
         {
            percent = (1.0 - swingTimeRemaining / timeToUseBeforeShift);
            percent = MathTools.clipToMinMax(percent, 0.0, 1.0);

            maxDistanceToMove = moveICPAwayAtEndOfSwingDistance.getDoubleValue();
            maxDistanceToMove = MathTools.clipToMinMax(maxDistanceToMove, 0.0, stanceToSwingDistance / 2.0);

            maxDistanceToMove -= moveICPAwayDuringSwingDistance.getDoubleValue();
            if (maxDistanceToMove < 0.0)
               maxDistanceToMove = 0.0;

            amountToMoveICPAway.set(duringSwingDistance + (percent * maxDistanceToMove));
         }

         // If already moved a long distance, then no need to move any more.
         if (distanceFromLineToOriginalICP.getDoubleValue() > amountToMoveICPAway.getDoubleValue())
         {
            desiredICPToMove.changeFrame(desiredICP.getReferenceFrame());

            return;
         }

         double additionalDistance = amountToMoveICPAway.getDoubleValue() - distanceFromLineToOriginalICP.getDoubleValue();

         stanceToSwingVector.scale(additionalDistance);
         icpAdjustment.set(stanceToSwingVector.getX(), stanceToSwingVector.getY());

         desiredICPToMove.add(stanceToSwingVector);
         desiredICPToMove.changeFrame(desiredICP.getReferenceFrame());
      }
   }

   public void reinitializePelvisOrientation(boolean reinitialize)
   {
      hasWalkingControllerBeenInitialized.set( !reinitialize );
   }
}
