package us.ihmc.commonWalkingControlModules.capturePoint;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.capturePoint.controller.ICPControllerParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation.SplitFractionCalculatorParametersReadOnly;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.CaptureRegionStepAdjustmentController;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.ErrorBasedStepAdjustmentController;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentController;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisICPBasedTranslationManager;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.*;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMContinuousContinuityCalculator;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlanner;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commonWalkingControlModules.messageHandlers.CenterOfMassTrajectoryHandler;
import us.ihmc.commonWalkingControlModules.messageHandlers.MomentumTrajectoryHandler;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegionsList;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPoint2DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPointcloud3DDefinition;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.*;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.*;

public class BalanceManager implements SCS2YoGraphicHolder
{
   private static final boolean USE_ERROR_BASED_STEP_ADJUSTMENT = true;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final boolean viewCoPHistory = false;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final ICPControlPlane icpControlPlane;
   private final BipedSupportPolygons bipedSupportPolygons;
   private final ICPControlPolygons icpControlPolygons;

   private final MomentumTrajectoryHandler momentumTrajectoryHandler;
   private final PrecomputedICPPlanner precomputedICPPlanner;

   private final LinearMomentumRateControlModuleInput linearMomentumRateControlModuleInput = new LinearMomentumRateControlModuleInput();

   private final PelvisICPBasedTranslationManager pelvisICPBasedTranslationManager;
   private final StepAdjustmentController stepAdjustmentController;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   private final YoFramePoint2D yoDesiredCapturePoint = new YoFramePoint2D("desiredICP", worldFrame, registry);
   private final YoFrameVector2D yoDesiredICPVelocity = new YoFrameVector2D("desiredICPVelocity", worldFrame, registry);
   private final YoFramePoint3D yoDesiredCoMPosition = new YoFramePoint3D("desiredCoMPosition", worldFrame, registry);
   private final YoFrameVector3D yoDesiredCoMVelocity = new YoFrameVector3D("desiredCoMVelocity", worldFrame, registry);
   private final YoFramePoint2D yoFinalDesiredICP = new YoFramePoint2D("finalDesiredICP", worldFrame, registry);
   private final YoFramePoint2D yoFinalDesiredCoP = new YoFramePoint2D("finalDesiredCoP", worldFrame, registry);
   private final YoFramePoint2D yoEquivalentRemainingCoP = new YoFramePoint2D("equivalentRemainingCoP", worldFrame, registry);
   private final YoFramePoint3D yoFinalDesiredCoM = new YoFramePoint3D("finalDesiredCoM", worldFrame, registry);
   private final YoFrameVector3D yoFinalDesiredCoMVelocity = new YoFrameVector3D("finalDesiredCoMVelocity", worldFrame, registry);
   private final YoFrameVector3D yoFinalDesiredCoMAcceleration = new YoFrameVector3D("finalDesiredCoMAcceleration", worldFrame, registry);

   private final TimeAdjustmentCalculator timeAdjustmentCalculator = new TimeAdjustmentCalculator();
   private final ICPControllerParameters.FeedbackAlphaCalculator feedbackAlphaCalculator;

   private final YoDouble swingSpeedUpForStepAdjustment = new YoDouble("swingSpeedUpForStepAdjustment", registry);
   private final YoDouble currentFeedbackAlpha = new YoDouble("currentFeedbackAlpha", registry);
   private final YoDouble finalFeedbackAlpha = new YoDouble("finalFeedbackAlpha", registry);

   /** CoP position according to the ICP planner */
   private final YoFramePoint3D yoPerfectCoP = new YoFramePoint3D("perfectCoP", worldFrame, registry);
   private final YoFrameVector2D yoPerfectCoPVelocity = new YoFrameVector2D("perfectCoPVelocity", worldFrame, registry);
   /** CMP position according to the ICP planner */
   private final YoFramePoint3D yoPerfectCMP = new YoFramePoint3D("perfectCMP", worldFrame, registry);

   private final BagOfBalls perfectCoPTrajectory;
   private final BagOfBalls perfectCMPTrajectory;

   private final YoBoolean useMomentumRecoveryModeForBalance = new YoBoolean("useMomentumRecoveryModeForBalance", registry);

   private final YoDouble yoTime;
   private final YoDouble icpErrorThresholdToAdjustTime = new YoDouble("icpErrorThresholdToAdjustTime", registry);
   private final YoBoolean shouldAdjustTimeFromTrackingError = new YoBoolean("shouldAdjustTimeFromTrackingError", registry);

   private final ReferenceFrame centerOfMassFrame;

   private final FramePoint3D centerOfMassPosition = new FramePoint3D();
   private final FramePoint2D centerOfMassPosition2d = new FramePoint2D();

   private final FramePoint2D capturePoint2d = new FramePoint2D();
   private final FramePoint2D desiredCapturePoint2d = new FramePoint2D();
   private final FramePoint2D desiredCoM2d = new FramePoint2D();
   private final FrameVector2D desiredCapturePointVelocity2d = new FrameVector2D();
   private final FramePoint2D perfectCMP2d = new FramePoint2D();
   private final FramePoint2D perfectCoP2d = new FramePoint2D();

   private final YoBoolean blendICPTrajectories = new YoBoolean("blendICPTrajectories", registry);

   private final FrameVector2D icpError2d = new FrameVector2D();

   private final ConvexPolygonScaler convexPolygonShrinker = new ConvexPolygonScaler();
   private final FrameConvexPolygon2D shrunkSupportPolygon = new FrameConvexPolygon2D();

   private final YoDouble distanceToShrinkSupportPolygonWhenHoldingCurrent = new YoDouble("distanceToShrinkSupportPolygonWhenHoldingCurrent", registry);

   private final YoBoolean holdICPToCurrentCoMLocationInNextDoubleSupport = new YoBoolean("holdICPToCurrentCoMLocationInNextDoubleSupport", registry);

   private final YoDouble normalizedICPError = new YoDouble("normalizedICPError", registry);
   private final DoubleProvider maxICPErrorBeforeSingleSupportForwardX;
   private final DoubleProvider maxICPErrorBeforeSingleSupportBackwardX;
   private final DoubleProvider maxICPErrorBeforeSingleSupportInnerY;
   private final DoubleProvider maxICPErrorBeforeSingleSupportOuterY;

   private final DoubleProvider icpDistanceOutsideSupportForStep = new DoubleParameter("icpDistanceOutsideSupportForStep", registry, 0.03);
   private final DoubleProvider ellipticICPErrorForMomentumRecovery = new DoubleParameter("ellipticICPErrorForMomentumRecovery", registry, 2.0);

   private final BooleanProvider useAngularMomentumOffset = new BooleanParameter("useAngularMomentumOffset", registry, false);
   private final BooleanProvider useAngularMomentumOffsetInStanding = new BooleanParameter("useAngularMomentumOffsetInStanding", registry, true);
   private final YoBoolean computeAngularMomentumOffset = new YoBoolean("computeAngularMomentumOffset", registry);

   /**
    * Duration parameter used to linearly decrease the desired ICP velocity once the current state is
    * done.
    * <p>
    * This reduction in desired ICP velocity is particularly useful to reduce the ICP tracking error
    * when the robot is getting stuck at the end of transfer.
    * </p>
    */
   private final DoubleParameter icpVelocityDecayDurationWhenDone = new DoubleParameter("ICPVelocityDecayDurationWhenDone", registry, Double.NaN);
   /**
    * Output of the linear reduction being applied on the desired ICP velocity when the current state
    * is done.
    * <p>
    * This reduction in desired ICP velocity is particularly useful to reduce the ICP tracking error
    * when the robot is getting stuck at the end of transfer. true*
    * </p>
    */
   private final YoDouble icpVelocityReductionFactor = new YoDouble("ICPVelocityReductionFactor", registry);

   private final CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus();

   private final ExecutionTimer plannerTimer = new ExecutionTimer("icpPlannerTimer", registry);
   private final ExecutionTimer amPlanningTimerTimer = new ExecutionTimer("amPlanningTimerTimer", registry);

   private boolean initializeOnStateChange = false;
   private boolean minimizeAngularMomentumRateZ = false;
   private RobotSide supportSide;
   private final FixedFramePoint2DBasics desiredCMP = new FramePoint2D();
   private final SimpleFootstep currentFootstep = new SimpleFootstep();
   private final SideDependentList<PlaneContactStateCommand> contactStateCommands = new SideDependentList<>(new PlaneContactStateCommand(),
                                                                                                            new PlaneContactStateCommand());
   private final SideDependentList<? extends ReferenceFrame> soleFrames;

   private final List<Footstep> footsteps = new ArrayList<>();
   private final List<FootstepTiming> footstepTimings = new ArrayList<>();
   private final StepConstraintRegionsList currentStepConstraints = new StepConstraintRegionsList();

   private final ContactStateManager contactStateManager;
   private final DoubleProvider timeShiftProvider = this::computeTimeShiftToMinimizeTrackingError;

   private final Predicate<RobotSide> contactStateBasedPredicate;
   private final CoPTrajectoryGeneratorState copTrajectoryState;
   private final WalkingCoPTrajectoryGenerator copTrajectory;
   private final FlamingoCoPTrajectoryGenerator flamingoCopTrajectory;

   private final AngularMomentumHandler<SettableContactStateProvider> angularMomentumHandler;
   private final CoMTrajectoryPlanner comTrajectoryPlanner;
   private final int maxNumberOfStepsToConsider;
   private final BooleanProvider maintainInitialCoMVelocityContinuitySingleSupport;
   private final BooleanProvider maintainInitialCoMVelocityContinuityTransfer;

   public BalanceManager(HighLevelHumanoidControllerToolbox controllerToolbox,
                         WalkingControllerParameters walkingControllerParameters,
                         CoPTrajectoryParameters copTrajectoryParameters,
                         SplitFractionCalculatorParametersReadOnly splitFractionParameters,
                         YoRegistry parentRegistry)
   {
      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();

      YoGraphicsListRegistry yoGraphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();

      double gravityZ = controllerToolbox.getGravityZ();
      double totalMass = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());

      this.controllerToolbox = controllerToolbox;
      yoTime = controllerToolbox.getYoTime();

      contactStateBasedPredicate = robotSide -> controllerToolbox.getFootContactState(robotSide).inContact();
      contactStateManager = new ContactStateManager(yoTime, walkingControllerParameters, registry);

      angularMomentumHandler = new AngularMomentumHandler<>(totalMass,
                                                            gravityZ,
                                                            controllerToolbox,
                                                            controllerToolbox.getReferenceFrames().getSoleFrames(),
                                                            SettableContactStateProvider::new,
                                                            registry,
                                                            yoGraphicsListRegistry);

      walkingControllerParameters.getICPControllerParameters().createFeedbackAlphaCalculator(registry, null);
      feedbackAlphaCalculator = walkingControllerParameters.getICPControllerParameters().getFeedbackAlphaCalculator();

      icpErrorThresholdToAdjustTime.set(walkingControllerParameters.getICPErrorThresholdToSpeedUpSwing());

      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();

      bipedSupportPolygons = controllerToolbox.getBipedSupportPolygons();
      icpControlPlane = new ICPControlPlane(centerOfMassFrame, gravityZ, registry);
      icpControlPolygons = new ICPControlPolygons(icpControlPlane, registry, yoGraphicsListRegistry);

      WalkingMessageHandler walkingMessageHandler = controllerToolbox.getWalkingMessageHandler();
      momentumTrajectoryHandler = walkingMessageHandler == null ? null : walkingMessageHandler.getMomentumTrajectoryHandler();

      if (walkingMessageHandler != null)
      {
         CenterOfMassTrajectoryHandler comTrajectoryHandler = walkingMessageHandler.getComTrajectoryHandler();
         double dt = controllerToolbox.getControlDT();
         precomputedICPPlanner = new PrecomputedICPPlanner(dt, comTrajectoryHandler, momentumTrajectoryHandler, registry, yoGraphicsListRegistry);
         precomputedICPPlanner.setOmega0(controllerToolbox.getOmega0());
         precomputedICPPlanner.setMass(totalMass);
         precomputedICPPlanner.setGravity(gravityZ);
      }
      else
      {
         precomputedICPPlanner = null;
      }
      blendICPTrajectories.set(true);

      distanceToShrinkSupportPolygonWhenHoldingCurrent.set(0.08);

      maxICPErrorBeforeSingleSupportForwardX = new DoubleParameter("maxICPErrorBeforeSingleSupportForwardX",
                                                                   registry,
                                                                   walkingControllerParameters.getMaxICPErrorBeforeSingleSupportForwardX());
      maxICPErrorBeforeSingleSupportBackwardX = new DoubleParameter("maxICPErrorBeforeSingleSupportBackwardX",
                                                                    registry,
                                                                    walkingControllerParameters.getMaxICPErrorBeforeSingleSupportBackwardX());
      maxICPErrorBeforeSingleSupportInnerY = new DoubleParameter("maxICPErrorBeforeSingleSupportInnerY",
                                                                 registry,
                                                                 walkingControllerParameters.getMaxICPErrorBeforeSingleSupportInnerY());
      maxICPErrorBeforeSingleSupportOuterY = new DoubleParameter("maxICPErrorBeforeSingleSupportOuterY",
                                                                 registry,
                                                                 walkingControllerParameters.getMaxICPErrorBeforeSingleSupportOuterY());

      double pelvisTranslationICPSupportPolygonSafeMargin = walkingControllerParameters.getPelvisTranslationICPSupportPolygonSafeMargin();
      pelvisICPBasedTranslationManager = new PelvisICPBasedTranslationManager(controllerToolbox,
                                                                              pelvisTranslationICPSupportPolygonSafeMargin,
                                                                              bipedSupportPolygons,
                                                                              registry);

      SideDependentList<FrameConvexPolygon2D> defaultSupportPolygons = controllerToolbox.getDefaultFootPolygons();
      soleFrames = controllerToolbox.getReferenceFrames().getSoleFrames();
      registry.addChild(copTrajectoryParameters.getRegistry());
      maxNumberOfStepsToConsider = copTrajectoryParameters.getMaxNumberOfStepsToConsider();
      maintainInitialCoMVelocityContinuitySingleSupport = new BooleanParameter("maintainInitialCoMVelocityContinuitySingleSupport", registry, true);
      maintainInitialCoMVelocityContinuityTransfer = new BooleanParameter("maintainInitialCoMVelocityContinuityTransfer", registry, true);
      comTrajectoryPlanner = new CoMTrajectoryPlanner(controllerToolbox.getGravityZ(), controllerToolbox.getOmega0Provider(), registry);
      comTrajectoryPlanner.setComContinuityCalculator(new CoMContinuousContinuityCalculator(controllerToolbox.getGravityZ(),
                                                                                            controllerToolbox.getOmega0Provider(),
                                                                                            registry));
      copTrajectoryState = new CoPTrajectoryGeneratorState(registry, maxNumberOfStepsToConsider);
      copTrajectoryState.registerStateToSave(copTrajectoryParameters);
      copTrajectory = new WalkingCoPTrajectoryGenerator(copTrajectoryParameters, splitFractionParameters, defaultSupportPolygons, registry);
      copTrajectory.registerState(copTrajectoryState);
      flamingoCopTrajectory = new FlamingoCoPTrajectoryGenerator(copTrajectoryParameters, registry);
      flamingoCopTrajectory.registerState(copTrajectoryState);

      if (USE_ERROR_BASED_STEP_ADJUSTMENT)
      {
         stepAdjustmentController = new ErrorBasedStepAdjustmentController(walkingControllerParameters,
                                                                           controllerToolbox.getReferenceFrames().getSoleZUpFrames(),
                                                                           bipedSupportPolygons,
                                                                           icpControlPolygons,
                                                                           controllerToolbox.getContactableFeet(),
                                                                           registry,
                                                                           yoGraphicsListRegistry);
      }
      else
      {
         stepAdjustmentController = new CaptureRegionStepAdjustmentController(walkingControllerParameters,
                                                                              controllerToolbox.getReferenceFrames().getSoleZUpFrames(),
                                                                              bipedSupportPolygons,
                                                                              registry,
                                                                              yoGraphicsListRegistry);
      }

      String graphicListName = getClass().getSimpleName();

      if (yoGraphicsListRegistry != null)
      {
         if (viewCoPHistory)
         {
            perfectCoPTrajectory = new BagOfBalls(150, 0.002, "perfectCoP", DarkViolet(), GraphicType.BALL_WITH_CROSS, registry, yoGraphicsListRegistry);
            perfectCMPTrajectory = new BagOfBalls(150, 0.002, "perfectCMP", BlueViolet(), GraphicType.BALL, registry, yoGraphicsListRegistry);
         }
         else
         {
            perfectCoPTrajectory = null;
            perfectCMPTrajectory = null;
         }

         // TODO Don't merge to develop as is
         //         comTrajectoryPlanner.setCornerPointViewer(new CornerPointViewer(true, false, registry, yoGraphicsListRegistry));
         //         copTrajectory.setWaypointViewer(new CoPPointViewer(registry, yoGraphicsListRegistry));

         YoGraphicPosition desiredCapturePointViz = new YoGraphicPosition("Desired Capture Point",
                                                                          yoDesiredCapturePoint,
                                                                          0.01,
                                                                          Yellow(),
                                                                          GraphicType.BALL_WITH_ROTATED_CROSS);
         YoGraphicPosition finalDesiredCapturePointViz = new YoGraphicPosition("Final Desired Capture Point",
                                                                               yoFinalDesiredICP,
                                                                               0.01,
                                                                               Beige(),
                                                                               GraphicType.BALL_WITH_ROTATED_CROSS);
         YoGraphicPosition finalDesiredCoMViz = new YoGraphicPosition("Final Desired CoM",
                                                                      yoFinalDesiredCoM,
                                                                      0.01,
                                                                      Black(),
                                                                      GraphicType.BALL_WITH_ROTATED_CROSS);
         YoGraphicPosition perfectCMPViz = new YoGraphicPosition("Perfect CMP", yoPerfectCMP, 0.002, BlueViolet());
         YoGraphicPosition perfectCoPViz = new YoGraphicPosition("Perfect CoP", yoPerfectCoP, 0.002, DarkViolet(), GraphicType.BALL_WITH_CROSS);

         yoGraphicsListRegistry.registerArtifact(graphicListName, desiredCapturePointViz.createArtifact());
         yoGraphicsListRegistry.registerArtifact(graphicListName, finalDesiredCapturePointViz.createArtifact());
         yoGraphicsListRegistry.registerArtifact(graphicListName, finalDesiredCoMViz.createArtifact());
         YoArtifactPosition perfectCMPArtifact = perfectCMPViz.createArtifact();
         perfectCMPArtifact.setVisible(false);
         yoGraphicsListRegistry.registerArtifact(graphicListName, perfectCMPArtifact);
         YoArtifactPosition perfectCoPArtifact = perfectCoPViz.createArtifact();
         perfectCoPArtifact.setVisible(false);
         yoGraphicsListRegistry.registerArtifact(graphicListName, perfectCoPArtifact);
      }
      else
      {
         perfectCoPTrajectory = null;
         perfectCMPTrajectory = null;
      }
      yoDesiredCapturePoint.setToNaN();
      yoFinalDesiredICP.setToNaN();
      yoFinalDesiredCoP.setToNaN();
      yoPerfectCMP.setToNaN();
      yoPerfectCoP.setToNaN();
      yoPerfectCoPVelocity.setToNaN();

      parentRegistry.addChild(registry);
   }

   public Footstep getFootstep(int stepNumber)
   {
      return footsteps.get(stepNumber);
   }

   public FootstepTiming getFootstepTiming(int stepNumber)
   {
      return footstepTimings.get(stepNumber);
   }

   public void setUseMomentumRecoveryModeForBalance(boolean useMomentumRecoveryModeForBalance)
   {
      this.useMomentumRecoveryModeForBalance.set(useMomentumRecoveryModeForBalance);
   }

   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing)
   {
      copTrajectoryState.addFootstep(footstep);
      copTrajectoryState.addFootstepTiming(timing);
      footsteps.add(footstep);
      footstepTimings.add(timing);
   }

   public void setCurrentStepConstraints(StepConstraintRegionsList stepConstraintRegionsList)
   {
      stepAdjustmentController.setStepConstraintRegions(stepConstraintRegionsList.getAsList());
   }

   public List<StepConstraintRegion> getCurrentStepConstraints()
   {
      return stepAdjustmentController.getStepConstraintRegions();
   }

   public boolean checkAndUpdateStepAdjustment(Footstep footstep)
   {
      boolean usingStepAdjustment = stepAdjustmentController.useStepAdjustment();

      if (!usingStepAdjustment)
      {
         return false;
      }
      // FIXME For now, only compute this at the beginning of swing. Once the step is adjusted, it should likely increase, and anecdotally, we want to prevent
      // FIXME this value from "running away" as the step continues to be adjusted. In theory, this should be ok, but it looks like it isn't. More
      // FIXME experimentation is necessary. RJG 07/27/2023
      if (initializeOnStateChange && feedbackAlphaCalculator != null)
      {
         this.finalFeedbackAlpha.set(feedbackAlphaCalculator.computeAlpha(yoFinalDesiredICP, bipedSupportPolygons.getSupportPolygonInWorld()));
      }

      double omega0 = controllerToolbox.getOmega0();

      controllerToolbox.getCapturePoint(capturePoint2d);
      icpControlPlane.setOmega0(omega0);
      icpControlPolygons.updateUsingContactStateCommand(contactStateCommands);

      if (!swingSpeedUpForStepAdjustment.isNaN() && swingSpeedUpForStepAdjustment.getValue() > 0.0)
         stepAdjustmentController.submitSwingSpeedUpUnderDisturbance(swingSpeedUpForStepAdjustment.getValue());

      // compute the amount to scale the support polygon, and the point to scale it about.
      double feedbackAlpha = Double.NaN;
      if (feedbackAlphaCalculator != null)
      {
         this.currentFeedbackAlpha.set(feedbackAlphaCalculator.computeAlpha(capturePoint2d, bipedSupportPolygons.getSupportPolygonInWorld()));
         double maxAlpha = Math.max(currentFeedbackAlpha.getDoubleValue(), finalFeedbackAlpha.getDoubleValue());

         if (getAdjustedTimeRemainingInCurrentSupportSequence() > 1e-5)
         {
            // get the feedback alpha that will be encountered at the end of swing.

            // Compute the CoP that is equivalent to having a constant COP for the duration of the support phase.
            double exponential = Math.exp(omega0 * getAdjustedTimeRemainingInCurrentSupportSequence());
            yoEquivalentRemainingCoP.scaleAdd(-exponential, yoDesiredCapturePoint, yoFinalDesiredICP);
            yoEquivalentRemainingCoP.scale(1.0 / (1.0 - exponential));

            // clamp it to be between the end points
            clampBetweenTwoPoints(yoEquivalentRemainingCoP, perfectCMP2d, yoFinalDesiredCoP);

            /* FIXME don't use this logic block, it doesn't work well, and results in significantly more scaling than we hsould use. RJG 07/27/2023
            if (perfectCMP2d.distanceSquared(yoFinalDesiredCoP) > 1e-3)
            {
               // if the CMP isn't at the end of the trajectory, figure out how far through the trajectory you are.
               double alphaRemaining = EuclidGeometryTools.percentageAlongLineSegment2D(yoEquivalentRemainingCoP, perfectCMP2d, yoFinalDesiredCoP);
               // Use this value to figure out what kind of support polygon scaling to do.
               feedbackAlpha = InterpolationTools.linearInterpolate(currentFeedbackAlpha.getDoubleValue(), maxAlpha, alphaRemaining);
            }
            else
            {
            */
               feedbackAlpha = 0.5 * (currentFeedbackAlpha.getDoubleValue() + maxAlpha);
//            }
            feedbackAlpha = MathTools.clamp(feedbackAlpha, 0.0, 1.0);
         }
         else
         {
            yoEquivalentRemainingCoP.set(yoFinalDesiredCoP);
            feedbackAlpha = maxAlpha;
         }
      }

      // use 1.0 - feedback alpha, because 0.0 feedback alpha does nothing, while 1.0 should allow no feedback.
      stepAdjustmentController.compute(yoTime.getDoubleValue(), yoDesiredCapturePoint, capturePoint2d, omega0, yoEquivalentRemainingCoP, 1.0 - feedbackAlpha);
      boolean footstepWasAdjusted = stepAdjustmentController.wasFootstepAdjusted();
      if (footstepWasAdjusted)
         footstep.setPose(stepAdjustmentController.getFootstepSolution());

      return footstepWasAdjusted;
   }

   private static void clampBetweenTwoPoints(FixedFramePoint2DBasics pointToClamp, FramePoint2DReadOnly pointA, FramePoint2DReadOnly pointB)
   {
      double minX = Math.min(pointA.getX(), pointB.getX());
      double maxX = Math.max(pointA.getX(), pointB.getX());

      double minY = Math.min(pointA.getY(), pointB.getY());
      double maxY = Math.max(pointA.getY(), pointB.getY());

      pointToClamp.setX(MathTools.clamp(pointToClamp.getX(), minX, maxX));
      pointToClamp.setY(MathTools.clamp(pointToClamp.getY(), minY, maxY));
   }

   public void clearICPPlan()
   {
      copTrajectoryState.clear();
      footsteps.clear();
      footstepTimings.clear();
   }

   public void setSwingFootTrajectory(RobotSide swingSide, MultipleWaypointsPoseTrajectoryGenerator swingTrajectory)
   {
      angularMomentumHandler.setSwingFootTrajectory(swingSide, swingTrajectory);
   }

   public void clearSwingFootTrajectory()
   {
      angularMomentumHandler.clearSwingFootTrajectory();
   }

   public void setICPPlanSupportSide(RobotSide supportSide)
   {
      this.supportSide = supportSide;
   }

   public void updateTimeInState()
   {
      shouldAdjustTimeFromTrackingError.set(getICPErrorMagnitude() > icpErrorThresholdToAdjustTime.getDoubleValue());
      contactStateManager.updateTimeInState(timeShiftProvider, shouldAdjustTimeFromTrackingError.getBooleanValue());
   }

   public void compute(RobotSide supportLeg, FeedbackControlCommand<?> heightControlCommand, boolean isUpperBodyLoadBearing, boolean controlHeightWithMomentum)
   {
      desiredCapturePoint2d.set(comTrajectoryPlanner.getDesiredDCMPosition());
      desiredCapturePointVelocity2d.set(comTrajectoryPlanner.getDesiredDCMVelocity());

      perfectCMP2d.set(comTrajectoryPlanner.getDesiredECMPPosition());
      desiredCoM2d.set(comTrajectoryPlanner.getDesiredCoMPosition());
      yoDesiredCoMVelocity.set(comTrajectoryPlanner.getDesiredCoMVelocity());

      capturePoint2d.setIncludingFrame(controllerToolbox.getCapturePoint());
      pelvisICPBasedTranslationManager.compute(supportLeg, isUpperBodyLoadBearing);
      pelvisICPBasedTranslationManager.addICPOffset(desiredCapturePoint2d, desiredCoM2d, perfectCMP2d);

      double omega0 = controllerToolbox.getOmega0();
      if (Double.isNaN(omega0))
         throw new RuntimeException("omega0 is NaN");

      if (precomputedICPPlanner != null)
      {
         if (blendICPTrajectories.getBooleanValue())
         {
            precomputedICPPlanner.computeAndBlend(yoTime.getDoubleValue(), desiredCapturePoint2d, desiredCapturePointVelocity2d, perfectCMP2d);
         }
         else
         {
            precomputedICPPlanner.compute(yoTime.getDoubleValue(), desiredCapturePoint2d, desiredCapturePointVelocity2d, perfectCMP2d);
         }
      }

      decayDesiredICPVelocity();
      if (!icpVelocityReductionFactor.isNaN())
         desiredCapturePointVelocity2d.scale(icpVelocityReductionFactor.getValue());

      yoDesiredCapturePoint.set(desiredCapturePoint2d);
      yoDesiredICPVelocity.set(desiredCapturePointVelocity2d);
      yoDesiredCoMPosition.set(desiredCoM2d, comTrajectoryPlanner.getDesiredCoMPosition().getZ());
      yoPerfectCoPVelocity.set(comTrajectoryPlanner.getDesiredVRPVelocity());

      CapturePointTools.computeCentroidalMomentumPivot(yoDesiredCapturePoint, yoDesiredICPVelocity, omega0, perfectCMP2d);
      yoPerfectCMP.set(perfectCMP2d, comTrajectoryPlanner.getDesiredECMPPosition().getZ());

      // This guy relies on the desired ICP being updated.
      updateTimeInState();

      if (computeAngularMomentumOffset.getValue())
         angularMomentumHandler.computeCoPPosition(yoPerfectCMP, yoPerfectCoP);
      else
         yoPerfectCoP.set(yoPerfectCMP);

      for (RobotSide robotSide : RobotSide.values)
      {
         YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);
         contactState.getPlaneContactStateCommand(contactStateCommands.get(robotSide));
      }

      if (heightControlCommand == null)
      {
         linearMomentumRateControlModuleInput.setHasHeightCommand(false);
      }
      else if (heightControlCommand.getCommandType() == ControllerCoreCommandType.POINT)
      {
         linearMomentumRateControlModuleInput.setHasHeightCommand(true);
         linearMomentumRateControlModuleInput.setUsePelvisHeightCommand(true);
         linearMomentumRateControlModuleInput.setPelvisHeightControlCommand((PointFeedbackControlCommand) heightControlCommand);
      }
      else if (heightControlCommand.getCommandType() == ControllerCoreCommandType.MOMENTUM)
      {
         linearMomentumRateControlModuleInput.setHasHeightCommand(true);
         linearMomentumRateControlModuleInput.setUsePelvisHeightCommand(false);
         linearMomentumRateControlModuleInput.setCenterOfMassHeightControlCommand((CenterOfMassFeedbackControlCommand) heightControlCommand);
      }
      else
      {
         throw new IllegalArgumentException("Invalid height control type.");
      }

      if (perfectCMPTrajectory != null)
      {
         perfectCMPTrajectory.setBallLoop(yoPerfectCMP);
         perfectCoPTrajectory.setBallLoop(yoPerfectCoP);
      }

      perfectCMP2d.setIncludingFrame(yoPerfectCMP);
      perfectCoP2d.setIncludingFrame(yoPerfectCoP);
      linearMomentumRateControlModuleInput.setInitializeOnStateChange(initializeOnStateChange);
      linearMomentumRateControlModuleInput.setKeepCoPInsideSupportPolygon(!isUpperBodyLoadBearing);
      linearMomentumRateControlModuleInput.setControlHeightWithMomentum(controlHeightWithMomentum);
      linearMomentumRateControlModuleInput.setOmega0(omega0);
      linearMomentumRateControlModuleInput.setUseMomentumRecoveryMode(useMomentumRecoveryModeForBalance.getBooleanValue());
      linearMomentumRateControlModuleInput.setDesiredCapturePoint(desiredCapturePoint2d);
      linearMomentumRateControlModuleInput.setDesiredCapturePointVelocity(desiredCapturePointVelocity2d);
      linearMomentumRateControlModuleInput.setDesiredCapturePointAtEndOfState(yoFinalDesiredICP);
      linearMomentumRateControlModuleInput.setPerfectCMP(perfectCMP2d);
      linearMomentumRateControlModuleInput.setPerfectCoP(perfectCoP2d);
      linearMomentumRateControlModuleInput.setMinimizeAngularMomentumRateZ(minimizeAngularMomentumRateZ);
      linearMomentumRateControlModuleInput.setContactStateCommand(contactStateCommands);

      initializeOnStateChange = false;

      supportSide = null;

      // This is for debugging such that the momentum trajectory handler YoVariables contain the current value:
      if (momentumTrajectoryHandler != null)
      {
         momentumTrajectoryHandler.packDesiredAngularMomentumAtTime(yoTime.getValue(), null, null);
      }
   }

   public void adjustFootstepInCoPPlan(Footstep footstep)
   {
      copTrajectoryState.getFootstep(0).set(footstep);
   }

   public void setHoldSplitFractions(boolean hold)
   {
      copTrajectory.setHoldSplitFractions(hold);
   }

   public void computeICPPlan()
   {
      computeICPPlan(contactStateBasedPredicate);
   }

   public void computeICPPlan(Predicate<RobotSide> isFootInSupport)
   {
      computeICPPlanInternal(isFootInSupport, copTrajectory);
   }

   public void computeFlamingoStateICPPlan()
   {
      computeAngularMomentumOffset.set(useAngularMomentumOffset.getValue() && useAngularMomentumOffsetInStanding.getValue());
      computeICPPlanInternal(flamingoCopTrajectory);
   }

   private void computeICPPlanInternal(CoPTrajectoryGenerator copTrajectory)
   {
      computeICPPlanInternal(contactStateBasedPredicate, copTrajectory);
   }

   private void computeICPPlanInternal(Predicate<RobotSide> isFootInSupport, CoPTrajectoryGenerator copTrajectory)
   {
      plannerTimer.startMeasurement();

      // update online to account for foot slip
      for (RobotSide robotSide : RobotSide.values)
      {
         if (isFootInSupport.test(robotSide))
            copTrajectoryState.initializeStance(robotSide, bipedSupportPolygons.getFootPolygonInSoleFrame(robotSide), soleFrames.get(robotSide));
      }

      copTrajectory.compute(copTrajectoryState);

      List<SettableContactStateProvider> contactStateProviders = copTrajectory.getContactStateProviders();
      amPlanningTimerTimer.startMeasurement();
      if (computeAngularMomentumOffset.getValue())
      {
         if (comTrajectoryPlanner.hasTrajectories())
         {
            angularMomentumHandler.solveForAngularMomentumTrajectory(copTrajectoryState, contactStateProviders, comTrajectoryPlanner.getCoMTrajectory());
            contactStateProviders = angularMomentumHandler.computeECMPTrajectory(contactStateProviders);
         }
         else
         {
            angularMomentumHandler.resetAngularMomentum();
         }

         angularMomentumHandler.computeAngularMomentum(Math.min(contactStateManager.getTimeInSupportSequence(),
                                                                contactStateManager.getCurrentStateDuration() - 1e-3));
      }
      else
      {
         comTrajectoryPlanner.reset();
      }
      amPlanningTimerTimer.stopMeasurement();

      comTrajectoryPlanner.solveForTrajectory(contactStateProviders);

      comTrajectoryPlanner.compute(contactStateManager.getCurrentStateDuration());
      yoFinalDesiredICP.set(comTrajectoryPlanner.getDesiredDCMPosition());
      yoFinalDesiredCoP.set(comTrajectoryPlanner.getDesiredECMPPosition()); // TODO replace with CoP

      comTrajectoryPlanner.compute(contactStateManager.getTotalStateDuration());

      yoFinalDesiredCoM.set(comTrajectoryPlanner.getDesiredCoMPosition());
      yoFinalDesiredCoMVelocity.set(comTrajectoryPlanner.getDesiredCoMVelocity());
      yoFinalDesiredCoMAcceleration.set(comTrajectoryPlanner.getDesiredCoMAcceleration());

      comTrajectoryPlanner.compute(contactStateManager.getTimeInSupportSequence());

      if (footstepTimings.isEmpty())
      {
         yoFinalDesiredICP.setToNaN();
         yoFinalDesiredCoP.setToNaN();
         yoFinalDesiredCoM.setToNaN();
         yoFinalDesiredCoMVelocity.setToNaN();
         yoFinalDesiredCoMAcceleration.setToNaN();
      }

      plannerTimer.stopMeasurement();
   }

   /**
    * Time-based decay of the desired ICP velocity activated when a double support is going over the
    * planner state duration.
    */
   private void decayDesiredICPVelocity()
   {
      if (Double.isNaN(icpVelocityDecayDurationWhenDone.getValue()))
      {
         icpVelocityReductionFactor.set(Double.NaN);
         return;
      }

      if (contactStateManager.isInSingleSupport() || !contactStateManager.isContactStateDone())
      {
         icpVelocityReductionFactor.set(Double.NaN);
         return;
      }

      if (icpVelocityReductionFactor.isNaN())
      {
         icpVelocityReductionFactor.set(1.0);
         return;
      }

      double scaleUpdated = icpVelocityReductionFactor.getValue() - controllerToolbox.getControlDT() / icpVelocityDecayDurationWhenDone.getValue();
      icpVelocityReductionFactor.set(Math.max(0.0, scaleUpdated));
   }

   public void disablePelvisXYControl()
   {
      pelvisICPBasedTranslationManager.disable();
   }

   public void enablePelvisXYControl()
   {
      pelvisICPBasedTranslationManager.enable();
   }

   private double computeTimeShiftToMinimizeTrackingError()
   {
      controllerToolbox.getCapturePoint(capturePoint2d);
      perfectCMP2d.set(yoPerfectCMP);
      return timeAdjustmentCalculator.estimateDeltaTimeBetweenDesiredICPAndActualICP(yoDesiredCapturePoint,
                                                                                     perfectCMP2d,
                                                                                     yoFinalDesiredICP,
                                                                                     capturePoint2d,
                                                                                     controllerToolbox.getOmega0());
   }

   public void freezePelvisXYControl()
   {
      pelvisICPBasedTranslationManager.reset();
   }

   public int getMaxNumberOfStepsToConsider()
   {
      return maxNumberOfStepsToConsider;
   }

   public int getNumberOfStepsBeingConsidered()
   {
      return footsteps.size();
   }

   public FramePoint2DReadOnly getDesiredCMP()
   {
      return desiredCMP;
   }

   public FramePoint2DReadOnly getFinalDesiredICP()
   {
      return yoFinalDesiredICP;
   }

   public FramePoint2DReadOnly getDesiredICP()
   {
      return yoDesiredCapturePoint;
   }

   public FramePoint2DReadOnly getPerfectCoP()
   {
      perfectCoP2d.set(yoPerfectCoP);
      return perfectCoP2d;
   }

   public FrameVector2DReadOnly getDesiredICPVelocity()
   {
      return yoDesiredICPVelocity;
   }

   public FramePoint3DReadOnly getFinalDesiredCoMPosition()
   {
      return yoFinalDesiredCoM;
   }

   public FrameVector3DReadOnly getFinalDesiredCoMVelocity()
   {
      return yoFinalDesiredCoMVelocity;
   }

   public FrameVector3DReadOnly getFinalDesiredCoMAcceleration()
   {
      return yoFinalDesiredCoMAcceleration;
   }

   public FrameVector3DReadOnly getDesiredCoMVelocity()
   {
      return yoDesiredCoMVelocity;
   }

   public FramePoint3DReadOnly getFirstExitCMPForToeOff(boolean isInTransfer)
   {
      if (isInTransfer)
         return copTrajectory.getContactStateProviders().get(0).getECMPStartPosition();
      else
         return copTrajectory.getContactStateProviders().get(4).getECMPEndPosition();
   }

   public double getTimeRemainingInCurrentState()
   {
      return contactStateManager.getTimeRemainingInCurrentSupportSequence();
   }

   public double getTimeIntoCurrentSupportSequence()
   {
      return contactStateManager.getTimeInSupportSequence();
   }

   public double getAdjustedTimeRemainingInCurrentSupportSequence()
   {
      return contactStateManager.getAdjustedTimeRemainingInCurrentSupportSequence();
   }

   public double getExtraTimeAdjustmentForSwing()
   {
      return contactStateManager.getExtraTimeAdjustmentForSwing();
   }

   public void goHome(double trajectoryDuration)
   {
      if (pelvisICPBasedTranslationManager.isEnabled())
         pelvisICPBasedTranslationManager.goToHome(trajectoryDuration);
   }

   public void handlePelvisTrajectoryCommand(PelvisTrajectoryCommand command)
   {
      pelvisICPBasedTranslationManager.handlePelvisTrajectoryCommand(command);
   }

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      pelvisICPBasedTranslationManager.handleStopAllTrajectoryCommand(command);
   }

   public void initialize()
   {
      yoFinalDesiredICP.setToNaN();
      yoFinalDesiredCoP.setToNaN();
      yoFinalDesiredCoM.setToNaN();
      yoFinalDesiredCoMVelocity.setToNaN();
      yoFinalDesiredCoMAcceleration.setToNaN();
      yoDesiredCapturePoint.set(controllerToolbox.getCapturePoint());
      yoDesiredCoMPosition.setFromReferenceFrame(controllerToolbox.getCenterOfMassFrame());
      yoDesiredCoMVelocity.setToZero();

      yoPerfectCoP.setMatchingFrame(bipedSupportPolygons.getSupportPolygonInMidFeetZUp().getCentroid(), 0.0);
      copTrajectoryState.setInitialCoP(yoPerfectCoP);
      copTrajectoryState.initializeStance(bipedSupportPolygons.getFootPolygonsInSoleZUpFrame(), soleFrames);
      comTrajectoryPlanner.setInitialCenterOfMassState(yoDesiredCoMPosition, yoDesiredCoMVelocity);

      contactStateManager.initialize();

      comTrajectoryPlanner.setMaintainInitialCoMVelocityContinuity(false);

      initializeOnStateChange = true;
      currentStepConstraints.clear();
      stepAdjustmentController.reset();
      comTrajectoryPlanner.reset();
      angularMomentumHandler.resetAngularMomentum();
   }

   public void initializeICPPlanForSingleSupport()
   {
      computeAngularMomentumOffset.set(useAngularMomentumOffset.getValue());
      currentFootstep.set(footsteps.get(0));

      double swingTime = footstepTimings.get(0).getSwingTime();
      double transferTime = footstepTimings.get(0).getTransferTime();

      currentStepConstraints.clear();
      stepAdjustmentController.reset();
      if (footsteps.size() > 1 && footstepTimings.size() > 1)
      {
         stepAdjustmentController.setFootstepQueueInformation(footsteps.size(), footstepTimings.get(1).getStepTime());
      }
      stepAdjustmentController.setFootstepToAdjust(currentFootstep, swingTime);
      stepAdjustmentController.initialize(yoTime.getDoubleValue(), supportSide);

      contactStateManager.initializeForSingleSupport(transferTime, swingTime);
      swingSpeedUpForStepAdjustment.setToNaN();

      comTrajectoryPlanner.setMaintainInitialCoMVelocityContinuity(maintainInitialCoMVelocityContinuitySingleSupport.getValue());
      initializeOnStateChange = true;
   }

   public void initializeICPPlanForStanding()
   {
      if (holdICPToCurrentCoMLocationInNextDoubleSupport.getBooleanValue())
      {
         requestICPPlannerToHoldCurrentCoM();
         holdICPToCurrentCoMLocationInNextDoubleSupport.set(false);
      }
      computeAngularMomentumOffset.set(useAngularMomentumOffset.getValue() && useAngularMomentumOffsetInStanding.getValue());

      angularMomentumHandler.clearSwingFootTrajectory();

      copTrajectoryState.setInitialCoP(yoPerfectCoP);
      copTrajectoryState.initializeStance(bipedSupportPolygons.getFootPolygonsInSoleZUpFrame(), soleFrames);
      comTrajectoryPlanner.setInitialCenterOfMassState(yoDesiredCoMPosition, yoDesiredCoMVelocity);
      comTrajectoryPlanner.initializeTrajectory(yoDesiredCoMPosition, Double.POSITIVE_INFINITY);
      swingSpeedUpForStepAdjustment.setToNaN();

      initializeOnStateChange = true;
      comTrajectoryPlanner.setMaintainInitialCoMVelocityContinuity(true);
      currentStepConstraints.clear();
      stepAdjustmentController.reset();
   }

   public void initializeICPPlanForTransferToStanding()
   {
      comTrajectoryPlanner.removeCompletedSegments(contactStateManager.getTotalStateDuration());
      computeAngularMomentumOffset.set(useAngularMomentumOffset.getValue());

      if (holdICPToCurrentCoMLocationInNextDoubleSupport.getBooleanValue())
      {
         requestICPPlannerToHoldCurrentCoM();
         holdICPToCurrentCoMLocationInNextDoubleSupport.set(false);
      }
      copTrajectoryState.setInitialCoP(yoPerfectCoP);
      copTrajectoryState.initializeStance(bipedSupportPolygons.getFootPolygonsInSoleZUpFrame(), soleFrames);
      comTrajectoryPlanner.setInitialCenterOfMassState(yoDesiredCoMPosition, yoDesiredCoMVelocity);
      swingSpeedUpForStepAdjustment.setToNaN();

      contactStateManager.initializeForTransferToStanding(copTrajectoryState.getFinalTransferDuration());

      initializeOnStateChange = true;
      comTrajectoryPlanner.setMaintainInitialCoMVelocityContinuity(true);
      currentStepConstraints.clear();
      stepAdjustmentController.reset();
   }

   public void initializeICPPlanForTransfer()
   {
      computeAngularMomentumOffset.set(useAngularMomentumOffset.getValue());
      if (holdICPToCurrentCoMLocationInNextDoubleSupport.getBooleanValue())
      {
         requestICPPlannerToHoldCurrentCoM();
         holdICPToCurrentCoMLocationInNextDoubleSupport.set(false);
      }

      currentStepConstraints.clear();
      stepAdjustmentController.reset();

      comTrajectoryPlanner.removeCompletedSegments(contactStateManager.getTotalStateDuration());

      copTrajectoryState.setInitialCoP(yoPerfectCoP);
      copTrajectoryState.initializeStance(bipedSupportPolygons.getFootPolygonsInSoleZUpFrame(), soleFrames);
      comTrajectoryPlanner.setInitialCenterOfMassState(yoDesiredCoMPosition, yoDesiredCoMVelocity);

      contactStateManager.initializeForTransfer(footstepTimings.get(0).getTransferTime(), footstepTimings.get(0).getSwingTime());
      swingSpeedUpForStepAdjustment.setToNaN();

      comTrajectoryPlanner.setMaintainInitialCoMVelocityContinuity(maintainInitialCoMVelocityContinuityTransfer.getValue());

      initializeOnStateChange = true;
   }

   public void computeNormalizedEllipticICPError(RobotSide transferToSide)
   {
      getICPError(icpError2d);
      ReferenceFrame leadingSoleZUpFrame = controllerToolbox.getReferenceFrames().getSoleZUpFrame(transferToSide);
      icpError2d.changeFrame(leadingSoleZUpFrame);
      boolean isICPErrorToTheInside = transferToSide == RobotSide.RIGHT ? icpError2d.getY() > 0.0 : icpError2d.getY() < 0.0;
      double maxICPErrorBeforeSingleSupportX = icpError2d.getX() > 0.0 ? maxICPErrorBeforeSingleSupportForwardX.getValue()
                                                                       : maxICPErrorBeforeSingleSupportBackwardX.getValue();
      double maxICPErrorBeforeSingleSupportY = isICPErrorToTheInside ? maxICPErrorBeforeSingleSupportInnerY.getValue()
                                                                     : maxICPErrorBeforeSingleSupportOuterY.getValue();
      normalizedICPError.set(MathTools.square(icpError2d.getX() / maxICPErrorBeforeSingleSupportX)
                             + MathTools.square(icpError2d.getY() / maxICPErrorBeforeSingleSupportY));
   }

   public double getNormalizedEllipticICPError()
   {
      return normalizedICPError.getValue();
   }

   public double getEllipticICPErrorForMomentumRecovery()
   {
      return ellipticICPErrorForMomentumRecovery.getValue();
   }

   public double getICPDistanceOutsideSupportForStep()
   {
      return icpDistanceOutsideSupportForStep.getValue();
   }

   public boolean shouldAdjustTimeFromTrackingError()
   {
      return shouldAdjustTimeFromTrackingError.getBooleanValue();
   }

   public double getICPErrorMagnitude()
   {
      return controllerToolbox.getCapturePoint().distanceXY(yoDesiredCapturePoint);
   }

   public void getICPError(FrameVector2D icpErrorToPack)
   {
      icpErrorToPack.setIncludingFrame(yoDesiredCapturePoint);
      icpErrorToPack.checkReferenceFrameMatch(controllerToolbox.getCapturePoint());
      icpErrorToPack.sub(controllerToolbox.getCapturePoint().getX(), controllerToolbox.getCapturePoint().getY());
   }

   public boolean isPrecomputedICPPlannerActive()
   {
      return precomputedICPPlanner.isWithinInterval(yoTime.getDoubleValue());
   }

   public boolean isICPPlanDone()
   {
      return contactStateManager.isContactStateDone();
   }

   public void requestICPPlannerToHoldCurrentCoMInNextDoubleSupport()
   {
      holdICPToCurrentCoMLocationInNextDoubleSupport.set(true);
   }

   public void requestICPPlannerToHoldCurrentCoM()
   {
      centerOfMassPosition.setToZero(centerOfMassFrame);

      FrameConvexPolygon2DReadOnly supportPolygonInMidFeetZUp = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();
      convexPolygonShrinker.scaleConvexPolygon(supportPolygonInMidFeetZUp,
                                               distanceToShrinkSupportPolygonWhenHoldingCurrent.getDoubleValue(),
                                               shrunkSupportPolygon);

      centerOfMassPosition.changeFrame(shrunkSupportPolygon.getReferenceFrame());
      centerOfMassPosition2d.setIncludingFrame(centerOfMassPosition);
      shrunkSupportPolygon.orthogonalProjection(centerOfMassPosition2d);
      centerOfMassPosition.set(centerOfMassPosition2d, 0.0);

      centerOfMassPosition.changeFrame(worldFrame);

      // This tricks it to the current value.
      copTrajectoryState.setInitialCoP(centerOfMassPosition);
   }

   public void setFinalTransferTime(double finalTransferDuration)
   {
      copTrajectoryState.setFinalTransferDuration(finalTransferDuration);
   }

   public CapturabilityBasedStatus updateAndReturnCapturabilityBasedStatus()
   {
      centerOfMassPosition.setToZero(centerOfMassFrame);
      centerOfMassPosition.changeFrame(worldFrame);

      capturabilityBasedStatus.setOmega(controllerToolbox.getOmega0());
      capturabilityBasedStatus.getCapturePoint2d().set(controllerToolbox.getCapturePoint());
      capturabilityBasedStatus.getDesiredCapturePoint2d().set(yoDesiredCapturePoint);
      capturabilityBasedStatus.getCenterOfMass3d().set(centerOfMassPosition);
      for (RobotSide robotSide : RobotSide.values)
      {
         HumanoidMessageTools.packFootSupportPolygon(robotSide, bipedSupportPolygons.getFootPolygonInSoleFrame(robotSide), capturabilityBasedStatus);
      }

      return capturabilityBasedStatus;
   }

   public void updateSwingTimeRemaining(double timeRemainingInSwing)
   {
      swingSpeedUpForStepAdjustment.set(contactStateManager.getTimeRemainingInCurrentSupportSequence() - timeRemainingInSwing);
   }

   public FramePoint3DReadOnly getCapturePoint()
   {
      return controllerToolbox.getCapturePoint();
   }

   public void minimizeAngularMomentumRateZ(boolean minimizeAngularMomentumRateZ)
   {
      this.minimizeAngularMomentumRateZ = minimizeAngularMomentumRateZ;
   }

   public LinearMomentumRateControlModuleInput getLinearMomentumRateControlModuleInput()
   {
      return linearMomentumRateControlModuleInput;
   }

   public void setLinearMomentumRateControlModuleOutput(LinearMomentumRateControlModuleOutput output)
   {
      desiredCMP.set(output.getDesiredCMP());
   }

   public TaskspaceTrajectoryStatusMessage pollPelvisXYTranslationStatusToReport()
   {
      return pelvisICPBasedTranslationManager.pollStatusToReport();
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(angularMomentumHandler.getSCS2YoGraphics());
      group.addChild(icpControlPolygons.getSCS2YoGraphics());
      if (precomputedICPPlanner != null)
         group.addChild(precomputedICPPlanner.getSCS2YoGraphics());
      group.addChild(stepAdjustmentController.getSCS2YoGraphics());

      if (viewCoPHistory)
      {
         YoGraphicPointcloud3DDefinition perfectCoPTrajViz;
         YoGraphicPointcloud3DDefinition perfectCMPTrajViz;
         perfectCoPTrajViz = newYoGraphicPointcloud3D("perfectCoP", perfectCoPTrajectory.getPositions(), 0.004, ColorDefinitions.DarkViolet());
         perfectCMPTrajViz = newYoGraphicPointcloud3D("perfectCMP", perfectCMPTrajectory.getPositions(), 0.004, ColorDefinitions.DarkViolet());
         group.addChild(perfectCoPTrajViz);
         group.addChild(perfectCMPTrajViz);
         group.addChild(newYoGraphicPointcloud2D(perfectCoPTrajViz, DefaultPoint2DGraphic.CIRCLE_PLUS));
         group.addChild(newYoGraphicPointcloud2D(perfectCMPTrajViz, DefaultPoint2DGraphic.CIRCLE));
      }
      group.addChild(newYoGraphicPoint2D("Desired Capture Point",
                                         yoDesiredCapturePoint,
                                         0.02,
                                         ColorDefinitions.Yellow().darker(),
                                         DefaultPoint2DGraphic.CIRCLE_CROSS));
      group.addChild(newYoGraphicPoint2D("Final Desired Capture Point",
                                         yoFinalDesiredICP,
                                         0.02,
                                         ColorDefinitions.Beige().darker(),
                                         DefaultPoint2DGraphic.CIRCLE_CROSS));
      group.addChild(newYoGraphicPoint2D("Final Desired CoM", yoFinalDesiredCoM, 0.02, ColorDefinitions.Black(), DefaultPoint2DGraphic.CIRCLE_CROSS));
      YoGraphicPoint2DDefinition perfectCMPViz = newYoGraphicPoint2D("Perfect CMP",
                                                                     yoPerfectCMP,
                                                                     0.004,
                                                                     ColorDefinitions.BlueViolet(),
                                                                     DefaultPoint2DGraphic.CIRCLE);
      perfectCMPViz.setVisible(false);
      group.addChild(perfectCMPViz);
      YoGraphicPoint2DDefinition perfectCoPViz = newYoGraphicPoint2D("Perfect CoP",
                                                                     yoPerfectCoP,
                                                                     0.004,
                                                                     ColorDefinitions.DarkViolet(),
                                                                     DefaultPoint2DGraphic.CIRCLE_CROSS);
      perfectCoPViz.setVisible(false);
      group.addChild(perfectCoPViz);
      group.addChild(copTrajectory.getSCS2YoGraphics());

      return group;
   }
}
