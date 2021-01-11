package us.ihmc.commonWalkingControlModules.capturePoint;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.Beige;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Black;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.BlueViolet;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.DarkViolet;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Yellow;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentController;
import us.ihmc.commonWalkingControlModules.captureRegion.PushRecoveryControlModule;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisICPBasedTranslationManager;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryGeneratorState;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.FlamingoCoPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.WalkingCoPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMContinuousContinuityCalculator;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlanner;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CornerPointViewer;
import us.ihmc.commonWalkingControlModules.messageHandlers.CenterOfMassTrajectoryHandler;
import us.ihmc.commonWalkingControlModules.messageHandlers.MomentumTrajectoryHandler;
import us.ihmc.commonWalkingControlModules.messageHandlers.StepConstraintRegionHandler;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class BalanceManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final ICPControlPlane icpControlPlane;
   private final BipedSupportPolygons bipedSupportPolygons;
   private final ICPControlPolygons icpControlPolygons;

   private final MomentumTrajectoryHandler momentumTrajectoryHandler;
   private final PrecomputedICPPlanner precomputedICPPlanner;

   private final LinearMomentumRateControlModuleInput linearMomentumRateControlModuleInput = new LinearMomentumRateControlModuleInput();

   private final PelvisICPBasedTranslationManager pelvisICPBasedTranslationManager;
   private final PushRecoveryControlModule pushRecoveryControlModule;
   private final StepAdjustmentController stepAdjustmentController;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   private final YoFramePoint2D yoDesiredCapturePoint = new YoFramePoint2D("desiredICP", worldFrame, registry);
   private final YoFrameVector2D yoDesiredICPVelocity = new YoFrameVector2D("desiredICPVelocity", worldFrame, registry);
   private final YoFramePoint3D yoDesiredCoMPosition = new YoFramePoint3D("desiredCoMPosition", worldFrame, registry);
   private final YoFrameVector3D yoDesiredCoMVelocity = new YoFrameVector3D("desiredCoMVelocity", worldFrame, registry);
   private final YoFramePoint2D yoFinalDesiredICP = new YoFramePoint2D("finalDesiredICP", worldFrame, registry);
   private final YoFramePoint3D yoFinalDesiredCoM = new YoFramePoint3D("finalDesiredCoM", worldFrame, registry);

   private final SwingSpeedUpCalculator swingSpeedUpCalculator = new SwingSpeedUpCalculator();

   /** CoP position according to the ICP planner */
   private final YoFramePoint2D yoPerfectCoP = new YoFramePoint2D("perfectCoP", worldFrame, registry);
   private final YoFrameVector2D yoPerfectCoPVelocity = new YoFrameVector2D("perfectCoPVelocity", worldFrame, registry);
   /** CMP position according to the ICP planner */
   private final YoFramePoint2D yoPerfectCMP = new YoFramePoint2D("perfectCMP", worldFrame, registry);

   private final YoBoolean useMomentumRecoveryModeForBalance = new YoBoolean("useMomentumRecoveryModeForBalance", registry);

   private final YoDouble yoTime;

   private final ReferenceFrame centerOfMassFrame;

   private final FramePoint3D centerOfMassPosition = new FramePoint3D();
   private final FramePoint2D centerOfMassPosition2d = new FramePoint2D();

   private final FramePoint2D capturePoint2d = new FramePoint2D();
   private final FramePoint2D desiredCapturePoint2d = new FramePoint2D();
   private final FramePoint2D desiredCoM2d = new FramePoint2D();
   private final FrameVector2D desiredCapturePointVelocity2d = new FrameVector2D();
   private final FramePoint2D perfectCoP2d = new FramePoint2D();

   private final YoBoolean blendICPTrajectories = new YoBoolean("blendICPTrajectories", registry);

   private final FramePoint2D adjustedDesiredCapturePoint2d = new FramePoint2D();
   private final YoFramePoint2D yoAdjustedDesiredCapturePoint = new YoFramePoint2D("adjustedDesiredICP", worldFrame, registry);

   private final FrameVector2D icpError2d = new FrameVector2D();

   private final ConvexPolygonScaler convexPolygonShrinker = new ConvexPolygonScaler();
   private final FrameConvexPolygon2D shrunkSupportPolygon = new FrameConvexPolygon2D();

   private StepConstraintRegionHandler stepConstraintRegionHandler;

   private final YoDouble safeDistanceFromSupportEdgesToStopCancelICPPlan = new YoDouble("safeDistanceFromSupportEdgesToStopCancelICPPlan", registry);
   private final YoDouble distanceToShrinkSupportPolygonWhenHoldingCurrent = new YoDouble("distanceToShrinkSupportPolygonWhenHoldingCurrent", registry);

   private final YoBoolean holdICPToCurrentCoMLocationInNextDoubleSupport = new YoBoolean("holdICPToCurrentCoMLocationInNextDoubleSupport", registry);

   private final YoDouble normalizedICPError = new YoDouble("normalizedICPError", registry);
   private final DoubleProvider maxICPErrorBeforeSingleSupportForwardX;
   private final DoubleProvider maxICPErrorBeforeSingleSupportBackwardX;
   private final DoubleProvider maxICPErrorBeforeSingleSupportInnerY;
   private final DoubleProvider maxICPErrorBeforeSingleSupportOuterY;

   private final DoubleProvider icpDistanceOutsideSupportForStep = new DoubleParameter("icpDistanceOutsideSupportForStep", registry, 0.03);
   private final DoubleProvider ellipticICPErrorForMomentumRecovery = new DoubleParameter("ellipticICPErrorForMomentumRecovery", registry, 2.0);

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

   private final YoBoolean icpPlannerDone = new YoBoolean("ICPPlannerDone", registry);
   private final ExecutionTimer plannerTimer = new ExecutionTimer("icpPlannerTimer", registry);

   private boolean initializeOnStateChange = false;
   private boolean minimizeAngularMomentumRateZ = false;
   private double timeRemainingInSwing = Double.NaN;
   private RobotSide supportSide;
   private final YoFrameVector2D residualICPErrorForStepAdjustment = new YoFrameVector2D("residualICPErrorForStepAdjustment", worldFrame, registry);
   private final FixedFramePoint2DBasics desiredCMP = new FramePoint2D();
   private final FixedFrameVector3DBasics effectiveICPAdjustment = new FrameVector3D();
   private final SimpleFootstep currentFootstep = new SimpleFootstep();
   private final SideDependentList<PlaneContactStateCommand> contactStateCommands = new SideDependentList<>(new PlaneContactStateCommand(),
                                                                                                            new PlaneContactStateCommand());
   private final SideDependentList<? extends ReferenceFrame> soleFrames;

   private final List<Footstep> footsteps = new ArrayList<>();
   private final List<FootstepTiming> footstepTimings = new ArrayList<>();

   private final YoBoolean inSingleSupport = new YoBoolean("InSingleSupport", registry);
   private final YoDouble currentStateDuration = new YoDouble("CurrentStateDuration", registry);
   private final YoDouble totalStateDuration = new YoDouble("totalStateDuration", registry);
   private final FootstepTiming currentTiming = new FootstepTiming();
   private final YoDouble timeInSupportSequence = new YoDouble("TimeInSupportSequence", registry);
   private final CoPTrajectoryGeneratorState copTrajectoryState;
   private final WalkingCoPTrajectoryGenerator copTrajectory;
   private final FlamingoCoPTrajectoryGenerator flamingoCopTrajectory;
   private final CoMTrajectoryPlanner comTrajectoryPlanner;

   public BalanceManager(HighLevelHumanoidControllerToolbox controllerToolbox,
                         WalkingControllerParameters walkingControllerParameters,
                         CoPTrajectoryParameters copTrajectoryParameters,
                         YoRegistry parentRegistry)
   {
      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();

      YoGraphicsListRegistry yoGraphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();

      double gravityZ = controllerToolbox.getGravityZ();
      double totalMass = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());

      this.controllerToolbox = controllerToolbox;
      yoTime = controllerToolbox.getYoTime();

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

      safeDistanceFromSupportEdgesToStopCancelICPPlan.set(0.05);
      distanceToShrinkSupportPolygonWhenHoldingCurrent.set(0.08);

      maxICPErrorBeforeSingleSupportForwardX = new DoubleParameter("maxICPErrorBeforeSingleSupportForwardX", registry, walkingControllerParameters.getMaxICPErrorBeforeSingleSupportForwardX());
      maxICPErrorBeforeSingleSupportBackwardX = new DoubleParameter("maxICPErrorBeforeSingleSupportBackwardX", registry, walkingControllerParameters.getMaxICPErrorBeforeSingleSupportBackwardX());
      maxICPErrorBeforeSingleSupportInnerY = new DoubleParameter("maxICPErrorBeforeSingleSupportInnerY", registry, walkingControllerParameters.getMaxICPErrorBeforeSingleSupportInnerY());
      maxICPErrorBeforeSingleSupportOuterY = new DoubleParameter("maxICPErrorBeforeSingleSupportOuterY", registry, walkingControllerParameters.getMaxICPErrorBeforeSingleSupportOuterY());

      double pelvisTranslationICPSupportPolygonSafeMargin = walkingControllerParameters.getPelvisTranslationICPSupportPolygonSafeMargin();
      pelvisICPBasedTranslationManager = new PelvisICPBasedTranslationManager(controllerToolbox, pelvisTranslationICPSupportPolygonSafeMargin, bipedSupportPolygons, registry);

      FrameConvexPolygon2D defaultSupportPolygon = controllerToolbox.getDefaultFootPolygons().get(RobotSide.LEFT);
      soleFrames = controllerToolbox.getReferenceFrames().getSoleFrames();
      registry.addChild(copTrajectoryParameters.getRegistry());
      comTrajectoryPlanner = new CoMTrajectoryPlanner(controllerToolbox.getGravityZ(), controllerToolbox.getOmega0Provider(), registry);
      comTrajectoryPlanner.setComContinuityCalculator(new CoMContinuousContinuityCalculator(controllerToolbox.getGravityZ(), controllerToolbox.getOmega0Provider(), registry));
      copTrajectoryState = new CoPTrajectoryGeneratorState(registry);
      copTrajectoryState.registerStateToSave(copTrajectoryParameters);
      copTrajectory = new WalkingCoPTrajectoryGenerator(copTrajectoryParameters, defaultSupportPolygon, registry);
      copTrajectory.registerState(copTrajectoryState);
      flamingoCopTrajectory = new FlamingoCoPTrajectoryGenerator(copTrajectoryParameters, registry);
      flamingoCopTrajectory.registerState(copTrajectoryState);

      pushRecoveryControlModule = new PushRecoveryControlModule(bipedSupportPolygons, controllerToolbox, walkingControllerParameters, registry);

      stepAdjustmentController = new StepAdjustmentController(walkingControllerParameters,
                                                              controllerToolbox.getReferenceFrames().getSoleZUpFrames(),
                                                              bipedSupportPolygons,
                                                              icpControlPolygons,
                                                              controllerToolbox.getContactableFeet(),
                                                              controllerToolbox.getControlDT(),
                                                              registry,
                                                              yoGraphicsListRegistry);

      String graphicListName = getClass().getSimpleName();

      if (yoGraphicsListRegistry != null)
      {
         comTrajectoryPlanner.setCornerPointViewer(new CornerPointViewer(true, false, registry, yoGraphicsListRegistry));
//         copTrajectory.setWaypointViewer(new WaypointViewer(registry, yoGraphicsListRegistry));

         YoGraphicPosition desiredCapturePointViz = new YoGraphicPosition("Desired Capture Point", yoDesiredCapturePoint, 0.01, Yellow(), GraphicType.BALL_WITH_ROTATED_CROSS);
         YoGraphicPosition finalDesiredCapturePointViz = new YoGraphicPosition("Final Desired Capture Point", yoFinalDesiredICP, 0.01, Beige(), GraphicType.BALL_WITH_ROTATED_CROSS);
         YoGraphicPosition finalDesiredCoMViz = new YoGraphicPosition("Final Desired CoM", yoFinalDesiredCoM, 0.01, Black(), GraphicType.BALL_WITH_ROTATED_CROSS);
         YoGraphicPosition perfectCMPViz = new YoGraphicPosition("Perfect CMP", yoPerfectCMP, 0.002, BlueViolet());
         YoGraphicPosition perfectCoPViz = new YoGraphicPosition("Perfect CoP", yoPerfectCoP, 0.002, DarkViolet(), GraphicType.BALL_WITH_CROSS);

         YoGraphicPosition adjustedDesiredCapturePointViz = new YoGraphicPosition("Adjusted Desired Capture Point", yoAdjustedDesiredCapturePoint, 0.005, Yellow(), GraphicType.DIAMOND);
         yoGraphicsListRegistry.registerArtifact(graphicListName, adjustedDesiredCapturePointViz.createArtifact());

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
      yoDesiredCapturePoint.setToNaN();
      yoFinalDesiredICP.setToNaN();
      yoPerfectCMP.setToNaN();
      yoPerfectCoP.setToNaN();
      yoPerfectCoPVelocity.setToNaN();

      parentRegistry.addChild(registry);
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

   public void setPlanarRegionStepConstraintHandler(StepConstraintRegionHandler planarRegionStepConstraint)
   {
      this.stepConstraintRegionHandler = planarRegionStepConstraint;
   }

   public boolean checkAndUpdateFootstep(Footstep footstep)
   {
      return pushRecoveryControlModule.checkAndUpdateFootstep(getTimeRemainingInCurrentState(), footstep);
   }

   public boolean checkAndUpdateStepAdjustment(Footstep footstep)
   {
      boolean usingStepAdjustment = stepAdjustmentController.useStepAdjustment();

      if (!usingStepAdjustment || initializeOnStateChange)
      {
         return false;
      }

      double omega0 = linearMomentumRateControlModuleInput.getOmega0();
      FramePoint2D desiredCapturePoint = linearMomentumRateControlModuleInput.getDesiredCapturePoint();

      controllerToolbox.getCapturePoint(capturePoint2d);
      icpControlPlane.setOmega0(omega0);
      icpControlPolygons.updateUsingContactStateCommand(contactStateCommands);

      if (!Double.isNaN(timeRemainingInSwing) && timeRemainingInSwing > 0.0)
         stepAdjustmentController.submitRemainingTimeInSwingUnderDisturbance(timeRemainingInSwing);
      if (stepConstraintRegionHandler != null && stepConstraintRegionHandler.hasNewStepConstraintRegion())
         stepAdjustmentController.setStepConstraintRegion(stepConstraintRegionHandler.pollHasNewStepConstraintRegion());

      stepAdjustmentController.compute(yoTime.getDoubleValue(), desiredCapturePoint, capturePoint2d, residualICPErrorForStepAdjustment, omega0);
      boolean footstepWasAdjusted = stepAdjustmentController.wasFootstepAdjusted();
      footstep.setPose(stepAdjustmentController.getFootstepSolution());
      return footstepWasAdjusted;
   }

   public void clearICPPlan()
   {
      copTrajectoryState.clear();
      footsteps.clear();
      footstepTimings.clear();
   }

   public void setICPPlanSupportSide(RobotSide supportSide)
   {
      this.supportSide = supportSide;
   }

   private void updatePolygons()
   {
      bipedSupportPolygons.updateUsingContactStateCommand(contactStateCommands);
   }

   public void compute(RobotSide supportLeg, FeedbackControlCommand<?> heightControlCommand, boolean keepCoPInsideSupportPolygon,
                       boolean controlHeightWithMomentum)
   {
      desiredCapturePoint2d.set(comTrajectoryPlanner.getDesiredDCMPosition());
      desiredCapturePointVelocity2d.set(comTrajectoryPlanner.getDesiredDCMVelocity());
      if (!icpVelocityReductionFactor.isNaN())
         desiredCapturePointVelocity2d.scale(icpVelocityReductionFactor.getValue());
      perfectCoP2d.set(comTrajectoryPlanner.getDesiredECMPPosition());
      desiredCoM2d.set(comTrajectoryPlanner.getDesiredCoMPosition());
      yoDesiredCoMVelocity.set(comTrajectoryPlanner.getDesiredCoMVelocity());

      capturePoint2d.setIncludingFrame(controllerToolbox.getCapturePoint());
      pelvisICPBasedTranslationManager.compute(supportLeg);
      pelvisICPBasedTranslationManager.addICPOffset(desiredCapturePoint2d, desiredCoM2d, perfectCoP2d);

      double omega0 = controllerToolbox.getOmega0();
      if (Double.isNaN(omega0))
         throw new RuntimeException("omega0 is NaN");

      if (supportLeg == null)
         pushRecoveryControlModule.updateForDoubleSupport(desiredCapturePoint2d, capturePoint2d, omega0);
      else
         pushRecoveryControlModule.updateForSingleSupport(desiredCapturePoint2d, capturePoint2d, omega0);

      // --- compute adjusted desired capture point
      controllerToolbox.getAdjustedDesiredCapturePoint(desiredCapturePoint2d, adjustedDesiredCapturePoint2d);
      yoAdjustedDesiredCapturePoint.set(adjustedDesiredCapturePoint2d);
      desiredCapturePoint2d.setIncludingFrame(adjustedDesiredCapturePoint2d);
      // ---

      if (precomputedICPPlanner != null)
      {
         if (blendICPTrajectories.getBooleanValue())
         {
            precomputedICPPlanner.computeAndBlend(yoTime.getDoubleValue(), desiredCapturePoint2d, desiredCapturePointVelocity2d, perfectCoP2d);
         }
         else
         {
            precomputedICPPlanner.compute(yoTime.getDoubleValue(), desiredCapturePoint2d, desiredCapturePointVelocity2d, perfectCoP2d);
         }
      }

      yoDesiredCapturePoint.set(desiredCapturePoint2d);
      yoDesiredICPVelocity.set(desiredCapturePointVelocity2d);
      yoDesiredCoMPosition.set(desiredCoM2d);
      yoPerfectCoP.set(perfectCoP2d);
      yoPerfectCoPVelocity.set(comTrajectoryPlanner.getDesiredVRPVelocity());

      CapturePointTools.computeCentroidalMomentumPivot(yoDesiredCapturePoint, yoDesiredICPVelocity, omega0, yoPerfectCMP);

      for (RobotSide robotSide : RobotSide.values)
      {
         YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);
         contactState.getPlaneContactStateCommand(contactStateCommands.get(robotSide));
      }

      if (heightControlCommand.getCommandType() == ControllerCoreCommandType.POINT)
      {
         linearMomentumRateControlModuleInput.setUsePelvisHeightCommand(true);
         linearMomentumRateControlModuleInput.setPelvisHeightControlCommand((PointFeedbackControlCommand) heightControlCommand);
      }
      else if (heightControlCommand.getCommandType() == ControllerCoreCommandType.MOMENTUM)
      {
         linearMomentumRateControlModuleInput.setUsePelvisHeightCommand(false);
         linearMomentumRateControlModuleInput.setCenterOfMassHeightControlCommand((CenterOfMassFeedbackControlCommand) heightControlCommand);
      }
      else
      {
         throw new IllegalArgumentException("Invalid height control type.");
      }
      linearMomentumRateControlModuleInput.setInitializeOnStateChange(initializeOnStateChange);
      linearMomentumRateControlModuleInput.setKeepCoPInsideSupportPolygon(keepCoPInsideSupportPolygon);
      linearMomentumRateControlModuleInput.setControlHeightWithMomentum(controlHeightWithMomentum);
      linearMomentumRateControlModuleInput.setOmega0(omega0);
      linearMomentumRateControlModuleInput.setUseMomentumRecoveryMode(useMomentumRecoveryModeForBalance.getBooleanValue());
      linearMomentumRateControlModuleInput.setDesiredCapturePoint(yoDesiredCapturePoint);
      linearMomentumRateControlModuleInput.setDesiredCapturePointVelocity(yoDesiredICPVelocity);
      linearMomentumRateControlModuleInput.setPerfectCMP(yoPerfectCMP);
      linearMomentumRateControlModuleInput.setPerfectCoP(yoPerfectCoP);
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

   public void adjustFootstep(Footstep footstep)
   {
      copTrajectoryState.getFootstep(0).set(footstep);
   }

   public void computeICPPlan()
   {
      computeICPPlanInternal(copTrajectory);
   }

   public void computeFlamingoStateICPPlan()
   {
      computeICPPlanInternal(flamingoCopTrajectory);
   }

   private void computeICPPlanInternal(CoPTrajectoryGenerator copTrajectory)
   {
      plannerTimer.startMeasurement();

      // update online to account for foot slip
      for (RobotSide robotSide : RobotSide.values)
      {
         if (controllerToolbox.getFootContactState(robotSide).inContact())
            copTrajectoryState.initializeStance(robotSide, bipedSupportPolygons.getFootPolygonsInSoleZUpFrame().get(robotSide), soleFrames.get(robotSide));
      }
      copTrajectory.compute(copTrajectoryState);

      comTrajectoryPlanner.solveForTrajectory(copTrajectory.getContactStateProviders());
      comTrajectoryPlanner.compute(totalStateDuration.getDoubleValue());

      yoFinalDesiredCoM.set(comTrajectoryPlanner.getDesiredCoMPosition());
      yoFinalDesiredICP.set(comTrajectoryPlanner.getDesiredDCMPosition());

      comTrajectoryPlanner.compute(timeInSupportSequence.getDoubleValue());

      if (footstepTimings.isEmpty())
      {
         yoFinalDesiredICP.setToNaN();
         yoFinalDesiredCoM.setToNaN();
      }

      // If this condition is false we are experiencing a late touchdown or a delayed liftoff. Do not advance the time in support sequence!
      if (footsteps.isEmpty() || !icpPlannerDone.getValue())
         timeInSupportSequence.add(controllerToolbox.getControlDT());

      icpPlannerDone.set(timeInSupportSequence.getValue() >= currentStateDuration.getValue());
      decayDesiredICPVelocity();

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

      if (inSingleSupport.getValue() || !icpPlannerDone.getValue())
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

   public void packFootstepForRecoveringFromDisturbance(RobotSide swingSide, double swingTimeRemaining, Footstep footstepToPack)
   {
      pushRecoveryControlModule.packFootstepForRecoveringFromDisturbance(swingSide, swingTimeRemaining, footstepToPack);
   }

   public void disablePelvisXYControl()
   {
      pelvisICPBasedTranslationManager.disable();
   }

   public void disablePushRecovery()
   {
      pushRecoveryControlModule.setIsEnabled(false);
   }

   public void enablePelvisXYControl()
   {
      pelvisICPBasedTranslationManager.enable();
   }

   public void enablePushRecovery()
   {
      pushRecoveryControlModule.setIsEnabled(true);
   }

   public double estimateTimeRemainingForSwingUnderDisturbance()
   {
      double stateDuration = currentTiming.getStepTime();
      double timeRemainingInCurrentState = timeInSupportSequence.getDoubleValue() - stateDuration;
      if (icpPlannerDone.getBooleanValue())
         return 0.0;

      controllerToolbox.getCapturePoint(capturePoint2d);
      double deltaTimeToBeAccounted = swingSpeedUpCalculator.estimateDeltaTimeBetweenDesiredICPAndActualICP(yoDesiredCapturePoint,
                                                                                                            yoPerfectCMP,
                                                                                                            yoFinalDesiredICP,
                                                                                                            capturePoint2d,
                                                                                                            controllerToolbox.getOmega0());

      if (Double.isNaN(deltaTimeToBeAccounted))
         return 0.0;

      double estimatedTimeRemaining = timeRemainingInCurrentState - deltaTimeToBeAccounted;
      estimatedTimeRemaining = MathTools.clamp(estimatedTimeRemaining, 0.0, Double.POSITIVE_INFINITY);

      return estimatedTimeRemaining;
   }

   public void freezePelvisXYControl()
   {
      pelvisICPBasedTranslationManager.freeze();
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

   public FrameVector2DReadOnly getDesiredICPVelocity()
   {
      return yoDesiredICPVelocity;
   }

   public FramePoint3DReadOnly getFinalDesiredCoMPosition()
   {
      return yoFinalDesiredCoM;
   }

   public FrameVector3DReadOnly getDesiredCoMVelocity()
   {
      return yoDesiredCoMVelocity;
   }

   public FramePoint3DReadOnly getFirstExitCMPForToeOff(boolean isInTransfer)
   {
      if (isInTransfer)
         return copTrajectory.getContactStateProviders().get(0).getCopStartPosition();
      else
         return copTrajectory.getContactStateProviders().get(4).getCopEndPosition();
   }

   public double getTimeRemainingInCurrentState()
   {
      if (footstepTimings.isEmpty())
         return 0.0;
      return currentTiming.getStepTime() - timeInSupportSequence.getValue();
   }

   public void goHome()
   {
      if (pelvisICPBasedTranslationManager.isEnabled())
         pelvisICPBasedTranslationManager.goToHome();
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
      yoFinalDesiredCoM.setToNaN();
      yoDesiredCapturePoint.set(controllerToolbox.getCapturePoint());
      yoDesiredCoMPosition.setFromReferenceFrame(controllerToolbox.getCenterOfMassFrame());
      yoDesiredCoMVelocity.setToZero();

      yoPerfectCoP.set(bipedSupportPolygons.getSupportPolygonInWorld().getCentroid());
      copTrajectoryState.setInitialCoP(bipedSupportPolygons.getSupportPolygonInWorld().getCentroid());
      copTrajectoryState.initializeStance(bipedSupportPolygons.getFootPolygonsInSoleZUpFrame(), soleFrames);
      comTrajectoryPlanner.setInitialCenterOfMassState(yoDesiredCoMPosition, yoDesiredCoMVelocity);
      timeInSupportSequence.set(0.0);
      inSingleSupport.set(false);
      currentStateDuration.set(Double.NaN);
      totalStateDuration.set(Double.NaN);

      comTrajectoryPlanner.setMaintainInitialCoMVelocityContinuity(false);

      initializeOnStateChange = true;
      stepAdjustmentController.reset();
   }

   public void prepareForDoubleSupportPushRecovery()
   {
      pushRecoveryControlModule.initializeParametersForDoubleSupportPushRecovery();
   }

   public void initializeICPPlanForSingleSupport()
   {
      inSingleSupport.set(true);
      currentTiming.set(footstepTimings.get(0));
      currentFootstep.set(footsteps.get(0));
      stepAdjustmentController.reset();
      stepAdjustmentController.setFootstepToAdjust(currentFootstep, currentTiming.getSwingTime(), currentTiming.getTransferTime());
      stepAdjustmentController.initialize(yoTime.getDoubleValue(), supportSide);

      timeInSupportSequence.set(currentTiming.getTransferTime());
      currentStateDuration.set(currentTiming.getStepTime());
      totalStateDuration.set(currentTiming.getStepTime());

      comTrajectoryPlanner.setMaintainInitialCoMVelocityContinuity(true);
      initializeOnStateChange = true;
      icpPlannerDone.set(false);
   }

   public void initializeICPPlanForStanding()
   {
      if (holdICPToCurrentCoMLocationInNextDoubleSupport.getBooleanValue())
      {
         requestICPPlannerToHoldCurrentCoM();
         holdICPToCurrentCoMLocationInNextDoubleSupport.set(false);
      }
      copTrajectoryState.setInitialCoP(yoPerfectCoP);
      copTrajectoryState.initializeStance(bipedSupportPolygons.getFootPolygonsInSoleZUpFrame(), soleFrames);
      comTrajectoryPlanner.setInitialCenterOfMassState(yoDesiredCoMPosition, yoDesiredCoMVelocity);

      timeInSupportSequence.set(0.0);
      currentStateDuration.set(Double.POSITIVE_INFINITY);
      totalStateDuration.set(Double.POSITIVE_INFINITY);

      inSingleSupport.set(false);
      initializeOnStateChange = true;
      comTrajectoryPlanner.setMaintainInitialCoMVelocityContinuity(true);
      stepAdjustmentController.reset();

      icpPlannerDone.set(false);
   }

   public void initializeICPPlanForTransferToStanding()
   {
      if (holdICPToCurrentCoMLocationInNextDoubleSupport.getBooleanValue())
      {
         requestICPPlannerToHoldCurrentCoM();
         holdICPToCurrentCoMLocationInNextDoubleSupport.set(false);
      }
      copTrajectoryState.setInitialCoP(yoPerfectCoP);
      copTrajectoryState.initializeStance(bipedSupportPolygons.getFootPolygonsInSoleZUpFrame(), soleFrames);
      comTrajectoryPlanner.setInitialCenterOfMassState(yoDesiredCoMPosition, yoDesiredCoMVelocity);

      timeInSupportSequence.set(0.0);
      currentStateDuration.set(copTrajectoryState.getFinalTransferDuration());
      totalStateDuration.set(copTrajectoryState.getFinalTransferDuration());

      inSingleSupport.set(false);
      initializeOnStateChange = true;
      comTrajectoryPlanner.setMaintainInitialCoMVelocityContinuity(true);
      stepAdjustmentController.reset();

      icpPlannerDone.set(false);
   }

   public void initializeICPPlanForTransfer()
   {
      if (holdICPToCurrentCoMLocationInNextDoubleSupport.getBooleanValue())
      {
         requestICPPlannerToHoldCurrentCoM();
         holdICPToCurrentCoMLocationInNextDoubleSupport.set(false);
      }

      stepAdjustmentController.reset();

      copTrajectoryState.setInitialCoP(yoPerfectCoP);
      copTrajectoryState.initializeStance(bipedSupportPolygons.getFootPolygonsInSoleZUpFrame(), soleFrames);
      comTrajectoryPlanner.setInitialCenterOfMassState(yoDesiredCoMPosition, yoDesiredCoMVelocity);

      currentTiming.set(footstepTimings.get(0));
      timeInSupportSequence.set(0.0);
      currentStateDuration.set(currentTiming.getTransferTime());
      totalStateDuration.set(currentTiming.getStepTime());

      inSingleSupport.set(false);
      comTrajectoryPlanner.setMaintainInitialCoMVelocityContinuity(true);

      initializeOnStateChange = true;
      icpPlannerDone.set(false);
   }

   public void computeNormalizedEllipticICPError(RobotSide transferToSide)
   {
      getICPError(icpError2d);
      ReferenceFrame leadingSoleZUpFrame = controllerToolbox.getReferenceFrames().getSoleZUpFrame(transferToSide);
      icpError2d.changeFrame(leadingSoleZUpFrame);
      boolean isICPErrorToTheInside = transferToSide == RobotSide.RIGHT ? icpError2d.getY() > 0.0 : icpError2d.getY() < 0.0;
      double maxICPErrorBeforeSingleSupportX = icpError2d.getX() > 0.0 ? maxICPErrorBeforeSingleSupportForwardX.getValue() : maxICPErrorBeforeSingleSupportBackwardX.getValue();
      double maxICPErrorBeforeSingleSupportY = isICPErrorToTheInside ? maxICPErrorBeforeSingleSupportInnerY.getValue() : maxICPErrorBeforeSingleSupportOuterY.getValue();
      normalizedICPError.set(MathTools.square(icpError2d.getX() / maxICPErrorBeforeSingleSupportX) + MathTools.square(icpError2d.getY() / maxICPErrorBeforeSingleSupportY));
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
      return icpPlannerDone.getValue();
   }

   public boolean isPushRecoveryEnabled()
   {
      return pushRecoveryControlModule.isEnabled();
   }

   public boolean isRecovering()
   {
      return pushRecoveryControlModule.isRecovering();
   }

   public boolean isRecoveringFromDoubleSupportFall()
   {
      return pushRecoveryControlModule.isEnabled() && pushRecoveryControlModule.isRecoveringFromDoubleSupportFall();
   }

   public boolean isRecoveryImpossible()
   {
      return pushRecoveryControlModule.isCaptureRegionEmpty();
   }

   public boolean isRobotBackToSafeState()
   {
      return pushRecoveryControlModule.isRobotBackToSafeState();
   }

   public RobotSide isRobotFallingFromDoubleSupport()
   {
      return pushRecoveryControlModule.isRobotFallingFromDoubleSupport();
   }

   public void resetPushRecovery()
   {
      pushRecoveryControlModule.reset();
   }

   public void requestICPPlannerToHoldCurrentCoMInNextDoubleSupport()
   {
      holdICPToCurrentCoMLocationInNextDoubleSupport.set(true);
   }

   public void requestICPPlannerToHoldCurrentCoM()
   {
      centerOfMassPosition.setToZero(centerOfMassFrame);

      FrameConvexPolygon2DReadOnly supportPolygonInMidFeetZUp = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();
      convexPolygonShrinker.scaleConvexPolygon(supportPolygonInMidFeetZUp, distanceToShrinkSupportPolygonWhenHoldingCurrent.getDoubleValue(), shrunkSupportPolygon);

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
      this.timeRemainingInSwing = timeRemainingInSwing;
   }

   @Deprecated
   public FrameVector3DReadOnly getEffectiveICPAdjustment()
   {
      return effectiveICPAdjustment;
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
      residualICPErrorForStepAdjustment.set(output.getResidualICPErrorForStepAdjustment());
   }

   public TaskspaceTrajectoryStatusMessage pollPelvisXYTranslationStatusToReport()
   {
      return pelvisICPBasedTranslationManager.pollStatusToReport();
   }
}
