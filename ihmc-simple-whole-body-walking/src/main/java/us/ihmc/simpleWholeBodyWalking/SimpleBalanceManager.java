package us.ihmc.simpleWholeBodyWalking;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.Beige;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.BlueViolet;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.DarkViolet;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Yellow;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commonWalkingControlModules.capturePoint.LinearMomentumRateControlModuleInput;
import us.ihmc.commonWalkingControlModules.capturePoint.LinearMomentumRateControlModuleOutput;
import us.ihmc.commonWalkingControlModules.capturePoint.lqrControl.LQRMomentumController;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.SmoothCMPBasedICPPlanner;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisICPBasedTranslationManager;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepShiftFractions;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector2D;
import us.ihmc.simpleWholeBodyWalking.SimpleBipedCoMTrajectoryPlanner;


public class SimpleBalanceManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BipedSupportPolygons bipedSupportPolygons;

   private final LinearMomentumRateControlModuleInput linearMomentumRateControlModuleInput = new LinearMomentumRateControlModuleInput();
   private final LQRMomentumController lqrMomentumController;

   private final PelvisICPBasedTranslationManager pelvisICPBasedTranslationManager;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   private final YoFramePoint2D yoDesiredCapturePoint = new YoFramePoint2D("desiredICP", worldFrame, registry);
   private final YoFrameVector2D yoDesiredICPVelocity = new YoFrameVector2D("desiredICPVelocity", worldFrame, registry);
   private final YoFramePoint3D yoDesiredCoMPosition = new YoFramePoint3D("desiredCoMPosition", worldFrame, registry);
   private final YoFrameVector2D yoDesiredCoMVelocity = new YoFrameVector2D("desiredCoMVelocity", worldFrame, registry);
   private final YoFramePoint2D yoFinalDesiredICP = new YoFramePoint2D("finalDesiredICP", worldFrame, registry);
   private final YoFramePoint3D yoFinalDesiredCoM = new YoFramePoint3D("finalDesiredCoM", worldFrame, registry);

   /** CoP position according to the ICP planner */
   private final YoFramePoint2D yoPerfectCoP = new YoFramePoint2D("perfectCoP", worldFrame, registry);
   /** CMP position according to the ICP planner */
   private final YoFramePoint2D yoPerfectCMP = new YoFramePoint2D("perfectCMP", worldFrame, registry);

   private final YoDouble yoTime;

   private final ReferenceFrame centerOfMassFrame;

   private final FramePoint3D centerOfMassPosition = new FramePoint3D();
   private final FramePoint2D centerOfMassPosition2d = new FramePoint2D();

   private final FramePoint2D capturePoint2d = new FramePoint2D();
   private final FramePoint3D tempCapturePoint = new FramePoint3D();
   private final FramePoint2D desiredCapturePoint2d = new FramePoint2D();
   private final FrameVector2D desiredCapturePointVelocity2d = new FrameVector2D();
   private final FramePoint2D perfectCoP2d = new FramePoint2D();

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

   private final CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus();

   private final SmoothCMPBasedICPPlanner smoothCMPPlanner;
   private final YoBoolean icpPlannerDone = new YoBoolean("ICPPlannerDone", registry);
   private final SimpleBipedCoMTrajectoryPlanner comPlanner;

   private boolean initializeForStanding = false;
   private boolean initializeForSingleSupport = false;
   private boolean initializeForTransfer = false;
   private boolean footstepWasAdjusted = false;
   private boolean usingStepAdjustment = false;
   private double finalTransferDuration;
   private double timeRemainingInSwing = Double.NaN;
   private RobotSide supportSide;
   private RobotSide transferToSide;
   private final FixedFramePose3DBasics footstepSolution = new FramePose3D();
   private final FixedFramePoint2DBasics desiredCMP = new FramePoint2D();
   private final FixedFrameVector3DBasics effectiveICPAdjustment = new FrameVector3D();
   private final RecyclingArrayList<Footstep> footsteps = new RecyclingArrayList<>(Footstep.class);
   private final RecyclingArrayList<FootstepTiming> footstepTimings = new RecyclingArrayList<>(FootstepTiming.class);
   private final SideDependentList<PlaneContactStateCommand> contactStateCommands = new SideDependentList<>(new PlaneContactStateCommand(),
                                                                                                            new PlaneContactStateCommand());

   public SimpleBalanceManager(HighLevelHumanoidControllerToolbox controllerToolbox, WalkingControllerParameters walkingControllerParameters,
                               ICPWithTimeFreezingPlannerParameters icpPlannerParameters,
                               YoVariableRegistry parentRegistry)
   {
      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();

      YoGraphicsListRegistry yoGraphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();
      SideDependentList<ContactableFoot> contactableFeet = controllerToolbox.getContactableFeet();

      this.controllerToolbox = controllerToolbox;
      yoTime = controllerToolbox.getYoTime();

      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();

      bipedSupportPolygons = controllerToolbox.getBipedSupportPolygons();

      SideDependentList<MovingReferenceFrame> soleZUpFrames = referenceFrames.getSoleZUpFrames();
      smoothCMPPlanner = new SmoothCMPBasedICPPlanner(fullRobotModel, bipedSupportPolygons, soleZUpFrames, contactableFeet, null, yoTime,
                                                      registry, yoGraphicsListRegistry, controllerToolbox.getGravityZ(), icpPlannerParameters);
      smoothCMPPlanner.setDefaultPhaseTimes(walkingControllerParameters.getDefaultSwingTime(), walkingControllerParameters.getDefaultTransferTime());


      this.smoothCMPPlanner.setOmega0(controllerToolbox.getOmega0());
      this.smoothCMPPlanner.setFinalTransferDuration(walkingControllerParameters.getDefaultTransferTime());

      comPlanner = new SimpleBipedCoMTrajectoryPlanner(soleZUpFrames, controllerToolbox.getGravityZ(), 
                                                       0.75, controllerToolbox.getOmega0Provider(), 
                                                       registry);
      lqrMomentumController = new LQRMomentumController(controllerToolbox.getOmega0Provider(), registry);
      
      distanceToShrinkSupportPolygonWhenHoldingCurrent.set(0.08);

      maxICPErrorBeforeSingleSupportForwardX = new DoubleParameter("maxICPErrorBeforeSingleSupportForwardX", registry, walkingControllerParameters.getMaxICPErrorBeforeSingleSupportForwardX());
      maxICPErrorBeforeSingleSupportBackwardX = new DoubleParameter("maxICPErrorBeforeSingleSupportBackwardX", registry, walkingControllerParameters.getMaxICPErrorBeforeSingleSupportBackwardX());
      maxICPErrorBeforeSingleSupportInnerY = new DoubleParameter("maxICPErrorBeforeSingleSupportInnerY", registry, walkingControllerParameters.getMaxICPErrorBeforeSingleSupportInnerY());
      maxICPErrorBeforeSingleSupportOuterY = new DoubleParameter("maxICPErrorBeforeSingleSupportOuterY", registry, walkingControllerParameters.getMaxICPErrorBeforeSingleSupportOuterY());

      double pelvisTranslationICPSupportPolygonSafeMargin = walkingControllerParameters.getPelvisTranslationICPSupportPolygonSafeMargin();
      pelvisICPBasedTranslationManager = new PelvisICPBasedTranslationManager(controllerToolbox, pelvisTranslationICPSupportPolygonSafeMargin, bipedSupportPolygons, registry);

      String graphicListName = getClass().getSimpleName();

      if (yoGraphicsListRegistry != null)
      {
         YoGraphicPosition desiredCapturePointViz = new YoGraphicPosition("Desired Capture Point", yoDesiredCapturePoint, 0.01, Yellow(), GraphicType.BALL_WITH_ROTATED_CROSS);
         YoGraphicPosition finalDesiredCapturePointViz = new YoGraphicPosition("Final Desired Capture Point", yoFinalDesiredICP, 0.01, Beige(), GraphicType.BALL_WITH_ROTATED_CROSS);
         YoGraphicPosition perfectCMPViz = new YoGraphicPosition("Perfect CMP", yoPerfectCMP, 0.002, BlueViolet());
         YoGraphicPosition perfectCoPViz = new YoGraphicPosition("Perfect CoP", yoPerfectCoP, 0.002, DarkViolet(), GraphicType.BALL_WITH_CROSS);


         yoGraphicsListRegistry.registerArtifact(graphicListName, desiredCapturePointViz.createArtifact());
         yoGraphicsListRegistry.registerArtifact(graphicListName, finalDesiredCapturePointViz.createArtifact());
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

      parentRegistry.addChild(registry);
   }

   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing, FootstepShiftFractions shiftFractions)
   {
      smoothCMPPlanner.addFootstepToPlan(footstep, timing, shiftFractions);
      footsteps.add().set(footstep);
      footstepTimings.add().set(timing);
   }


   public double getCurrentTransferDurationAdjustedForReachability()
   {
      return smoothCMPPlanner.getTransferDuration(0);
   }

   public double getCurrentSwingDurationAdjustedForReachability()
   {
      return smoothCMPPlanner.getSwingDuration(0);
   }

   public double getNextTransferDurationAdjustedForReachability()
   {
      return smoothCMPPlanner.getTransferDuration(1);
   }

   public boolean checkAndUpdateFootstepFromICPOptimization(Footstep footstep)
   {
      if (!usingStepAdjustment || initializeForSingleSupport || initializeForTransfer || initializeForStanding)
      {
         return false;
      }
      footstep.setPose(footstepSolution);
      return footstepWasAdjusted;
   }

   public void clearICPPlan()
   {
      smoothCMPPlanner.clearPlan();
      footsteps.clear();
      footstepTimings.clear();
   }

   public void setICPPlanSupportSide(RobotSide supportSide)
   {
      smoothCMPPlanner.setSupportLeg(supportSide);
      this.supportSide = supportSide;
   }

   public void setICPPlanTransferToSide(RobotSide transferToSide)
   {
      smoothCMPPlanner.setTransferToSide(transferToSide);
      this.transferToSide = transferToSide;
   }

   public void setICPPlanTransferFromSide(RobotSide robotSide)
   {
      smoothCMPPlanner.setTransferFromSide(robotSide);
      this.transferToSide = robotSide != null ? robotSide.getOppositeSide() : null;
   }

   public void endTick()
   {
      if (smoothCMPPlanner != null)
      {
         smoothCMPPlanner.endTick();
      }
   }

   private final FramePoint3D copEstimate = new FramePoint3D();
   public void compute(RobotSide supportLeg, double desiredCoMHeightAcceleration, boolean keepCoPInsideSupportPolygon, boolean controlHeightWithMomentum)
   {
      controllerToolbox.getCoP(copEstimate);

      smoothCMPPlanner.getDesiredCapturePointPosition(desiredCapturePoint2d);
      smoothCMPPlanner.getDesiredCapturePointVelocity(desiredCapturePointVelocity2d);
      smoothCMPPlanner.getDesiredCenterOfPressurePosition(perfectCoP2d);
      smoothCMPPlanner.getDesiredCenterOfMassPosition(yoDesiredCoMPosition);
      smoothCMPPlanner.getDesiredCenterOfMassVelocity(yoDesiredCoMVelocity);

      pelvisICPBasedTranslationManager.compute(supportLeg, capturePoint2d);
      pelvisICPBasedTranslationManager.addICPOffset(desiredCapturePoint2d, desiredCapturePointVelocity2d, perfectCoP2d);

      double omega0 = controllerToolbox.getOmega0();
      if (Double.isNaN(omega0))
         throw new RuntimeException("omega0 is NaN");

      // --- compute adjusted desired capture point

      // ---

      yoDesiredCapturePoint.set(desiredCapturePoint2d);
      yoDesiredICPVelocity.set(desiredCapturePointVelocity2d);
      yoPerfectCoP.set(perfectCoP2d);

      getICPError(icpError2d);

      CapturePointTools.computeCentroidalMomentumPivot(desiredCapturePoint2d, desiredCapturePointVelocity2d, omega0, yoPerfectCMP);

      for (RobotSide robotSide : RobotSide.values)
      {
         YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);
         contactState.getPlaneContactStateCommand(contactStateCommands.get(robotSide));
      }

      linearMomentumRateControlModuleInput.setDesiredCenterOfMassHeightAcceleration(desiredCoMHeightAcceleration);
      linearMomentumRateControlModuleInput.setInitializeForStanding(initializeForStanding);
      linearMomentumRateControlModuleInput.setInitializeForTransfer(initializeForTransfer);
      linearMomentumRateControlModuleInput.setInitializeForSingleSupport(initializeForSingleSupport);
      linearMomentumRateControlModuleInput.setFromFootsteps(footsteps);
      linearMomentumRateControlModuleInput.setFromFootstepTimings(footstepTimings);
      linearMomentumRateControlModuleInput.setFinalTransferDuration(finalTransferDuration);
      linearMomentumRateControlModuleInput.setKeepCoPInsideSupportPolygon(keepCoPInsideSupportPolygon);
      linearMomentumRateControlModuleInput.setControlHeightWithMomentum(controlHeightWithMomentum);
      linearMomentumRateControlModuleInput.setOmega0(omega0);
      linearMomentumRateControlModuleInput.setDesiredCapturePoint(desiredCapturePoint2d);
      linearMomentumRateControlModuleInput.setDesiredCapturePointVelocity(desiredCapturePointVelocity2d);
      linearMomentumRateControlModuleInput.setDesiredICPAtEndOfState(yoFinalDesiredICP);
      linearMomentumRateControlModuleInput.setPerfectCMP(yoPerfectCMP);
      linearMomentumRateControlModuleInput.setPerfectCoP(yoPerfectCoP);
      linearMomentumRateControlModuleInput.setSupportSide(supportSide);
      linearMomentumRateControlModuleInput.setTransferToSide(transferToSide);
      linearMomentumRateControlModuleInput.setRemainingTimeInSwingUnderDisturbance(timeRemainingInSwing);
      linearMomentumRateControlModuleInput.setContactStateCommand(contactStateCommands);

      initializeForStanding = false;
      initializeForTransfer = false;
      initializeForSingleSupport = false;
      footstepTimings.clear();
      footsteps.clear();
      supportSide = null;
      transferToSide = null;
      timeRemainingInSwing = Double.NaN;
   }

   public void computeICPPlan()
   {
      controllerToolbox.getCapturePoint(capturePoint2d);
      controllerToolbox.getCoP(copEstimate);
      smoothCMPPlanner.compute(yoTime.getDoubleValue());
      icpPlannerDone.set(smoothCMPPlanner.isDone());
   }

   public void disablePelvisXYControl()
   {
      pelvisICPBasedTranslationManager.disable();
   }

   public void enablePelvisXYControl()
   {
      pelvisICPBasedTranslationManager.enable();
   }

   public double estimateTimeRemainingForSwingUnderDisturbance()
   {
      controllerToolbox.getCapturePoint(capturePoint2d);

      return smoothCMPPlanner.estimateTimeRemainingForStateUnderDisturbance(capturePoint2d);
   }

   public void getDesiredICP(FramePoint2D desiredICPToPack)
   {
      desiredICPToPack.setIncludingFrame(yoDesiredCapturePoint);
   }

   public void getDesiredICPVelocity(FrameVector2D desiredICPVelocityToPack)
   {
      desiredICPVelocityToPack.setIncludingFrame(yoDesiredICPVelocity);
   }

   public void getFinalDesiredCoMPosition(FixedFramePoint3DBasics desiredCoMPositionToPack)
   {
      desiredCoMPositionToPack.set(yoFinalDesiredCoM);
   }

   public void getDesiredCoMPosition(FixedFramePoint3DBasics desiredCoMPositionToPack)
   {
      desiredCoMPositionToPack.set(yoDesiredCoMPosition);
   }

   public void getDesiredCoMVelocity(FixedFrameVector2DBasics desiredCoMVelocityToPack)
   {
      desiredCoMVelocityToPack.set(yoDesiredCoMVelocity);
   }

   public void getNextExitCMP(FramePoint3D entryCMPToPack)
   {
      smoothCMPPlanner.getNextExitCMP(entryCMPToPack);
   }

   public double getTimeRemainingInCurrentState()
   {
      return smoothCMPPlanner.getTimeInCurrentStateRemaining();
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
      yoFinalDesiredICP.set(Double.NaN, Double.NaN);
      controllerToolbox.getCapturePoint(tempCapturePoint);
      yoDesiredCapturePoint.set(tempCapturePoint);

      smoothCMPPlanner.holdCurrentICP(tempCapturePoint);
      smoothCMPPlanner.initializeForStanding(yoTime.getDoubleValue());

      initializeForStanding = true;

      endTick();
   }

   public void initializeICPPlanForSingleSupport(double finalTransferTime)
   {
      setFinalTransferTime(finalTransferTime);
      smoothCMPPlanner.initializeForSingleSupport(yoTime.getDoubleValue());
      initializeForSingleSupport = true;


      smoothCMPPlanner.getFinalDesiredCapturePointPosition(yoFinalDesiredICP);
      smoothCMPPlanner.getFinalDesiredCenterOfMassPosition(yoFinalDesiredCoM);

      icpPlannerDone.set(false);
   }


   public void initializeICPPlanForStanding()
   {
      if (holdICPToCurrentCoMLocationInNextDoubleSupport.getBooleanValue())
      {
         requestICPPlannerToHoldCurrentCoM();
         holdICPToCurrentCoMLocationInNextDoubleSupport.set(false);
      }
      smoothCMPPlanner.initializeForStanding(yoTime.getDoubleValue());
      initializeForStanding = true;

      icpPlannerDone.set(false);
   }

   public void initializeICPPlanForTransferToStanding(double finalTransferTime)
   {
      if (holdICPToCurrentCoMLocationInNextDoubleSupport.getBooleanValue())
      {
         requestICPPlannerToHoldCurrentCoM();
         holdICPToCurrentCoMLocationInNextDoubleSupport.set(false);
      }
      setFinalTransferTime(finalTransferTime);
      smoothCMPPlanner.initializeForTransfer(yoTime.getDoubleValue());
      initializeForStanding = true;

      icpPlannerDone.set(false);
   }

   public void initializeICPPlanForTransfer(double finalTransferTime)
   {
      if (holdICPToCurrentCoMLocationInNextDoubleSupport.getBooleanValue())
      {
         requestICPPlannerToHoldCurrentCoM();
         holdICPToCurrentCoMLocationInNextDoubleSupport.set(false);
      }
      setFinalTransferTime(finalTransferTime);
      smoothCMPPlanner.initializeForTransfer(yoTime.getDoubleValue());

      initializeForTransfer = true;

      smoothCMPPlanner.getFinalDesiredCapturePointPosition(yoFinalDesiredICP);
      smoothCMPPlanner.getFinalDesiredCenterOfMassPosition(yoFinalDesiredCoM);

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

   public double getICPDistanceOutsideSupportForStep()
   {
      return icpDistanceOutsideSupportForStep.getValue();
   }

   public double getICPErrorMagnitude()
   {
      controllerToolbox.getCapturePoint(capturePoint2d);
      return capturePoint2d.distance(yoDesiredCapturePoint);
   }

   public void getICPError(FrameVector2D icpErrorToPack)
   {
      controllerToolbox.getCapturePoint(capturePoint2d);
      desiredCapturePoint2d.setIncludingFrame(yoDesiredCapturePoint);
      icpErrorToPack.setIncludingFrame(desiredCapturePoint2d);
      icpErrorToPack.sub(capturePoint2d);
   }


   public boolean isICPPlanDone()
   {
      return icpPlannerDone.getValue();
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
      smoothCMPPlanner.holdCurrentICP(centerOfMassPosition);
   }

   public void setFinalTransferWeightDistribution(double weightDistribution)
   {
      smoothCMPPlanner.setFinalTransferWeightDistribution(weightDistribution);
   }

   public void setFinalTransferSplitFraction(double finalTransferSplitFraction)
   {
      smoothCMPPlanner.setFinalTransferDurationAlpha(finalTransferSplitFraction);
   }

   public void setFinalTransferTime(double finalTransferDuration)
   {
      smoothCMPPlanner.setFinalTransferDuration(finalTransferDuration);
      this.finalTransferDuration = finalTransferDuration;
   }

   /**
    * Update the basics: capture point, omega0, and the support polygons.
    */
   public void update()
   {
      computeICPPlan();
      smoothCMPPlanner.getFinalDesiredCapturePointPosition(yoFinalDesiredICP);
      smoothCMPPlanner.getFinalDesiredCenterOfMassPosition(yoFinalDesiredCoM);
   }

   public CapturabilityBasedStatus updateAndReturnCapturabilityBasedStatus()
   {
      desiredCapturePoint2d.setIncludingFrame(yoDesiredCapturePoint);
      centerOfMassPosition.setToZero(centerOfMassFrame);
      centerOfMassPosition.changeFrame(worldFrame);

      controllerToolbox.getCapturePoint(capturePoint2d);
      capturePoint2d.checkReferenceFrameMatch(worldFrame);
      desiredCapturePoint2d.checkReferenceFrameMatch(worldFrame);

      capturabilityBasedStatus.setOmega(controllerToolbox.getOmega0());
      capturabilityBasedStatus.getCapturePoint2d().set(capturePoint2d);
      capturabilityBasedStatus.getDesiredCapturePoint2d().set(desiredCapturePoint2d);
      capturabilityBasedStatus.getCenterOfMass3d().set(centerOfMassPosition);
      for (RobotSide robotSide : RobotSide.values)
      {
         HumanoidMessageTools.packFootSupportPolygon(robotSide, bipedSupportPolygons.getFootPolygonInWorldFrame(robotSide), capturabilityBasedStatus);
      }

      return capturabilityBasedStatus;
   }

   public void updateCurrentICPPlan()
   {
      smoothCMPPlanner.updateCurrentPlan();
   }

   public void updateSwingTimeRemaining(double timeRemainingInSwing)
   {
      this.timeRemainingInSwing = timeRemainingInSwing;
   }

   public FrameVector3DReadOnly getEffectiveICPAdjustment()
   {
      return effectiveICPAdjustment;
   }

   public void getCapturePoint(FramePoint2D capturePointToPack)
   {
      controllerToolbox.getCapturePoint(capturePointToPack);
   }

   public LinearMomentumRateControlModuleInput getLinearMomentumRateControlModuleInput()
   {
      return linearMomentumRateControlModuleInput;
   }

   public void setLinearMomentumRateControlModuleOutput(LinearMomentumRateControlModuleOutput output)
   {
      desiredCMP.set(output.getDesiredCMP());
      effectiveICPAdjustment.set(output.getEffectiveICPAdjustment());
      footstepSolution.set(output.getFootstepSolution());
      footstepWasAdjusted = output.getFootstepWasAdjusted();
      usingStepAdjustment = output.getUsingStepAdjustment();
   }

   public TaskspaceTrajectoryStatusMessage pollPelvisXYTranslationStatusToReport()
   {
      return pelvisICPBasedTranslationManager.pollStatusToReport();
   }
}
