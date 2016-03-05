package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.captureRegion.PushRecoveryControlModule;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisICPBasedTranslationManager;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableGoHomeMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiablePelvisTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableStopAllTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.output.ControllerStatusOutputManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.ICPAndMomentumBasedController;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.InverseDynamicsCommand;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.controllers.YoPDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.ConvexPolygonShrinker;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class BalanceManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BipedSupportPolygons bipedSupportPolygons;
   private final ICPPlannerWithTimeFreezer icpPlanner;
   private final ICPAndMomentumBasedController icpAndMomentumBasedController;
   private final PelvisICPBasedTranslationManager pelvisICPBasedTranslationManager;
   private final PushRecoveryControlModule pushRecoveryControlModule;

   private final YoFramePoint2d finalDesiredICPInWorld = new YoFramePoint2d("finalDesiredICPInWorld", "", worldFrame, registry);

   private final YoFramePoint2d desiredECMP = new YoFramePoint2d("desiredECMP", "", worldFrame, registry);
   private final YoFramePoint ecmpViz = new YoFramePoint("ecmpViz", worldFrame, registry);

   private final DoubleYoVariable yoTime;

   private final ReferenceFrame centerOfMassFrame;

   private final FramePoint2d desiredCapturePoint2d = new FramePoint2d();
   private final FramePoint2d capturePoint2d = new FramePoint2d();
   private final FrameVector2d desiredCapturePointVelocity2d = new FrameVector2d();
   private final FramePoint2d finalDesiredCapturePoint2d = new FramePoint2d();
   private final ConvexPolygonShrinker convexPolygonShrinker = new ConvexPolygonShrinker();
   private final FrameConvexPolygon2d shrunkSupportPolygon = new FrameConvexPolygon2d();

   private final FramePoint centerOfMass = new FramePoint();
   private final FramePoint2d centerOfMass2d = new FramePoint2d();

   public BalanceManager(ControllerStatusOutputManager statusOutputManager, MomentumBasedController momentumBasedController,
         WalkingControllerParameters walkingControllerParameters, CapturePointPlannerParameters capturePointPlannerParameters,
         YoVariableRegistry parentRegistry)
   {
      CommonHumanoidReferenceFrames referenceFrames = momentumBasedController.getReferenceFrames();
      FullHumanoidRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();

      SideDependentList<ReferenceFrame> ankleZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();
      SideDependentList<ReferenceFrame> soleZUpFrames = referenceFrames.getSoleZUpFrames();
      ReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
      YoGraphicsListRegistry yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();
      SideDependentList<? extends ContactablePlaneBody> contactableFeet = momentumBasedController.getContactableFeet();
      double omega0 = walkingControllerParameters.getOmega0();
      ICPControlGains icpControlGains = walkingControllerParameters.getICPControlGains();
      double controlDT = momentumBasedController.getControlDT();
      double gravityZ = momentumBasedController.getGravityZ();
      double totalMass = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());
      double minimumSwingTimeForDisturbanceRecovery = walkingControllerParameters.getMinimumSwingTimeForDisturbanceRecovery();

      yoTime = momentumBasedController.getYoTime();

      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();

      bipedSupportPolygons = new BipedSupportPolygons(ankleZUpFrames, midFeetZUpFrame, soleZUpFrames, registry, yoGraphicsListRegistry);

      ICPBasedLinearMomentumRateOfChangeControlModule icpBasedLinearMomentumRateOfChangeControlModule = new ICPBasedLinearMomentumRateOfChangeControlModule(
            referenceFrames, bipedSupportPolygons, controlDT, totalMass, gravityZ, icpControlGains, registry, yoGraphicsListRegistry);

      icpPlanner = new ICPPlannerWithTimeFreezer(bipedSupportPolygons, contactableFeet, capturePointPlannerParameters, registry, yoGraphicsListRegistry);
      icpPlanner.setMinimumSingleSupportTimeForDisturbanceRecovery(minimumSwingTimeForDisturbanceRecovery);
      icpPlanner.setOmega0(omega0);
      icpPlanner.setSingleSupportTime(walkingControllerParameters.getDefaultSwingTime());
      icpPlanner.setDoubleSupportTime(walkingControllerParameters.getDefaultTransferTime());
      icpAndMomentumBasedController = new ICPAndMomentumBasedController(momentumBasedController, omega0, icpBasedLinearMomentumRateOfChangeControlModule,
            bipedSupportPolygons, statusOutputManager, parentRegistry);
      YoPDGains pelvisXYControlGains = walkingControllerParameters.createPelvisICPBasedXYControlGains(registry);
      pelvisICPBasedTranslationManager = new PelvisICPBasedTranslationManager(momentumBasedController, bipedSupportPolygons, pelvisXYControlGains, registry);

      pushRecoveryControlModule = new PushRecoveryControlModule(bipedSupportPolygons, momentumBasedController, walkingControllerParameters, registry);

      YoGraphicPosition dynamicGraphicPositionECMP = new YoGraphicPosition("ecmpviz", ecmpViz, 0.002, YoAppearance.BlueViolet());
      yoGraphicsListRegistry.registerYoGraphic("ecmpviz", dynamicGraphicPositionECMP);
      yoGraphicsListRegistry.registerArtifact("ecmpviz", dynamicGraphicPositionECMP.createArtifact());

      parentRegistry.addChild(registry);
   }

   public void addFootstepToPlan(Footstep footstep)
   {
      icpPlanner.addFootstepToPlan(footstep);
   }

   public boolean checkAndUpdateFootstep(Footstep footstep)
   {
      return pushRecoveryControlModule.checkAndUpdateFootstep(getTimeRemainingInCurrentState(), footstep);
   }

   public void clearICPPlan()
   {
      icpPlanner.clearPlan();
   }

   public void compute(RobotSide supportLeg, boolean keepCMPInsideSupportPolygon)
   {
      if (supportLeg == null)
         computeForDoubleSupport();
      else
         computeForSingleSupport(supportLeg);

      finalDesiredICPInWorld.getFrameTuple2dIncludingFrame(finalDesiredCapturePoint2d);
      icpAndMomentumBasedController.compute(finalDesiredCapturePoint2d, keepCMPInsideSupportPolygon);
   }

   public void computeForDoubleSupport()
   {
      getCapturePoint(capturePoint2d);
      icpPlanner.getDesiredCapturePointPositionAndVelocity(desiredCapturePoint2d, desiredCapturePointVelocity2d, capturePoint2d, yoTime.getDoubleValue());
      computePelvisXY(null, desiredCapturePoint2d, desiredCapturePointVelocity2d);
      setDesiredICP(desiredCapturePoint2d);
      setDesiredICPVelocity(desiredCapturePointVelocity2d);
      updatePushRecovery(true);
   }

   public void computeForSingleSupport(RobotSide supportSide)
   {
      icpPlanner.getDesiredCapturePointPositionAndVelocity(desiredCapturePoint2d, desiredCapturePointVelocity2d, yoTime.getDoubleValue());
      computePelvisXY(supportSide, desiredCapturePoint2d, desiredCapturePointVelocity2d);
      setDesiredICP(desiredCapturePoint2d);
      setDesiredICPVelocity(desiredCapturePointVelocity2d);
      updatePushRecovery(false);
   }

   public void computePelvisXY(RobotSide supportLeg, FramePoint2d desiredICPToModify, FrameVector2d desiredICPVelocityToModify)
   {
      icpAndMomentumBasedController.getCapturePoint(capturePoint2d);
      pelvisICPBasedTranslationManager.compute(supportLeg, capturePoint2d);
      pelvisICPBasedTranslationManager.addICPOffset(desiredICPToModify, desiredICPVelocityToModify);
   }

   public Footstep createFootstepForRecoveringFromDisturbance(RobotSide swingSide, double swingTimeRemaining)
   {
      return pushRecoveryControlModule.createFootstepForRecoveringFromDisturbance(swingSide, swingTimeRemaining);
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
      icpAndMomentumBasedController.getCapturePoint(capturePoint2d);
      return icpPlanner.estimateTimeRemainingForStateUnderDisturbance(yoTime.getDoubleValue(), capturePoint2d);
   }

   public void freezePelvisXYControl()
   {
      pelvisICPBasedTranslationManager.freeze();
   }

   public BipedSupportPolygons getBipedSupportPolygons()
   {
      return bipedSupportPolygons;
   }

   public void getCapturePoint(FramePoint2d capturePointToPack)
   {
      icpAndMomentumBasedController.getCapturePoint(capturePointToPack);
   }

   public DoubleYoVariable getControlledCoMHeightAcceleration()
   {
      return icpAndMomentumBasedController.getControlledCoMHeightAcceleration();
   }

   public void getDesiredCapturePointPositionAndVelocity(FramePoint2d desiredCapturePointPositionToPack, FrameVector2d desiredCapturePointVelocityToPack)
   {
      icpAndMomentumBasedController.getCapturePoint(capturePoint2d);
      icpPlanner.getDesiredCapturePointPositionAndVelocity(desiredCapturePointPositionToPack, desiredCapturePointVelocityToPack, capturePoint2d,
            yoTime.getDoubleValue());
   }

   public void getDesiredCMP(FramePoint2d desiredCMP)
   {
      icpAndMomentumBasedController.getDesiredCMP(desiredCMP);
   }

   public void getDesiredICP(FramePoint2d desiredICP)
   {
      icpAndMomentumBasedController.getDesiredICP(desiredICP);
   }

   public void getDesiredICPVelocity(FrameVector2d desiredICPToPack)
   {
      icpAndMomentumBasedController.getDesiredICPVelocity(desiredICPToPack);
   }

   public void getFinalDesiredCapturePointPosition(FramePoint2d finalDesiredCapturePointPositionToPack)
   {
      icpPlanner.getFinalDesiredCapturePointPosition(finalDesiredCapturePointPositionToPack);
   }

   public void getFinalDesiredCapturePointPosition(YoFramePoint2d finalDesiredCapturePointPositionToPack)
   {
      icpPlanner.getFinalDesiredCapturePointPosition(finalDesiredCapturePointPositionToPack);
   }

   public double getInitialTransferDuration()
   {
      return icpPlanner.getInitialTransferDuration();
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return icpAndMomentumBasedController.getInverseDynamicsCommand();
   }

   public void getNextExitCMP(FramePoint entryCMPToPack)
   {
      icpPlanner.getNextExitCMP(entryCMPToPack);
   }

   public double getOmega0()
   {
      return icpAndMomentumBasedController.getOmega0();
   }

   public double getTimeRemainingInCurrentState()
   {
      return icpPlanner.computeAndReturnTimeInCurrentState(yoTime.getDoubleValue());
   }

   public EnumYoVariable<RobotSide> getYoSupportLeg()
   {
      return icpAndMomentumBasedController.getYoSupportLeg();
   }

   public void handleGoHomeMessage(ModifiableGoHomeMessage message)
   {
      pelvisICPBasedTranslationManager.handleGoHomeMessage(message);
   }

   public void handlePelvisTrajectoryMessage(ModifiablePelvisTrajectoryMessage message)
   {
      pelvisICPBasedTranslationManager.handlePelvisTrajectoryMessage(message);
   }

   public void handleStopAllTrajectoryMessage(ModifiableStopAllTrajectoryMessage message)
   {
      pelvisICPBasedTranslationManager.handleStopAllTrajectoryMessage(message);
   }

   public void holdCurrentICP(double doubleValue, FramePoint tmpFramePoint)
   {
      icpPlanner.holdCurrentICP(doubleValue, tmpFramePoint);
   }

   public void initialize()
   {
      finalDesiredICPInWorld.set(Double.NaN, Double.NaN);
      icpAndMomentumBasedController.getDesiredICP().setByProjectionOntoXYPlane(icpAndMomentumBasedController.getCapturePoint());
   }

   public void initializeForDoubleSupportPushRecovery()
   {
      pushRecoveryControlModule.initializeParametersForDoubleSupportPushRecovery();
   }

   public void initializeICPPlanForSingleSupport(RobotSide supportSide)
   {
      icpPlanner.initializeForSingleSupport(yoTime.getDoubleValue(), supportSide);
      icpPlanner.getFinalDesiredCapturePointPosition(finalDesiredICPInWorld);
   }

   public void initializeICPPlanForStanding()
   {
      icpPlanner.initializeForStanding(yoTime.getDoubleValue());
   }

   public void initializeICPPlanForTransfer(RobotSide transferToSide)
   {
      icpPlanner.initializeForTransfer(yoTime.getDoubleValue(), transferToSide);
      icpPlanner.getFinalDesiredCapturePointPosition(finalDesiredICPInWorld);
   }

   public double getICPErrorMagnitude()
   {
      return icpAndMomentumBasedController.getCapturePoint().getXYPlaneDistance(icpAndMomentumBasedController.getDesiredICP());
   }

   public void getICPError(FrameVector2d icpErrorToPack)
   {
      getCapturePoint(capturePoint2d);
      getDesiredICP(desiredCapturePoint2d);
      icpErrorToPack.setIncludingFrame(desiredCapturePoint2d);
      icpErrorToPack.sub(capturePoint2d);
   }

   public boolean isICPPlanDone(double doubleValue)
   {
      return icpPlanner.isDone(doubleValue);
   }

   public boolean isOnExitCMP()
   {
      return icpPlanner.isOnExitCMP();
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

   public void requestICPPlannerToHoldCurrentCoM(double distancceFromSupportPolygonEdges)
   {
      centerOfMass.setToZero(centerOfMassFrame);

      updateBipedSupportPolygons();
      FrameConvexPolygon2d supportPolygonInMidFeetZUp = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();
      convexPolygonShrinker.shrinkConstantDistanceInto(supportPolygonInMidFeetZUp, distancceFromSupportPolygonEdges, shrunkSupportPolygon);

      centerOfMass.changeFrame(shrunkSupportPolygon.getReferenceFrame());
      centerOfMass2d.setByProjectionOntoXYPlaneIncludingFrame(centerOfMass);
      shrunkSupportPolygon.orthogonalProjection(centerOfMass2d);
      centerOfMass.setXY(centerOfMass2d);

      centerOfMass.changeFrame(worldFrame);
      icpPlanner.holdCurrentICP(yoTime.getDoubleValue(), centerOfMass);
   }

   public void setDesiredICP(FramePoint2d desiredCapturePoint)
   {
      icpAndMomentumBasedController.setDesiredICP(desiredCapturePoint);
   }

   public void setDesiredICPVelocity(FrameVector2d desiredICP)
   {
      icpAndMomentumBasedController.setDesiredICPVelocity(desiredICP);
   }

   public void setDoubleSupportTime(double newDoubleSupportTime)
   {
      icpPlanner.setDoubleSupportTime(newDoubleSupportTime);
   }

   public void setSingleSupportTime(double newSingleSupportTime)
   {
      icpPlanner.setSingleSupportTime(newSingleSupportTime);
   }

   public void update()
   {
      YoFramePoint2d desiredICP = icpAndMomentumBasedController.getDesiredICP();
      YoFrameVector2d desiredICPVelocity = icpAndMomentumBasedController.getDesiredICPVelocity();
      CapturePointTools.computeDesiredCentroidalMomentumPivot(desiredICP, desiredICPVelocity, getOmega0(), desiredECMP);
      ecmpViz.setXY(desiredECMP);
      icpAndMomentumBasedController.update();
   }

   public void updateBipedSupportPolygons()
   {
      icpAndMomentumBasedController.updateBipedSupportPolygons();
   }

   public void updateICPPlanForSingleSupportDisturbances()
   {
      icpAndMomentumBasedController.getCapturePoint(capturePoint2d);
      icpPlanner.updatePlanForSingleSupportDisturbances(yoTime.getDoubleValue(), capturePoint2d);
      icpPlanner.getFinalDesiredCapturePointPosition(finalDesiredICPInWorld);
   }

   public void updatePushRecovery(boolean isInDoubleSupport)
   {
      getDesiredICP(desiredCapturePoint2d);
      getCapturePoint(capturePoint2d);
      if (isInDoubleSupport)
      {
         pushRecoveryControlModule.updateForDoubleSupport(desiredCapturePoint2d, capturePoint2d, getOmega0());
      }
      else
      {
         pushRecoveryControlModule.updateForSingleSupport(desiredCapturePoint2d, capturePoint2d, getOmega0());
      }
   }
}
