package us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedFeetUpdater;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DesiredPelvisOrientationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.BalanceSupportControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.FootOrientationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.KneeExtensionControlModule;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredVelocityControlModule;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LowerBodyTorques;
import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.stateEstimation.LegToTrustForVelocityWriteOnly;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;


public class CommonStanceSubController implements StanceSubController
{
   private final ProcessedSensorsInterface processedSensors;
   private final CouplingRegistry couplingRegistry;
   private final CommonHumanoidReferenceFrames referenceFrames;
   private final DesiredVelocityControlModule desiredVelocityControlModule;
   private final DesiredPelvisOrientationControlModule desiredPelvisOrientationControlModule;
   private final BalanceSupportControlModule balanceSupportControlModule;
   private final FootOrientationControlModule footOrientationControlModule;
   private final DesiredHeadingControlModule desiredHeadingControlModule;
   private final KneeExtensionControlModule kneeExtensionControlModule;

   private final YoVariableRegistry registry = new YoVariableRegistry("StanceSubController");

   private final DoubleYoVariable toeOffFootPitch = new DoubleYoVariable("toeOffFootPitch",
                                                       "This is the desired foot pitch at the end of toe-off during stance", registry);
   private final DoubleYoVariable toeOffMoveDuration = new DoubleYoVariable("toeOffMoveDuration", "The duration of the toe-off move during stance", registry);
   private final DoubleYoVariable loadingPreSwingAEntryTime = new DoubleYoVariable("doubleSupportStartTime", registry);

   private final DoubleYoVariable minDoubleSupportTimeBeforeWalking = new DoubleYoVariable("minDoubleSupportTimeBeforeWalking", registry);
   private final DoubleYoVariable minPercentageTowardsDesired = new DoubleYoVariable("minPercentageTowardsDesired", registry);

   private final LegToTrustForVelocityWriteOnly supportLegAndLegToTrustForVelocity;
   private final BipedFeetUpdater bipedFeetUpdater;
   
   private final BooleanYoVariable doPushRecovery = new BooleanYoVariable("doPushRecovery", registry);

   private final double footWidth;

   public CommonStanceSubController(ProcessedSensorsInterface processedSensors, CouplingRegistry couplingRegistry,
                                    CommonHumanoidReferenceFrames referenceFrames, DesiredHeadingControlModule desiredHeadingControlModule,
                                    DesiredVelocityControlModule desiredVelocityControlModule,
                                    DesiredPelvisOrientationControlModule desiredPelvisOrientationControlModule,
                                    BalanceSupportControlModule balanceSupportControlModule, FootOrientationControlModule footOrientationControlModule,
                                    KneeExtensionControlModule kneeExtensionControlModule,
                                    LegToTrustForVelocityWriteOnly supportLegAndLegToTrustForVelocity, BipedFeetUpdater bipedFeetUpdater, YoVariableRegistry parentRegistry, double footWidth)
   {
      this.processedSensors = processedSensors;
      this.couplingRegistry = couplingRegistry;
      this.referenceFrames = referenceFrames;
      this.desiredVelocityControlModule = desiredVelocityControlModule;
      this.desiredPelvisOrientationControlModule = desiredPelvisOrientationControlModule;
      this.balanceSupportControlModule = balanceSupportControlModule;
      this.footOrientationControlModule = footOrientationControlModule;
      this.desiredHeadingControlModule = desiredHeadingControlModule;
      this.kneeExtensionControlModule = kneeExtensionControlModule;
      this.supportLegAndLegToTrustForVelocity = supportLegAndLegToTrustForVelocity;
      this.bipedFeetUpdater = bipedFeetUpdater;
      this.footWidth = footWidth;

      parentRegistry.addChild(registry);
   }

   public boolean canWeStopNow()
   {
      return true;
   }

   public void doEarlyStance(LegTorques legTorquesToPackForStanceSide, double timeInState)
   {
      doSingleSupportControl(legTorquesToPackForStanceSide, SingleSupportCondition.EarlyStance, timeInState);

      // Here is where we want to add the torque for the kneeExtensionController
      kneeExtensionControlModule.doEarlyStanceKneeExtensionControl(legTorquesToPackForStanceSide);
      setEstimatedDoubleSupportTimeRemaining(legTorquesToPackForStanceSide.getRobotSide(), true);
   }

   public void doLateStance(LegTorques legTorquesToPackForStanceSide, double timeInState)
   {
      doSingleSupportControl(legTorquesToPackForStanceSide, SingleSupportCondition.LateStance, timeInState);
      footOrientationControlModule.addAdditionalTorqueForFootOrientationControl(legTorquesToPackForStanceSide, timeInState);
      kneeExtensionControlModule.doLateStanceKneeExtensionControl(legTorquesToPackForStanceSide);
      setEstimatedDoubleSupportTimeRemaining(legTorquesToPackForStanceSide.getRobotSide(), true);
   }

   public void doTerminalStance(LegTorques legTorquesToPackForStanceSide, double timeInState)
   {
      RobotSide supportLeg = legTorquesToPackForStanceSide.getRobotSide();

      doSingleSupportControl(legTorquesToPackForStanceSide, SingleSupportCondition.TerminalStance, timeInState);

//      footOrientationControlModule.addAdditionalTorqueForFootOrientationControl(legTorquesToPackForStanceSide,
//              timeInState + timeSpentInLateStance.getDoubleValue());

      kneeExtensionControlModule.breakKneeForDownhillSlopes(supportLeg.getOppositeSide());
      setEstimatedDoubleSupportTimeRemaining(legTorquesToPackForStanceSide.getRobotSide(), true);
   }


   public void doLoadingPreSwingA(LowerBodyTorques lowerBodyTorquesToPack, RobotSide loadingLeg, double timeInState)
   {
      doDoubleSupportControl(lowerBodyTorquesToPack, loadingLeg, true);

//      if ((timeSpentInLateStance.getDoubleValue() + timeSpentInTerminalStance.getDoubleValue()) != 0.0)
//         footOrientationControlModule.addAdditionalTorqueForFootOrientationControl(lowerBodyTorquesToPack.getLegTorques(loadingLeg.getOppositeSide()),
//                 timeInState + timeSpentInLateStance.getDoubleValue() + timeSpentInTerminalStance.getDoubleValue());

      kneeExtensionControlModule.doLoadingControl(lowerBodyTorquesToPack.getLegTorques(loadingLeg));
      setEstimatedDoubleSupportTimeRemaining(null, true);
   }

   public void doLoadingPreSwingB(LowerBodyTorques lowerBodyTorquesToPack, RobotSide loadingLeg, double timeInState)
   {
      doDoubleSupportControl(lowerBodyTorquesToPack, loadingLeg, true);
      kneeExtensionControlModule.doLoadingControl(lowerBodyTorquesToPack.getLegTorques(loadingLeg));
      setEstimatedDoubleSupportTimeRemaining(null, true);
   }

   public void doLoadingPreSwingC(LegTorques legTorquesToPackForStanceSide, RobotSide loadingLeg, double timeInState)
   {
      doSingleSupportControl(legTorquesToPackForStanceSide, SingleSupportCondition.Loading, timeInState);
      kneeExtensionControlModule.doLoadingControl(legTorquesToPackForStanceSide);
      setEstimatedDoubleSupportTimeRemaining(null, true);
   }

   public void doStartWalkingDoubleSupport(LowerBodyTorques lowerBodyTorquesToPack, RobotSide loadingLeg, double timeInState)
   {
      doDoubleSupportControl(lowerBodyTorquesToPack, loadingLeg, false);
      couplingRegistry.setEstimatedDoubleSupportTimeRemaining(Double.POSITIVE_INFINITY);
      
      bipedFeetUpdater.setResizePolygonInDoubleSupport(false);
      setEstimatedDoubleSupportTimeRemaining(null, false);
   }

   public void doUnloadLegToTransferIntoWalking(LowerBodyTorques lowerBodyTorquesToPack, RobotSide supportLeg, double timeInState)
   {
      doDoubleSupportControl(lowerBodyTorquesToPack, supportLeg, true);
      // TODO: set estimated double support time remaining
   }

   public void doLoadingForSingleLegBalance(LowerBodyTorques lowerBodyTorques, RobotSide upcomingSupportSide, double timeInCurrentState)
   {
      doDoubleSupportControl(lowerBodyTorques, upcomingSupportSide, false);
      // TODO: set estimated double support time remaining
   }

   public void doSingleLegBalance(LegTorques legTorquesToPack, RobotSide supportLeg, double timeInCurrentState)
   {
      FrameVector2d finalHeadingTarget = new FrameVector2d(referenceFrames.getAnkleZUpFrame(supportLeg), 1.0, 0.0);
      finalHeadingTarget.changeFrame(ReferenceFrame.getWorldFrame());
      desiredHeadingControlModule.setFinalHeadingTarget(finalHeadingTarget);

      doSingleSupportControl(legTorquesToPack, SingleSupportCondition.StopWalking, timeInCurrentState);
      couplingRegistry.setEstimatedSwingTimeRemaining(0.0);
   }

   public void doTransitionIntoEarlyStance(RobotSide stanceSide)
   {
      kneeExtensionControlModule.doTransitionIntoStance(stanceSide);
   }

   public void doTransitionIntoLateStance(RobotSide stanceSide)
   {
      ReferenceFrame desiredHeadingFrame = desiredHeadingControlModule.getDesiredHeadingFrame();
      FrameOrientation finalOrientation = new FrameOrientation(desiredHeadingFrame, 0.0, toeOffFootPitch.getDoubleValue(), 0.0);
      footOrientationControlModule.initializeFootOrientationMove(toeOffMoveDuration.getDoubleValue(), finalOrientation, stanceSide);
   }

   public void doTransitionIntoTerminalStance(RobotSide stanceSide)
   {
   }

   public void doTransitionIntoLoadingPreSwingA(RobotSide loadingLeg)
   {
      supportLegAndLegToTrustForVelocity.setLegToTrustForVelocity(loadingLeg, true);
      supportLegAndLegToTrustForVelocity.setLegToTrustForVelocity(loadingLeg.getOppositeSide(), false);
      supportLegAndLegToTrustForVelocity.setSupportLeg(null);
      supportLegAndLegToTrustForVelocity.setLegToUseForCOMOffset(loadingLeg);

      kneeExtensionControlModule.doTransitionIntoLoading(loadingLeg);
      loadingPreSwingAEntryTime.set(processedSensors.getTime());
      
      couplingRegistry.setUpcomingSupportLeg(loadingLeg);
      
      bipedFeetUpdater.setResizePolygonInDoubleSupport(true);
   }

   public void doTransitionIntoLoadingPreSwingB(RobotSide loadingLeg)
   {
      supportLegAndLegToTrustForVelocity.setLegToTrustForVelocity(loadingLeg, true);
      supportLegAndLegToTrustForVelocity.setLegToTrustForVelocity(loadingLeg.getOppositeSide(), false);
      
      bipedFeetUpdater.setResizePolygonInDoubleSupport(true);

   }

   public void doTransitionIntoLoadingPreSwingC(RobotSide loadingLeg)
   {
      supportLegAndLegToTrustForVelocity.setLegToTrustForVelocity(loadingLeg, true);
      supportLegAndLegToTrustForVelocity.setLegToTrustForVelocity(loadingLeg.getOppositeSide(), false);
      supportLegAndLegToTrustForVelocity.setSupportLeg(loadingLeg);
      supportLegAndLegToTrustForVelocity.setLegToUseForCOMOffset(loadingLeg);
   }

   public void doTransitionIntoStartStopWalkingDoubleSupport(RobotSide stanceSide)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         supportLegAndLegToTrustForVelocity.setLegToTrustForVelocity(robotSide, true);
      }
      supportLegAndLegToTrustForVelocity.setSupportLeg(null);
      couplingRegistry.setUpcomingSupportLeg(stanceSide);
      supportLegAndLegToTrustForVelocity.setLegToUseForCOMOffset(null);

      couplingRegistry.setEstimatedDoubleSupportTimeRemaining(Double.POSITIVE_INFINITY);
      
      desiredPelvisOrientationControlModule.useTwistScale(false);
//    balanceSupportControlModule.setDesiredCoPOffset(new FramePoint2d(ReferenceFrame.getWorldFrame())); // didn't do anything...
   }

   public void doTransitionIntoUnloadLegToTransferIntoWalking(RobotSide stanceSide)
   {
      supportLegAndLegToTrustForVelocity.setLegToTrustForVelocity(stanceSide, true);
      supportLegAndLegToTrustForVelocity.setLegToTrustForVelocity(stanceSide.getOppositeSide(), false);
      supportLegAndLegToTrustForVelocity.setSupportLeg(null);
      supportLegAndLegToTrustForVelocity.setLegToUseForCOMOffset(stanceSide);
      
      bipedFeetUpdater.setResizePolygonInDoubleSupport(true);

//    BipedSupportPolygons bipedSupportPolygons = couplingRegistry.getBipedSupportPolygons();
//
//    FrameConvexPolygon2d singleSupportFootPolygon = bipedSupportPolygons.getFootPolygonInAnkleZUp(stanceSide);    // processedSensors.getFootPolygons().get(stanceSide).getFrameConvexPolygon2dCopy();
//    FramePoint2d singleSupportCentroid = singleSupportFootPolygon.getCentroidCopy();
//
//    FrameConvexPolygon2d supportPolygon = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();
//    FramePoint2d doubleSupportCentroid = getCenterOfBoundingBoxOfPolygon(supportPolygon).changeFrameCopy(singleSupportCentroid.getReferenceFrame());
//
//    singleSupportCentroid.sub(doubleSupportCentroid);
//
//    balanceSupportControlModule.setDesiredCoPOffset(singleSupportCentroid); // didn't do anything...
   }


   public void doTransitionIntoLoadingForSingleLegBalance(RobotSide upcomingSupportSide)
   {
      supportLegAndLegToTrustForVelocity.setLegToTrustForVelocity(upcomingSupportSide, true);
      supportLegAndLegToTrustForVelocity.setLegToTrustForVelocity(upcomingSupportSide.getOppositeSide(), false);
      supportLegAndLegToTrustForVelocity.setSupportLeg(upcomingSupportSide);
      supportLegAndLegToTrustForVelocity.setLegToUseForCOMOffset(upcomingSupportSide);
   }

   public void doTransitionIntoSingleLegBalance(RobotSide supportLeg, double[] desiredYawPitchRoll)
   {
      desiredPelvisOrientationControlModule.setDesiredPelvisOrientation(new FrameOrientation(desiredHeadingControlModule.getDesiredHeadingFrame(), desiredYawPitchRoll));
   }

   public void doTransitionOutOfEarlyStance(RobotSide stanceSide)
   {
   }

   public void doTransitionOutOfLateStance(RobotSide stanceSide)
   {
   }

   public void doTransitionOutOfLoadingPreSwingA(RobotSide loadingLeg)
   {
      bipedFeetUpdater.setResizePolygonInDoubleSupport(false);
   }

   public void doTransitionOutOfLoadingPreSwingB(RobotSide loadingLeg)
   {
      bipedFeetUpdater.setResizePolygonInDoubleSupport(false);
   }

   public void doTransitionOutOfLoadingPreSwingC(RobotSide loadingLeg)
   {
   }

   public void doTransitionOutOfStartStopWalkingDoubleSupport(RobotSide loadingLeg)
   {
      supportLegAndLegToTrustForVelocity.setLegToTrustForVelocity(loadingLeg, true);
      supportLegAndLegToTrustForVelocity.setLegToTrustForVelocity(loadingLeg.getOppositeSide(), false);
      supportLegAndLegToTrustForVelocity.setSupportLeg(loadingLeg);
      supportLegAndLegToTrustForVelocity.setLegToUseForCOMOffset(loadingLeg);
      
      desiredPelvisOrientationControlModule.useTwistScale(true);
   }

   public void doTransitionOutOfTerminalStance(RobotSide stanceSide)
   {
   }

   public void doTransitionOutOfUnloadLegToTransferIntoWalking(RobotSide stanceSide)
   {
      bipedFeetUpdater.setResizePolygonInDoubleSupport(false);
   }


   public void doTransitionOutOfLoadingForSingleLegBalance(RobotSide upcomingSupportSide)
   {
      // TODO Auto-generated method stub
   }

   public void doTransitionOutOfSingleLegBalance(RobotSide supportLeg)
   {
      desiredPelvisOrientationControlModule.setDesiredPelvisOrientation(null);
   }

   public boolean isReadyToStartStopWalkingDoubleSupport(RobotSide loadingLeg, double timeInState)
   {
      return (timeInState > minDoubleSupportTimeBeforeWalking.getDoubleValue());
   }

   public boolean isDoneUnloadLegToTransferIntoWalking(RobotSide loadingLeg, double timeInState)
   {
//      ReferenceFrame loadingLegZUpFrame = referenceFrames.getAnkleZUpFrame(loadingLeg);
//      FramePoint2d capturePoint = couplingRegistry.getCapturePointInFrame(loadingLegZUpFrame).toFramePoint2d();
//
//      boolean capturePointFarEnoughForward = capturePoint.getX() > xCaptureToTransfer.getDoubleValue();
//      boolean capturePointInside;
//      if (loadingLeg == RobotSide.LEFT)
//         capturePointInside = capturePoint.getY() > -yCaptureToTransfer.getDoubleValue();
//      else
//         capturePointInside = capturePoint.getY() < yCaptureToTransfer.getDoubleValue();
//
//      return (capturePointFarEnoughForward && capturePointInside);

      return isOverPercentageTowardDesired(loadingLeg, 0.9);
   }

   public boolean isDoneWithLoadingPreSwingA(RobotSide loadingLeg, double timeInState)
   {
      FrameConvexPolygon2d loadingFootPolygon = couplingRegistry.getOldBipedSupportPolygons().getFootPolygonInAnkleZUp(loadingLeg);
      FramePoint2d desiredCoP = couplingRegistry.getDesiredCoP();
      desiredCoP.changeFrame(loadingFootPolygon.getReferenceFrame());      
      boolean copInsideLoadingFootPolygon = loadingFootPolygon.isPointInside(desiredCoP);

      FrameConvexPolygon2d supportPolygon = couplingRegistry.getOldBipedSupportPolygons().getSupportPolygonInMidFeetZUp();
      FramePoint2d capturePoint = couplingRegistry.getCapturePointInFrame(supportPolygon.getReferenceFrame()).toFramePoint2d();
      boolean icpOutsideSupportPolygon = !supportPolygon.isPointInside(capturePoint);

      return copInsideLoadingFootPolygon || icpOutsideSupportPolygon;
//      return isOverPercentageTowardDesired(loadingLeg, minPercentageTowardsDesired.getDoubleValue());
   }

   private boolean isOverPercentageTowardDesired(RobotSide loadingLeg, double minPercentageTowardsDesired)
   {
      ReferenceFrame loadingLegZUpFrame = referenceFrames.getAnkleZUpFrame(loadingLeg);
      FramePoint2d capturePoint = couplingRegistry.getCapturePointInFrame(loadingLegZUpFrame).toFramePoint2d();

      FramePoint2d unLoadingSweetSpot = couplingRegistry.getOldBipedSupportPolygons().getSweetSpotCopy(loadingLeg.getOppositeSide());
      FramePoint2d loadingSweetSpot = couplingRegistry.getOldBipedSupportPolygons().getSweetSpotCopy(loadingLeg);

      unLoadingSweetSpot.changeFrame(loadingLegZUpFrame);
      loadingSweetSpot.changeFrame(loadingLegZUpFrame);

      FrameLineSegment2d footToFoot = new FrameLineSegment2d(unLoadingSweetSpot, loadingSweetSpot);

//    FramePoint2d projectedCapturePoint = footToFoot.orthogonalProjectionCopy(capturePoint);
//    double distanceFromLineSegment = projectedCapturePoint.distance(capturePoint);
      double percentageAlongLineSegment = footToFoot.percentageAlongLineSegment(capturePoint);

      return percentageAlongLineSegment > minPercentageTowardsDesired;
   }

   public boolean isDoneWithLoadingPreSwingB(RobotSide loadingLeg, double timeInState)
   {
      return true;
   }

   public boolean isDoneLoadingForSingleLegBalance(RobotSide upcomingSupportSide, double timeInCurrentState)
   {
      FramePoint2d sweetSpot = couplingRegistry.getOldBipedSupportPolygons().getSweetSpotCopy(upcomingSupportSide);
      FramePoint2d capturePoint = couplingRegistry.getCapturePointInFrame(sweetSpot.getReferenceFrame()).toFramePoint2d();

      double doneLoadingForSingleLegBalanceSafetyFactor = 4.0;
      double minDistanceToSweetSpot = footWidth / doneLoadingForSingleLegBalanceSafetyFactor;
      boolean isCapturePointCloseEnoughToSweetSpot = capturePoint.distance(sweetSpot) < minDistanceToSweetSpot;
      boolean inStateLongEnough = timeInCurrentState > 1.0;

      return isCapturePointCloseEnoughToSweetSpot && inStateLongEnough;
   }

   @SuppressWarnings("unused")
   private boolean isCapturePointOutsideBaseOfSupport()
   {
      FrameConvexPolygon2d supportPolygon = couplingRegistry.getOldBipedSupportPolygons().getSupportPolygonInMidFeetZUp();
      FramePoint2d capturePoint = couplingRegistry.getCapturePointInFrame(supportPolygon.getReferenceFrame()).toFramePoint2d();
      boolean hasCapturePointLeftBaseOfSupport = !(supportPolygon.isPointInside(capturePoint));

      return hasCapturePointLeftBaseOfSupport;
   }

   private final FrameVector2d desiredVelocity = new FrameVector2d();
   
   private void doSingleSupportControl(LegTorques legTorquesToPackForStanceSide, SingleSupportCondition singleSupportCondition, double timeInState)
   {      
      if (singleSupportCondition == SingleSupportCondition.StopWalking)
      {
         desiredVelocity.setToZero(ReferenceFrame.getWorldFrame());
      }
      else
      {
         desiredVelocityControlModule.getDesiredVelocity(desiredVelocity);
      }

      FrameOrientation desiredPelvisOrientation =
         desiredPelvisOrientationControlModule.getDesiredPelvisOrientationSingleSupportCopy(legTorquesToPackForStanceSide.getRobotSide());
      Wrench upperBodyWrench = couplingRegistry.getDesiredUpperBodyWrench();
      balanceSupportControlModule.doSingleSupportBalance(legTorquesToPackForStanceSide, desiredVelocity, desiredPelvisOrientation, upperBodyWrench, singleSupportCondition, timeInState);
   }

   private void doDoubleSupportControl(LowerBodyTorques lowerBodyTorquesToPack, RobotSide loadingLeg, boolean walk)
   {
      if (walk)
      {
         desiredVelocityControlModule.getDesiredVelocity(desiredVelocity);
      }
      else
      {
         desiredVelocityControlModule.getDesiredVelocity(desiredVelocity);
         desiredVelocity.setToZero();
      }
                                      
      FrameOrientation desiredPelvisOrientation = desiredPelvisOrientationControlModule.getDesiredPelvisOrientationDoubleSupportCopy();
      balanceSupportControlModule.doDoubleSupportBalance(lowerBodyTorquesToPack, loadingLeg, desiredVelocity, desiredPelvisOrientation);

//    balanceSupportControlModule.doDoubleSupportBalanceToeOff(lowerBodyTorquesToPack, loadingLeg, desiredVelocity, desiredPelvisOrientation);
   }

   public void doStopWalkingDoubleSupport(LowerBodyTorques lowerBodyTorquesToPack, RobotSide loadingLeg, double timeInState)
   {
      FrameVector zeroVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
      zeroVelocity.changeFrame(desiredHeadingControlModule.getDesiredHeadingFrame());

//    desiredVelocityControlModule.setDesiredVelocity(zeroVelocity);
      FrameOrientation desiredPelvisOrientation = desiredPelvisOrientationControlModule.getDesiredPelvisOrientationDoubleSupportCopy();
      balanceSupportControlModule.doDoubleSupportBalance(lowerBodyTorquesToPack, loadingLeg, zeroVelocity.toFrameVector2d(), desiredPelvisOrientation);
   }

// private static FramePoint2d getCenterOfBoundingBoxOfPolygon(FrameConvexPolygon2d supportPolygon)
// {
//    BoundingBox2d boundingBox = supportPolygon.getBoundingBox();
//    Point2d center = new Point2d();
//    boundingBox.getCenterPointCopy(center);
//    FramePoint2d doubleSupportCentroid = new FramePoint2d(ReferenceFrame.getWorldFrame(), center);
//
//    return doubleSupportCentroid;
// }

   public void setParametersForR2()
   {
      minDoubleSupportTimeBeforeWalking.set(0.3);
      toeOffFootPitch.set(0.1);    // 0.3);
      toeOffMoveDuration.set(0.05);
      minPercentageTowardsDesired.set(0.9);
   }

   private void setParametersForM2V2()
   {
      minDoubleSupportTimeBeforeWalking.set(0.3);
      toeOffFootPitch.set(0.1);    // 0.3);
      toeOffMoveDuration.set(0.05);
      minPercentageTowardsDesired.set(0.95);
   }
   
   public void setParametersForM2V2PushRecovery()
   {
      setParametersForM2V2();
      doPushRecovery.set(true);      
   }

   public void setParametersForM2V2Walking()
   {
      setParametersForM2V2();
      doPushRecovery.set(false);
   }

   public void initialize()
   {      
   }

   public boolean needToTakeAStep(RobotSide supportLeg)
   {
      if (doPushRecovery.getBooleanValue())
      {
         double epsilon = 1e-2;

         FrameConvexPolygon2d supportPolygon = couplingRegistry.getOldBipedSupportPolygons().getSupportPolygonInMidFeetZUp();
         FramePoint2d capturePoint = couplingRegistry.getCapturePointInFrame(supportPolygon.getReferenceFrame()).toFramePoint2d();
         return supportPolygon.distance(capturePoint) > epsilon;  
      }
      else
         return false;
   }

   private void setEstimatedDoubleSupportTimeRemaining(RobotSide supportLeg, boolean walking)
   {
      if (supportLeg == null)
      {
         if (walking)
         {
            double timeSpentInDoubleSupport = processedSensors.getTime() - loadingPreSwingAEntryTime.getDoubleValue();
            double estimatedDoubleSupportTimeRemaining = couplingRegistry.getDoubleSupportDuration() - timeSpentInDoubleSupport;
            estimatedDoubleSupportTimeRemaining = MathTools.clipToMinMax(estimatedDoubleSupportTimeRemaining, 0.0, Double.POSITIVE_INFINITY);
            couplingRegistry.setEstimatedDoubleSupportTimeRemaining(estimatedDoubleSupportTimeRemaining);            
         }
         else
            couplingRegistry.setEstimatedDoubleSupportTimeRemaining(Double.POSITIVE_INFINITY);
      }
      else
      {
         couplingRegistry.setEstimatedDoubleSupportTimeRemaining(0.0);
      }
   }
}
