package us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait;

import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DesiredPelvisOrientationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.BalanceSupportControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.FootOrientationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.KneeExtensionControlModule;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredVelocityControlModule;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LowerBodyTorques;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.BoundingBox2d;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.Orientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class CommonStanceSubController implements StanceSubController
{
   private final CouplingRegistry couplingRegistry;
   private final CommonWalkingReferenceFrames referenceFrames;
   private final DesiredVelocityControlModule desiredVelocityControlModule;
   private final DesiredPelvisOrientationControlModule desiredPelvisOrientationControlModule;
   private final BalanceSupportControlModule balanceSupportControlModule;
   private final FootOrientationControlModule footOrientationControlModule;
   private final DesiredHeadingControlModule desiredHeadingControlModule;
   private final KneeExtensionControlModule kneeExtensionControlModule;

   private final YoVariableRegistry registry = new YoVariableRegistry("StanceSubController");

   private final DoubleYoVariable minDoubleSupportTime = new DoubleYoVariable("minDoubleSupportTime", "Time to stay in double support.", registry);
   private final DoubleYoVariable captureXToFinishDoubleSupport =
      new DoubleYoVariable("captureXToFinishDoubleSupport",
                           "Finish double support once the capture point x value is greater than this value in front of your upcoming support ankle.",
                           registry);

   private final DoubleYoVariable kVelocityDoubleSupportTransfer =
      new DoubleYoVariable("kVelocityDoubleSupportTransfer", "Gain from velocity error to amount of capture point motion to extend double support phase.",
                           registry);

   private final DoubleYoVariable toeOffFootPitch = new DoubleYoVariable("toeOffFootPitch",
                                                       "This is the desired foot pitch at the end of toe-off during stance", registry);
   private final DoubleYoVariable toeOffMoveDuration = new DoubleYoVariable("toeOffMoveDuration", "The duration of the toe-off move during stance", registry);
   private final DoubleYoVariable timeSpentInEarlyStance = new DoubleYoVariable("timeSpentInEarlyStance", registry);
   private final DoubleYoVariable timeSpentInLateStance = new DoubleYoVariable("timeSpentInLateStance", registry);
   private final DoubleYoVariable timeSpentInTerminalStance = new DoubleYoVariable("timeSpentInTerminalStance", registry);

   private final DoubleYoVariable timeSpentInLoadingPreSwingA = new DoubleYoVariable("timeSpentInLoadingPreSwingA", registry);
   private final DoubleYoVariable timeSpentInLoadingPreSwingB = new DoubleYoVariable("timeSpentInLoadingPreSwingB", registry);
   private final DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration", registry);

   private final DoubleYoVariable minDoubleSupportTimeBeforeWalking = new DoubleYoVariable("minDoubleSupportTimeBeforeWalking", registry);
   private final DoubleYoVariable yCaptureToTransfer = new DoubleYoVariable("yCaptureToTransfer", registry);
   private final DoubleYoVariable minCaptureXToFinishDoubleSupport = new DoubleYoVariable("minCaptureXToFinishDoubleSupport", registry);
   private final DoubleYoVariable maxCaptureXToFinishDoublesupport = new DoubleYoVariable("maxCaptureXToFinishDoublesupport", registry);
   private final DoubleYoVariable baseCaptureXToFinishDoubleSupport = new DoubleYoVariable("baseCaptureXToFinishDoubleSupport", registry);
   private final DoubleYoVariable captureXVelocityScale = new DoubleYoVariable("captureXVelocityScale", registry);

   private boolean WAIT_IN_LOADING_PRE_SWING_B;


   public CommonStanceSubController(CouplingRegistry couplingRegistry, CommonWalkingReferenceFrames referenceFrames,
                                    DesiredHeadingControlModule desiredHeadingControlModule, DesiredVelocityControlModule desiredVelocityControlModule,
                                    DesiredPelvisOrientationControlModule desiredPelvisOrientationControlModule,
                                    BalanceSupportControlModule balanceSupportControlModule, FootOrientationControlModule footOrientationControlModule,
                                    KneeExtensionControlModule kneeExtensionControlModule, YoVariableRegistry parentRegistry)
   {
      this.couplingRegistry = couplingRegistry;
      this.referenceFrames = referenceFrames;
      this.desiredVelocityControlModule = desiredVelocityControlModule;
      this.desiredPelvisOrientationControlModule = desiredPelvisOrientationControlModule;
      this.balanceSupportControlModule = balanceSupportControlModule;
      this.footOrientationControlModule = footOrientationControlModule;
      this.desiredHeadingControlModule = desiredHeadingControlModule;
      this.kneeExtensionControlModule = kneeExtensionControlModule;

      doubleSupportDuration.set(0.75);    // FIXME: This is a hack but allows to compute the first desired step
      couplingRegistry.setDoubleSupportDuration(doubleSupportDuration);

      parentRegistry.addChild(registry);
   }

   public boolean canWeStopNowStanceSubController()
   {
      return true;
   }

   public void doEarlyStance(LegTorques legTorquesToPackForStanceSide, double timeInState)
   {
      balanceSupportControlModule.unSetPutWeightOnToes(legTorquesToPackForStanceSide.getRobotSide());
      doSingleSupportControl(legTorquesToPackForStanceSide);

      // Here is where we want to add the torque for the kneeExtensionController
      kneeExtensionControlModule.doEarlyStanceKneeExtensionControl(legTorquesToPackForStanceSide);
      timeSpentInEarlyStance.set(timeInState);
   }

   public void doLateStance(LegTorques legTorquesToPackForStanceSide, double timeInState)
   {
      RobotSide stanceLeg = legTorquesToPackForStanceSide.getRobotSide();
      balanceSupportControlModule.setPutWeightOnToes(stanceLeg);

      doSingleSupportControl(legTorquesToPackForStanceSide);
      footOrientationControlModule.addAdditionalTorqueForFootOrientationControl(legTorquesToPackForStanceSide, timeInState);
      kneeExtensionControlModule.doLateStanceKneeExtensionControl(legTorquesToPackForStanceSide);


      timeSpentInLateStance.set(timeInState);
   }

   public void doTerminalStance(LegTorques legTorquesToPackForLoadingLeg, double timeInState)
   {
      RobotSide loadingLeg = legTorquesToPackForLoadingLeg.getRobotSide();
      balanceSupportControlModule.setPutWeightOnToes(loadingLeg);

//    doSingleSupportControl(legTorquesToPackForStanceSide);

      doTerminalSingleSupportControl(legTorquesToPackForLoadingLeg, timeInState);

      footOrientationControlModule.addAdditionalTorqueForFootOrientationControl(legTorquesToPackForLoadingLeg,
              timeInState + timeSpentInLateStance.getDoubleValue());

      kneeExtensionControlModule.breakKneeForDownhillSlopes(loadingLeg.getOppositeSide());

      timeSpentInTerminalStance.set(timeInState);
   }


   public void doLoadingPreSwingA(LowerBodyTorques lowerBodyTorquesToPack, RobotSide loadingLeg, double timeInState)
   {
      balanceSupportControlModule.unSetPutWeightOnToes(loadingLeg);

      doDoubleSupportControl(lowerBodyTorquesToPack, loadingLeg);

      if ((timeSpentInLateStance.getDoubleValue() + timeSpentInTerminalStance.getDoubleValue()) != 0.0)
         footOrientationControlModule.addAdditionalTorqueForFootOrientationControl(lowerBodyTorquesToPack.getLegTorques(loadingLeg.getOppositeSide()),
                 timeInState + timeSpentInLateStance.getDoubleValue() + timeSpentInTerminalStance.getDoubleValue());

      kneeExtensionControlModule.doLoadingControl(lowerBodyTorquesToPack.getLegTorques(loadingLeg));

      timeSpentInLoadingPreSwingA.set(timeInState);
   }

   public void doLoadingPreSwingB(LowerBodyTorques lowerBodyTorquesToPack, RobotSide loadingLeg, double timeInState)
   {
      balanceSupportControlModule.unSetPutWeightOnToes(loadingLeg);

      doDoubleSupportControl(lowerBodyTorquesToPack, loadingLeg);
      kneeExtensionControlModule.doLoadingControl(lowerBodyTorquesToPack.getLegTorques(loadingLeg));

      timeSpentInLoadingPreSwingB.set(timeInState);
   }

   public void doLoadingPreSwingC(LegTorques legTorquesToPackForStanceSide, RobotSide loadingLeg, double timeInState)
   {
      balanceSupportControlModule.unSetPutWeightOnToes(loadingLeg);

      doSingleSupportControl(legTorquesToPackForStanceSide);
      kneeExtensionControlModule.doLoadingControl(legTorquesToPackForStanceSide);
   }

   public void doStartWalkingDoubleSupport(LowerBodyTorques lowerBodyTorquesToPack, RobotSide loadingLeg, double timeInState)
   {
      doDoubleSupportControl(lowerBodyTorquesToPack, loadingLeg);
   }

   public void doTransitionIntoEarlyStance(RobotSide stanceSide)
   {
      kneeExtensionControlModule.doTransitionIntoStance(stanceSide);
   }

   public void doTransitionIntoLateStance(RobotSide stanceSide)
   {
      ReferenceFrame desiredHeadingFrame = desiredHeadingControlModule.getDesiredHeadingFrame();
      Orientation finalOrientation = new Orientation(desiredHeadingFrame, 0.0, toeOffFootPitch.getDoubleValue(), 0.0);
      footOrientationControlModule.initializeFootOrientationMove(toeOffMoveDuration.getDoubleValue(), finalOrientation, stanceSide);

      timeSpentInLateStance.set(0.0);
   }

   public void doTransitionIntoTerminalStance(RobotSide stanceSide)
   {
      timeSpentInTerminalStance.set(0.0);
   }

   public void doTransitionIntoLoadingPreSwingA(RobotSide loadingLeg)
   {
      kneeExtensionControlModule.doTransitionIntoLoading(loadingLeg);

      // Reset the timers
      timeSpentInLoadingPreSwingA.set(0.0);
      timeSpentInLoadingPreSwingB.set(0.0);
      doubleSupportDuration.set(0.0);
   }

   public void doTransitionIntoLoadingPreSwingB(RobotSide loadingLeg)
   {
   }

   public void doTransitionIntoLoadingPreSwingC(RobotSide loadingLeg)
   {
   }

   public void doTransitionIntoStartStopWalkingDoubleSupport(RobotSide stanceSide)
   {
      balanceSupportControlModule.setDesiredCoPOffset(new FramePoint2d(ReferenceFrame.getWorldFrame()));
   }

   public void doTransitionIntoUnloadLegToTransferIntoWalking(RobotSide stanceSide)
   {
      BipedSupportPolygons bipedSupportPolygons = couplingRegistry.getBipedSupportPolygons();

      FrameConvexPolygon2d singleSupportFootPolygon = bipedSupportPolygons.getFootPolygonInAnkleZUp(stanceSide);    // processedSensors.getFootPolygons().get(stanceSide).getFrameConvexPolygon2dCopy();
      FramePoint2d singleSupportCentroid = singleSupportFootPolygon.getCentroidCopy();

      FrameConvexPolygon2d supportPolygon = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();
      FramePoint2d doubleSupportCentroid = getCenterOfBoundingBoxOfPolygon(supportPolygon).changeFrameCopy(singleSupportCentroid.getReferenceFrame());

      singleSupportCentroid.sub(doubleSupportCentroid);

      balanceSupportControlModule.setDesiredCoPOffset(singleSupportCentroid);
   }

   public void doTransitionOutOfEarlyStance(RobotSide stanceSide)
   {
   }

   public void doTransitionOutOfLateStance(RobotSide stanceSide)
   {
   }

   public void doTransitionOutOfLoadingPreSwingA(RobotSide loadingLeg)
   {
   }

   public void doTransitionOutOfLoadingPreSwingB(RobotSide loadingLeg)
   {
      doubleSupportDuration.set(timeSpentInLoadingPreSwingA.getDoubleValue() + timeSpentInLoadingPreSwingB.getDoubleValue());
      couplingRegistry.setDoubleSupportDuration(doubleSupportDuration);
   }

   public void doTransitionOutOfLoadingPreSwingC(RobotSide loadingLeg)
   {
   }

   public void doTransitionOutOfStartStopWalkingDoubleSupport(RobotSide stanceSide)
   {
   }

   public void doTransitionOutOfTerminalStance(RobotSide stanceSide)
   {
//    doubleSupportDuration =
   }

   public void doTransitionOutOfUnloadLegToTransferIntoWalking(RobotSide stanceSide)
   {
   }

   public void doUnloadLegToTransferIntoWalking(LowerBodyTorques lowerBodyTorquesToPack, RobotSide supportLeg, double timeInState)
   {
      doDoubleSupportControl(lowerBodyTorquesToPack, supportLeg);
   }

   public RobotSide getUpcomingSupportLegForStartingToWalkFromDoubleSupport()
   {
      // TODO Auto-generated method stub
      return null;
   }

   public boolean isReadyToStartStopWalkingDoubleSupport(RobotSide loadingLeg, double timeInState)
   {
      return (timeInState > minDoubleSupportTimeBeforeWalking.getDoubleValue());
   }

   public boolean isDoneUnloadLegToTransferIntoWalking(RobotSide loadingLeg, double timeInState)
   {
      return isDoneWithLoadingPreSwingA(loadingLeg, timeInState);

//    processedSensors.getReferenceFrames();
//    FramePoint actualCop = processedSensors.getCombinedCenterOfPressureInFrame(R2ReferenceFrames.getWorldFrame());
//
//    FrameConvexPolygon2d supportFootPolygon = processedSensors.getFootPolygons().get(loadingLeg).getFrameConvexPolygon2dCopy();
//    FrameConvexPolygon2d shrunkenSupportFootPolygon = FrameConvexPolygon2d.shrinkConstantDistanceInto(0.02, supportFootPolygon);
//
////  double comVelocityX = processedSensors.getCenterOfMassVelocityInFrame(ReferenceFrame.getWorldFrame()).getX();
////  double comVelocityY = processedSensors.getCenterOfMassVelocityInFrame(ReferenceFrame.getWorldFrame()).getY();
////  double comVelocityZ = processedSensors.getCenterOfMassVelocityInFrame(ReferenceFrame.getWorldFrame()).getZ();
//
////  double comVelocity = Math.sqrt(comVelocityX * comVelocityX + comVelocityY * comVelocityY + comVelocityZ * comVelocityZ);
//
//    if (shrunkenSupportFootPolygon.isPointInside(actualCop.toFramePoint2d()))
//       return true;
//    else
//       return false;

   }

   public boolean isDoneWithLoadingPreSwingA(RobotSide loadingLeg, double timeInState)
   {
      boolean inStateLongEnough = (timeInState > minDoubleSupportTime.getDoubleValue());

      ReferenceFrame loadingLegZUpFrame = referenceFrames.getAnkleZUpFrame(loadingLeg);

      FramePoint2d capturePoint = couplingRegistry.getCapturePoint().toFramePoint2d().changeFrameCopy(loadingLegZUpFrame);

      boolean capturePointPastAnkle = capturePoint.getX() > 0.0;

      FrameVector2d desiredVelocity = desiredVelocityControlModule.getDesiredVelocity().changeFrameCopy(loadingLegZUpFrame);
      FrameVector2d velocityError = desiredVelocityControlModule.getVelocityErrorInFrame(loadingLegZUpFrame);
      captureXToFinishDoubleSupport.set(getCaptureXToFinishDoubleSupport(desiredVelocity, velocityError));
      boolean capturePointIsFarEnoughForward = capturePoint.getX() > captureXToFinishDoubleSupport.getDoubleValue();

      boolean capturePointIsInside;
      if (loadingLeg == RobotSide.LEFT)
         capturePointIsInside = capturePoint.getY() > -yCaptureToTransfer.getDoubleValue();
      else
         capturePointIsInside = capturePoint.getY() < yCaptureToTransfer.getDoubleValue();

      if (!inStateLongEnough)
         return false;

      if (capturePointIsFarEnoughForward)
         return true;

      if (capturePointPastAnkle && capturePointIsInside)
         return true;

      return false;
   }

   private double getCaptureXToFinishDoubleSupport(FrameVector2d desiredVelocity, FrameVector2d velocityError)
   {
      desiredVelocity.checkReferenceFrameMatch(velocityError);
      double ret = baseCaptureXToFinishDoubleSupport.getDoubleValue() + captureXVelocityScale.getDoubleValue() * desiredVelocity.getX()
                   + velocityError.getX() * kVelocityDoubleSupportTransfer.getDoubleValue();

      double min = minCaptureXToFinishDoubleSupport.getDoubleValue();
      double max = maxCaptureXToFinishDoublesupport.getDoubleValue();

      return MathTools.clipToMinMax(ret, min, max);
   }

   public boolean isDoneWithLoadingPreSwingB(RobotSide loadingLeg, double timeInState)
   {
      if (WAIT_IN_LOADING_PRE_SWING_B)
         return timeInState > 0.25;
      else
         return true;
   }

   private void doSingleSupportControl(LegTorques legTorquesToPackForStanceSide)
   {
      FrameVector2d desiredVelocity = desiredVelocityControlModule.getDesiredVelocity();
      Orientation desiredPelvisOrientation =
         desiredPelvisOrientationControlModule.getDesiredPelvisOrientationSingleSupport(legTorquesToPackForStanceSide.getRobotSide());
      balanceSupportControlModule.doSingleSupportBalance(legTorquesToPackForStanceSide, desiredVelocity, desiredPelvisOrientation);
   }

   private void doTerminalSingleSupportControl(LegTorques legTorquesToPackForStanceSide, double timeInTerminalState)
   {
      FrameVector2d desiredVelocity = desiredVelocityControlModule.getDesiredVelocity();
      Orientation desiredPelvisOrientation =
         desiredPelvisOrientationControlModule.getDesiredPelvisOrientationSingleSupport(legTorquesToPackForStanceSide.getRobotSide());
      balanceSupportControlModule.doTerminalSingleSupportBalance(legTorquesToPackForStanceSide, desiredVelocity, desiredPelvisOrientation, timeInTerminalState);
   }

   private void doDoubleSupportControl(LowerBodyTorques lowerBodyTorquesToPack, RobotSide loadingLeg)
   {
      FrameVector2d desiredVelocity = desiredVelocityControlModule.getDesiredVelocity();
      Orientation desiredPelvisOrientation = desiredPelvisOrientationControlModule.getDesiredPelvisOrientationDoubleSupport();
      balanceSupportControlModule.doDoubleSupportBalance(lowerBodyTorquesToPack, loadingLeg, desiredVelocity, desiredPelvisOrientation);

//    balanceSupportControlModule.doDoubleSupportBalanceToeOff(lowerBodyTorquesToPack, loadingLeg, desiredVelocity, desiredPelvisOrientation);
   }

   public void doStopWalkingDoubleSupport(LowerBodyTorques lowerBodyTorquesToPack, RobotSide loadingLeg, double timeInState)
   {
      FrameVector zeroVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
      zeroVelocity = zeroVelocity.changeFrameCopy(desiredHeadingControlModule.getDesiredHeadingFrame());

//    desiredVelocityControlModule.setDesiredVelocity(zeroVelocity);
      Orientation desiredPelvisOrientation = desiredPelvisOrientationControlModule.getDesiredPelvisOrientationDoubleSupport();
      balanceSupportControlModule.doDoubleSupportBalance(lowerBodyTorquesToPack, loadingLeg, zeroVelocity.toFrameVector2d(), desiredPelvisOrientation);
   }

   private static FramePoint2d getCenterOfBoundingBoxOfPolygon(FrameConvexPolygon2d supportPolygon)
   {
      BoundingBox2d boundingBox = supportPolygon.getBoundingBox();
      Point2d center = new Point2d();
      boundingBox.getCenterPointCopy(center);
      FramePoint2d doubleSupportCentroid = new FramePoint2d(ReferenceFrame.getWorldFrame(), center);

      return doubleSupportCentroid;
   }

   public void setParametersForR2()
   {
      minDoubleSupportTime.set(0.05);
      minDoubleSupportTimeBeforeWalking.set(0.3);
      yCaptureToTransfer.set(0.04);    // 0.0;
      minCaptureXToFinishDoubleSupport.set(0.0);    // 0.03);
      maxCaptureXToFinishDoublesupport.set(0.16);    // 20);
      baseCaptureXToFinishDoubleSupport.set(0.03);    // 0.08);
      captureXVelocityScale.set(0.08);
      kVelocityDoubleSupportTransfer.set(0.05);    // 0.1);
      toeOffFootPitch.set(0.1);    // 0.3);
      toeOffMoveDuration.set(0.05);
      WAIT_IN_LOADING_PRE_SWING_B = false;
   }

   public void setParametersForM2V2()
   {
      // TODO: tune
      minDoubleSupportTime.set(0.05);
      minDoubleSupportTimeBeforeWalking.set(0.3);
      yCaptureToTransfer.set(0.04);    // 0.0;
      minCaptureXToFinishDoubleSupport.set(0.03);
      maxCaptureXToFinishDoublesupport.set(0.20);
      baseCaptureXToFinishDoubleSupport.set(0.08);
      captureXVelocityScale.set(0.08);
      kVelocityDoubleSupportTransfer.set(0.1);
      toeOffFootPitch.set(0.1);    // 0.3);
      toeOffMoveDuration.set(0.05);
      WAIT_IN_LOADING_PRE_SWING_B = true;
   }
}
