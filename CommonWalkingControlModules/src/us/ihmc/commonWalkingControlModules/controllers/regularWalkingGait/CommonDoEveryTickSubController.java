package us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedFeetUpdater;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedFootInterface;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.captureRegion.CapturePointCalculatorInterface;
import us.ihmc.commonWalkingControlModules.captureRegion.CaptureRegionCalculator;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredVelocityControlModule;
import us.ihmc.commonWalkingControlModules.desiredStepLocation.DesiredStepLocationCalculator;
import us.ihmc.commonWalkingControlModules.desiredStepLocation.Footstep;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class CommonDoEveryTickSubController implements DoEveryTickSubController
{
   private final CommonWalkingReferenceFrames referenceFrames;
   private final ProcessedSensorsInterface processedSensors;

   private final BipedFootInterface leftFoot;
   private final BipedFootInterface rightFoot;
   private final BipedFeetUpdater bipedFeetUpdater;

   private final DesiredHeadingControlModule desiredHeadingControlModule;
   private final DesiredVelocityControlModule desiredVelocityControlModule;

   private final DesiredStepLocationCalculator desiredStepLocationCalculator;

   private final CapturePointCalculatorInterface capturePointCalculator;
   private final CaptureRegionCalculator captureRegionCalculator;
   private final CouplingRegistry couplingRegistry; 

   public CommonDoEveryTickSubController(CommonWalkingReferenceFrames referenceFrames, ProcessedSensorsInterface processedSensors, BipedFootInterface leftFoot,
           BipedFootInterface rightFoot, BipedFeetUpdater bipedFeetUpdater,
           DesiredHeadingControlModule desiredHeadingControlModule, DesiredVelocityControlModule desiredVelocityControlModule,
           DesiredStepLocationCalculator desiredStepLocationCalculator, CapturePointCalculatorInterface capturePointCalculator, CaptureRegionCalculator captureRegionCalculator, CouplingRegistry couplingRegistry)
   {
      this.referenceFrames = referenceFrames;
      this.processedSensors = processedSensors;

      this.leftFoot = leftFoot;
      this.rightFoot = rightFoot;
      this.bipedFeetUpdater = bipedFeetUpdater;

      this.desiredHeadingControlModule = desiredHeadingControlModule;
      this.desiredVelocityControlModule = desiredVelocityControlModule;
      this.desiredStepLocationCalculator = desiredStepLocationCalculator;

      this.capturePointCalculator = capturePointCalculator;
      this.captureRegionCalculator = captureRegionCalculator;
      this.couplingRegistry = couplingRegistry;
   }


   public void doEveryControlTick(RobotSide supportLeg)
   {
      desiredVelocityControlModule.updateDesiredVelocity();
      FrameVector2d desiredVelocity = desiredVelocityControlModule.getDesiredVelocity();
      couplingRegistry.setDesiredVelocity(desiredVelocity);
      desiredHeadingControlModule.updateDesiredHeadingFrame();

      // Compute the instantaneous capture point.
      capturePointCalculator.computeCapturePoint(supportLeg);
      FramePoint capturePointInMidfeetZUp = capturePointCalculator.getCapturePointInFrame(referenceFrames.getMidFeetZUpFrame());
      couplingRegistry.setCapturePoint(capturePointInMidfeetZUp);

      boolean forceHindOnToes = false;

//    boolean forceHindOnToes = couplingRegistry.getForceHindOnToes();

      bipedFeetUpdater.updateBipedFeet(leftFoot, rightFoot, supportLeg, capturePointInMidfeetZUp, forceHindOnToes);

      BipedSupportPolygons bipedSupportPolygons = couplingRegistry.getBipedSupportPolygons();
      bipedSupportPolygons.update(leftFoot, rightFoot);

      if (supportLeg != null)
      {
         // TODO: also compute capture regions in double support

         FrameConvexPolygon2d supportFoot = bipedSupportPolygons.getFootPolygonInAnkleZUp(supportLeg);
         FrameConvexPolygon2d captureRegion = captureRegionCalculator.calculateCaptureRegion(supportLeg, supportFoot,
                                                 couplingRegistry.getEstimatedSwingTimeRemaining());
         couplingRegistry.setCaptureRegion(captureRegion);

//       ReferenceFrame desiredHeadingFrame = desiredHeadingControlModule.getDesiredHeadingFrame();
         Footstep desiredStepLocation = desiredStepLocationCalculator.computeDesiredStepLocation(supportLeg, bipedSupportPolygons, captureRegion,
                                           couplingRegistry.getCapturePoint());

//       Footstep desiredStepLocation = desiredStepLocationCalculator.getDesiredStepLocation(supportLeg.getOppositeSide(), couplingRegistry);
         couplingRegistry.setDesiredFootstep(desiredStepLocation);
      }
      else
      {
         captureRegionCalculator.hideCaptureRegion();
         couplingRegistry.setCaptureRegion(null);
         couplingRegistry.setDesiredFootstep(null);
      }
   }


   public void doFirstTick()
   {
      double currentHeading = processedSensors.getPelvisOrientationInFrame(ReferenceFrame.getWorldFrame()).getYawPitchRoll()[0];
      desiredHeadingControlModule.resetHeading(currentHeading);
   }
}
