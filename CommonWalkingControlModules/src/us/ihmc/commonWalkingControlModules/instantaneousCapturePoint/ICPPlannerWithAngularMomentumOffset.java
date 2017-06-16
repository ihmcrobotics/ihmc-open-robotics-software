package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class ICPPlannerWithAngularMomentumOffset extends ICPPlannerWithTimeFreezer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final String namePrefix = "icpPlanner";

   private final YoBoolean modifyICPPlanByAngularMomentum;

   private final YoFrameVector modifiedICPVelocity;
   private final YoFrameVector modifiedICPAcceleration;

   private final YoFramePoint modifiedCMPPosition;
   private final YoFrameVector modifiedCMPVelocity;

   private final FrameVector cmpOffsetFromCoP = new FrameVector();
   private final YoFrameVector cmpOffset;
   private final AlphaFilteredYoFrameVector filteredCMPOffset;

   private final YoDouble modifiedTimeInCurrentState;
   private final YoDouble modifiedTimeInCurrentStateRemaining;

   private final YoDouble angularMomentumRateForwardGain;
   private final YoDouble angularMomentumRateLateralGain;

   public ICPPlannerWithAngularMomentumOffset(BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                              CapturePointPlannerParameters capturePointPlannerParameters, YoVariableRegistry parentRegistry,
                                              YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(bipedSupportPolygons, contactableFeet, capturePointPlannerParameters, parentRegistry, yoGraphicsListRegistry);

      modifyICPPlanByAngularMomentum = new YoBoolean(namePrefix + "ModifyICPPlanByAngularMomentum", registry);
      modifyICPPlanByAngularMomentum.set(capturePointPlannerParameters.getModifyICPPlanByAngularMomentumRate());

      modifiedICPVelocity = new YoFrameVector(namePrefix + "ModifiedCapturePointVelocity", worldFrame, registry);
      modifiedICPAcceleration = new YoFrameVector(namePrefix + "ModifiedCapturePointAcceleration", worldFrame, registry);

      modifiedCMPPosition = new YoFramePoint(namePrefix + "ModifiedCMPPosition", worldFrame, registry);
      modifiedCMPVelocity = new YoFrameVector(namePrefix + "ModifiedCMPVelocity", worldFrame, registry);

      YoDouble cmpOffsetAlphaFilter = new YoDouble(namePrefix + "CMPOffsetAlphaFilter", registry);
      cmpOffsetAlphaFilter.set(capturePointPlannerParameters.getCMPOffsetAlphaFilter());

      cmpOffset = new YoFrameVector(namePrefix + "CMPOffset", worldFrame, registry);
      filteredCMPOffset = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector(namePrefix + "FilteredCMPOffset", "", registry,
                                                                                      cmpOffsetAlphaFilter, cmpOffset);

      modifiedTimeInCurrentState = new YoDouble(namePrefix + "ModifiedTimeInCurrentState", registry);
      modifiedTimeInCurrentStateRemaining = new YoDouble(namePrefix + "ModifiedRemainingTime", registry);

      angularMomentumRateForwardGain = new YoDouble(namePrefix + "AngularMomentumRateForwardGain", registry);
      angularMomentumRateLateralGain = new YoDouble(namePrefix + "AngularMomentumRateLateralGain", registry);
      angularMomentumRateForwardGain.set(capturePointPlannerParameters.getAngularMomentumRateForwardGain());
      angularMomentumRateLateralGain.set(capturePointPlannerParameters.getAngularMomentumRateLateralGain());
   }

   public void modifyDesiredICPForAngularMomentum(FramePoint copEstimate, RobotSide supportSide)
   {
      if (!modifyICPPlanByAngularMomentum.getBooleanValue() || copEstimate.containsNaN())
      {
         modifiedCMPPosition.set(desiredCMPPosition);
         modifiedCMPVelocity.set(desiredCMPVelocity);

         modifiedICPVelocity.set(desiredICPVelocity);
         modifiedICPAcceleration.set(desiredICPAcceleration);

         modifiedTimeInCurrentState.set(timeInCurrentState.getDoubleValue());
         modifiedTimeInCurrentStateRemaining.set(timeInCurrentStateRemaining.getDoubleValue());
      }
      else
      {
         desiredCMPPosition.getFrameTuple(cmpOffsetFromCoP);
         cmpOffsetFromCoP.sub(copEstimate);

         if (supportSide == null && transferToSide.getEnumValue() == null)
         {
            cmpOffsetFromCoP.scale(angularMomentumRateForwardGain.getDoubleValue());
         }
         else
         {
            ReferenceFrame soleFrame;
            if (supportSide != null)
               soleFrame = soleZUpFrames.get(supportSide);
            else
               soleFrame = soleZUpFrames.get(transferToSide.getEnumValue());

            cmpOffsetFromCoP.changeFrame(soleFrame);
            cmpOffsetFromCoP.setX(angularMomentumRateForwardGain.getDoubleValue() * cmpOffsetFromCoP.getX());
            cmpOffsetFromCoP.setY(angularMomentumRateLateralGain.getDoubleValue() * cmpOffsetFromCoP.getY());
            cmpOffsetFromCoP.changeFrame(worldFrame);
         }

         cmpOffset.set(cmpOffsetFromCoP);
         filteredCMPOffset.update();

         modifiedCMPPosition.set(desiredCMPPosition);
         modifiedCMPPosition.add(filteredCMPOffset);

         estimateCurrentTimeWithModifiedCMP(modifiedCMPPosition.getFrameTuple());

         double omega0 = this.omega0.getDoubleValue();
         modifiedICPVelocity.set(desiredICPPosition);
         modifiedICPVelocity.sub(modifiedCMPPosition);
         modifiedICPVelocity.scale(omega0);

         CapturePointTools.computeDesiredCapturePointAcceleration(omega0, modifiedICPVelocity, modifiedICPAcceleration);
         CapturePointTools.computeDesiredCentroidalMomentumPivotVelocity(modifiedICPVelocity, modifiedICPAcceleration, omega0, modifiedCMPVelocity);
      }
   }

   private void estimateCurrentTimeWithModifiedCMP(FramePoint desiredCoPFromAngularMomentum)
   {
      double copCMPDistance = desiredCMPPosition.getXYPlaneDistance(desiredCoPFromAngularMomentum);
      double distanceFromCMP = desiredICPPosition.getXYPlaneDistance(modifiedCMPPosition);

      double modifiedTimeInState = 1.0 / omega0.getDoubleValue() * Math.log(distanceFromCMP / copCMPDistance);
      modifiedTimeInCurrentState.set(modifiedTimeInState);
      modifiedTimeInCurrentStateRemaining.set(getCurrentStateDuration() - modifiedTimeInCurrentState.getDoubleValue());
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCapturePointVelocity(FrameVector desiredCapturePointVelocityToPack)
   {
      modifiedICPVelocity.getFrameTupleIncludingFrame(desiredCapturePointVelocityToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCapturePointVelocity(FrameVector2d desiredCapturePointVelocityToPack)
   {
      modifiedICPVelocity.getFrameTuple2dIncludingFrame(desiredCapturePointVelocityToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCapturePointVelocity(YoFrameVector desiredCapturePointVelocityToPack)
   {
      desiredCapturePointVelocityToPack.set(modifiedICPVelocity);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCentroidalMomentumPivotPosition(FramePoint desiredCentroidalMomentumPivotPositionToPack)
   {
      modifiedCMPPosition.getFrameTupleIncludingFrame(desiredCentroidalMomentumPivotPositionToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCentroidalMomentumPivotPosition(FramePoint2d desiredCentroidalMomentumPivotPositionToPack)
   {
      modifiedCMPPosition.getFrameTuple2dIncludingFrame(desiredCentroidalMomentumPivotPositionToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCentroidalMomentumPivotVelocity(FrameVector desiredCentroidalMomentumPivotVelocityToPack)
   {
      modifiedCMPVelocity.getFrameTupleIncludingFrame(desiredCentroidalMomentumPivotVelocityToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCentroidalMomentumPivotVelocity(FrameVector2d desiredCentroidalMomentumPivotVelocityToPack)
   {
      modifiedCMPVelocity.getFrameTuple2dIncludingFrame(desiredCentroidalMomentumPivotVelocityToPack);
   }

   /** {@inheritDoc} */
   @Override
   public double getTimeInCurrentState()
   {
      return modifiedTimeInCurrentState.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getTimeInCurrentStateRemaining()
   {
      return modifiedTimeInCurrentStateRemaining.getDoubleValue();
   }
}
