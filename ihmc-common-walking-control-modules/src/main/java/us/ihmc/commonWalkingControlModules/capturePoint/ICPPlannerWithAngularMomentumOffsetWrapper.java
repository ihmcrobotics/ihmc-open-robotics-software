package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.commonWalkingControlModules.configurations.ICPAngularMomentumModifierParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPTimeFreezerParameters;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class ICPPlannerWithAngularMomentumOffsetWrapper extends ICPPlannerWithTimeFreezerWrapper implements ICPPlannerWithAngularMomentumOffsetInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String namePrefix = "icpPlanner";

   private final YoBoolean modifyICPPlanByAngularMomentum;

   private final YoFrameVector3D modifiedICPVelocity;
   private final YoFrameVector3D modifiedICPAcceleration;

   private final YoFramePoint3D modifiedCMPPosition;
   private final YoFrameVector3D modifiedCMPVelocity;

   private final FrameVector3D cmpOffsetFromCoP = new FrameVector3D();
   private final YoFrameVector3D cmpOffset;
   private final AlphaFilteredYoFrameVector filteredCMPOffset;

   private final YoDouble modifiedTimeInCurrentState;
   private final YoDouble modifiedTimeInCurrentStateRemaining;

   private final YoDouble angularMomentumRateForwardGain;
   private final YoDouble angularMomentumRateLateralGain;

   private final YoDouble cmpOffsetAlphaFilter;

   private final SideDependentList<? extends ReferenceFrame> soleZUpFrames;

   public ICPPlannerWithAngularMomentumOffsetWrapper(ICPPlannerInterface icpPlanner, SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                                     ICPTimeFreezerParameters icpTimeFreezerParameters, ICPAngularMomentumModifierParameters angularMomentumModifierParameters)
   {
      super(icpPlanner, icpTimeFreezerParameters);

      this.soleZUpFrames = soleZUpFrames;

      modifyICPPlanByAngularMomentum = new YoBoolean(namePrefix + "ModifyICPPlanByAngularMomentum", registry);

      modifiedICPVelocity = new YoFrameVector3D(namePrefix + "ModifiedCapturePointVelocity", worldFrame, registry);
      modifiedICPAcceleration = new YoFrameVector3D(namePrefix + "ModifiedCapturePointAcceleration", worldFrame, registry);

      modifiedCMPPosition = new YoFramePoint3D(namePrefix + "ModifiedCMPPosition", worldFrame, registry);
      modifiedCMPVelocity = new YoFrameVector3D(namePrefix + "ModifiedCMPVelocity", worldFrame, registry);

      cmpOffsetAlphaFilter = new YoDouble(namePrefix + "CMPOffsetAlphaFilter", registry);

      cmpOffset = new YoFrameVector3D(namePrefix + "CMPOffset", worldFrame, registry);
      filteredCMPOffset = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector(namePrefix + "FilteredCMPOffset", "", registry,
                                                                                      cmpOffsetAlphaFilter, cmpOffset);

      modifiedTimeInCurrentState = new YoDouble(namePrefix + "ModifiedTimeInCurrentState", registry);
      modifiedTimeInCurrentStateRemaining = new YoDouble(namePrefix + "ModifiedRemainingTime", registry);

      angularMomentumRateForwardGain = new YoDouble(namePrefix + "AngularMomentumRateForwardGain", registry);
      angularMomentumRateLateralGain = new YoDouble(namePrefix + "AngularMomentumRateLateralGain", registry);

      initializeAngularMomentumParameters(angularMomentumModifierParameters);
   }

   private void initializeAngularMomentumParameters(ICPAngularMomentumModifierParameters angularMomentumModifierParameters)
   {
      if (angularMomentumModifierParameters != null)
      {
         modifyICPPlanByAngularMomentum.set(angularMomentumModifierParameters.getModifyICPPlanByAngularMomentumRate());
         cmpOffsetAlphaFilter.set(angularMomentumModifierParameters.getCMPOffsetAlphaFilter());
         angularMomentumRateForwardGain.set(angularMomentumModifierParameters.getAngularMomentumRateForwardGain());
         angularMomentumRateLateralGain.set(angularMomentumModifierParameters.getAngularMomentumRateLateralGain());
      }
   }

   private final FramePoint3D desiredCMPPosition = new FramePoint3D();
   private final FrameVector3D desiredCMPVelocity = new FrameVector3D();

   private final FramePoint3D desiredICPPosition = new FramePoint3D();
   private final FrameVector3D desiredICPVelocity = new FrameVector3D();
   private final FrameVector3D desiredICPAcceleration = new FrameVector3D();

   // TODO have this guy account for the desired difference between the CMP and CoP.
   @Override
   public void modifyDesiredICPForAngularMomentum(FramePoint3D copEstimate, RobotSide supportSide)
   {
      super.getDesiredCentroidalMomentumPivotPosition(desiredCMPPosition);
      super.getDesiredCentroidalMomentumPivotVelocity(desiredCMPVelocity);

      super.getDesiredCapturePointPosition(desiredICPPosition);
      super.getDesiredCapturePointVelocity(desiredICPVelocity);
      super.getDesiredCapturePointAcceleration(desiredICPAcceleration);

      if (!modifyICPPlanByAngularMomentum.getBooleanValue() || copEstimate.containsNaN())
      {
         modifiedCMPPosition.set(desiredCMPPosition);
         modifiedCMPVelocity.set(desiredCMPVelocity);

         modifiedICPVelocity.set(desiredICPVelocity);
         modifiedICPAcceleration.set(desiredICPAcceleration);

         modifiedTimeInCurrentState.set(super.getTimeInCurrentState());
         modifiedTimeInCurrentStateRemaining.set(super.getTimeInCurrentStateRemaining());
      }
      else
      {
         cmpOffsetFromCoP.set(desiredCMPPosition);
         cmpOffsetFromCoP.sub(copEstimate);

         RobotSide transferToSide = super.getTransferToSide();

         if (supportSide == null && transferToSide == null)
         {
            cmpOffsetFromCoP.scale(angularMomentumRateForwardGain.getDoubleValue());
         }
         else
         {
            ReferenceFrame soleFrame;
            if (supportSide != null)
               soleFrame = soleZUpFrames.get(supportSide);
            else
               soleFrame = soleZUpFrames.get(transferToSide);

            cmpOffsetFromCoP.changeFrame(soleFrame);
            cmpOffsetFromCoP.setX(angularMomentumRateForwardGain.getDoubleValue() * cmpOffsetFromCoP.getX());
            cmpOffsetFromCoP.setY(angularMomentumRateLateralGain.getDoubleValue() * cmpOffsetFromCoP.getY());
            cmpOffsetFromCoP.changeFrame(worldFrame);
         }

         cmpOffset.set(cmpOffsetFromCoP);
         filteredCMPOffset.update();

         modifiedCMPPosition.set(desiredCMPPosition);
         modifiedCMPPosition.add(filteredCMPOffset);

         estimateCurrentTimeWithModifiedCMP(modifiedCMPPosition);

         double omega0 = super.getOmega0();
         modifiedICPVelocity.set(desiredICPPosition);
         modifiedICPVelocity.sub(modifiedCMPPosition);
         modifiedICPVelocity.scale(omega0);

         CapturePointTools.computeDesiredCapturePointAcceleration(omega0, modifiedICPVelocity, modifiedICPAcceleration);
         CapturePointTools.computeDesiredCentroidalMomentumPivotVelocity(modifiedICPVelocity, modifiedICPAcceleration, omega0, modifiedCMPVelocity);
      }
   }

   private void estimateCurrentTimeWithModifiedCMP(FramePoint3DReadOnly desiredCoPFromAngularMomentum)
   {
      double copCMPDistance = desiredCMPPosition.distanceXY(desiredCoPFromAngularMomentum);
      double distanceFromCMP = desiredICPPosition.distanceXY(modifiedCMPPosition);

      double modifiedTimeInState = 1.0 / super.getOmega0() * Math.log(distanceFromCMP / copCMPDistance);
      modifiedTimeInCurrentState.set(modifiedTimeInState);
      modifiedTimeInCurrentStateRemaining.set(getCurrentStateDuration() - modifiedTimeInCurrentState.getDoubleValue());
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCapturePointVelocity(FrameVector3D desiredCapturePointVelocityToPack)
   {
      desiredCapturePointVelocityToPack.setIncludingFrame(modifiedICPVelocity);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCapturePointVelocity(FrameVector2D desiredCapturePointVelocityToPack)
   {
      desiredCapturePointVelocityToPack.setIncludingFrame(modifiedICPVelocity);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCapturePointVelocity(YoFrameVector3D desiredCapturePointVelocityToPack)
   {
      desiredCapturePointVelocityToPack.set(modifiedICPVelocity);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCentroidalMomentumPivotPosition(FramePoint3D desiredCentroidalMomentumPivotPositionToPack)
   {
      desiredCentroidalMomentumPivotPositionToPack.setIncludingFrame(modifiedCMPPosition);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCentroidalMomentumPivotPosition(FramePoint2D desiredCentroidalMomentumPivotPositionToPack)
   {
      desiredCentroidalMomentumPivotPositionToPack.setIncludingFrame(modifiedCMPPosition);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCentroidalMomentumPivotVelocity(FrameVector3D desiredCentroidalMomentumPivotVelocityToPack)
   {
      desiredCentroidalMomentumPivotVelocityToPack.setIncludingFrame(modifiedCMPVelocity);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCentroidalMomentumPivotVelocity(FrameVector2D desiredCentroidalMomentumPivotVelocityToPack)
   {
      desiredCentroidalMomentumPivotVelocityToPack.setIncludingFrame(modifiedCMPVelocity);
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
