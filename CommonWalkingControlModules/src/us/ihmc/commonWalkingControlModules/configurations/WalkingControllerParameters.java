package us.ihmc.commonWalkingControlModules.configurations;

import javax.media.j3d.Transform3D;

import us.ihmc.robotSide.SideDependentList;

public interface WalkingControllerParameters extends HeadOrientationControllerParameters
{
   public abstract SideDependentList<Transform3D> getDesiredHandPosesWithRespectToChestFrame();

   public abstract String[] getDefaultChestOrientationControlJointNames();

   public abstract boolean checkOrbitalEnergyCondition();

   public abstract double getGroundReactionWrenchBreakFrequencyHertz();

   public abstract boolean resetDesiredICPToCurrentAtStartOfSwing();

   public abstract double getFootForwardOffset();

   public abstract double getFootBackwardOffset();

   public abstract double getFootWidth();

   public abstract double getToeWidth();

   public abstract double getFootLength();

   public abstract double getAnkleHeight();

   public abstract double getLegLength();

   public abstract double getMinLegLengthBeforeCollapsingSingleSupport();

   public abstract double minimumHeightAboveAnkle();

   public abstract double nominalHeightAboveAnkle();

   public abstract double maximumHeightAboveAnkle();

   public abstract boolean finishSwingWhenTrajectoryDone();

   public abstract boolean stayOnToes();

   public abstract boolean doToeOffIfPossible();

   public abstract double getMaximumToeOffAngle();

   public abstract double getFootSwitchCoPThresholdFraction();

   public abstract boolean doToeTouchdownIfPossible();

   public abstract double getToeTouchdownAngle();

   public abstract boolean doHeelTouchdownIfPossible();

   public abstract double getHeelTouchdownAngle();

   public abstract double getFinalToeOffPitchAngularVelocity();

   public abstract double getInPlaceWidth();

   public abstract double getDesiredStepForward();

   public abstract double getMaxStepLength();

   public abstract double getMinStepWidth();

   public abstract double getMaxStepWidth();

   public abstract double getStepPitch();

   public abstract double getCaptureKpParallelToMotion();

   public abstract double getCaptureKpOrthogonalToMotion();

   public abstract double getCaptureKi();

   public abstract double getCaptureKiBleedoff();

   public abstract double getCaptureFilterBreakFrequencyInHz();

   public abstract double getCMPRateLimit();

   public abstract double getCMPAccelerationLimit();

   public abstract double getDefaultDesiredPelvisPitch();

   public abstract double getKpPelvisOrientation();

   public abstract double getZetaPelvisOrientation();

   public abstract double getMaxAccelerationPelvisOrientation();

   public abstract double getMaxJerkPelvisOrientation();

   public abstract double getKpCoMHeight();

   public abstract double getZetaCoMHeight();

   public abstract double getKpHeadOrientation();

   public abstract double getZetaHeadOrientation();

   public abstract double getTrajectoryTimeHeadOrientation();

   public abstract double getKpUpperBody();

   public abstract double getZetaUpperBody();

   public abstract double getMaxAccelerationUpperBody();

   public abstract double getMaxJerkUpperBody();

   public abstract double getSwingKpXY();

   public abstract double getSwingKpZ();

   public abstract double getSwingHeightMaxForPushRecoveryTrajectory();

   public abstract double getSwingKpOrientation();

   public abstract double getSwingZetaXYZ();

   public abstract double getSwingZetaOrientation();

   public abstract double getHoldKpXY();

   public abstract double getHoldKpOrientation();

   public abstract double getHoldZeta();

   public abstract double getSwingMaxPositionAcceleration();

   public abstract double getSwingMaxPositionJerk();

   public abstract double getSwingMaxOrientationAcceleration();

   public abstract double getSwingMaxOrientationJerk();

   public abstract double getSupportSingularityEscapeMultiplier();

   public abstract double getSwingSingularityEscapeMultiplier();

   public abstract double getToeOffKpXY();

   public abstract double getToeOffKpOrientation();

   public abstract double getToeOffZeta();

   public abstract boolean doPrepareManipulationForLocomotion();

   public abstract boolean isRunningOnRealRobot();

   public abstract double getDefaultTransferTime();

   public abstract double getDefaultSwingTime();

   public abstract double getPelvisPitchUpperLimit();

   public abstract double getPelvisPitchLowerLimit();

   public abstract boolean isPelvisPitchReversed();

   public abstract double getFoot_start_toetaper_from_back();

   public abstract double getSideLengthOfBoundingBoxForFootstepHeight();

   public abstract double getDesiredTouchdownVelocity();

   public abstract double getContactThresholdForce();
}