package us.ihmc.commonWalkingControlModules.configurations;

import javax.media.j3d.Transform3D;

import us.ihmc.robotSide.SideDependentList;

public interface WalkingControllerParameters extends HeadOrientationControllerParameters, ManipulationControllerParameters
{
   public abstract SideDependentList<Transform3D> getDesiredHandPosesWithRespectToChestFrame();

   public abstract String[] getAllowableChestOrientationControlJointNames();
   public abstract String[] getDefaultChestOrientationControlJointNames();

   public abstract boolean checkOrbitalEnergyCondition();

   public abstract double getGroundReactionWrenchBreakFrequencyHertz();

   public abstract boolean resetDesiredICPToCurrentAtStartOfSwing();
   
   public abstract double getFootForwardOffset();
   
   public abstract double getFootBackwardOffset();
   
   public abstract double getAnkleHeight();
   
   public abstract double minimumHeightAboveAnkle();
   public abstract double nominalHeightAboveAnkle();
   public abstract double maximumHeightAboveAnkle();
   
   public abstract boolean finishSwingWhenTrajectoryDone();

   public abstract boolean stayOnToes();
   
   public abstract boolean doToeOffIfPossible();
   public abstract boolean doToeTouchdownIfPossible();
   public abstract boolean doHeelTouchdownIfPossible();

   public abstract double getFinalToeOffPitchAngularVelocity();

   public abstract double getInPlaceWidth();

   public abstract double getDesiredStepForward();

   public abstract double getMaxStepLength();

   public abstract double getMinStepWidth();

   public abstract double getMaxStepWidth();

   public abstract double getStepPitch();

   public abstract double getCaptureKpParallelToMotion();
   public abstract double getCaptureKpOrthogonalToMotion();
   public abstract double getCaptureFilterBreakFrequencyInHz();

   public abstract double getKpPelvisOrientation();
   public abstract double getZetaPelvisOrientation();

   public abstract double getKpCoMHeight();
   public abstract double getZetaCoMHeight();

   public abstract double getKpHeadOrientation();
   public abstract double getZetaHeadOrientation();

   public abstract double getKpUpperBody();
   public abstract double getZetaUpperBody();
   public abstract double getMaxAccelerationUpperBody();

}