package us.ihmc.commonWalkingControlModules.configurations;

import javax.media.j3d.Transform3D;

import us.ihmc.robotSide.SideDependentList;

public interface WalkingControllerParameters extends HeadOrientationControllerParameters
{
   public abstract SideDependentList<Transform3D> getDesiredHandPosesWithRespectToChestFrame();

   public abstract boolean doStrictPelvisControl();
      
   public abstract String[] getChestOrientationControlJointNames();

   public abstract boolean checkOrbitalEnergyCondition();

   public abstract double getGroundReactionWrenchBreakFrequencyHertz();

   public abstract boolean resetDesiredICPToCurrentAtStartOfSwing();
   
   public abstract double getFootForwardOffset();
   
   public abstract double getFootBackwardOffset();
   
   public abstract double getAnkleHeight();
   
   public abstract double nominalHeightAboveAnkle();
   
   public abstract boolean landOnHeels();
   
   public abstract SideDependentList<Transform3D> getHandControlFramesWithRespectToFrameAfterWrist();

   boolean finishSwingWhenTrajectoryDone();
}