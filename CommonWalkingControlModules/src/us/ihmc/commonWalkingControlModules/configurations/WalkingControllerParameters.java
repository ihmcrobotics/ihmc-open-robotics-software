package us.ihmc.commonWalkingControlModules.configurations;

import javax.media.j3d.Transform3D;

import us.ihmc.robotSide.SideDependentList;

public interface WalkingControllerParameters
{
   public abstract SideDependentList<Transform3D> getDesiredHandPosesWithRespectToChestFrame();

   public abstract double getDesiredCoMHeight();

   public abstract boolean doStrictPelvisControl();
   
   public abstract String[] neckJointsToUseForHeadOrientationControl();
   
   public abstract boolean checkOrbitalCondition();
   
   public abstract void setCheckOrbitalCondition(boolean checkOrbitalCondition);
   
   public abstract double nominalHeightAboveGround();
   
   public abstract void setNominalHeightAboveGround(double nominalHeightAboveGround);
   
   public abstract double initialHeightAboveGround();
   
   public abstract void setInitialHeightAboveGround(double initialHeightAboveGround);
}