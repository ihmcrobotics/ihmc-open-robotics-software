package us.ihmc.commonWalkingControlModules.configurations;

import javax.media.j3d.Transform3D;

import us.ihmc.robotSide.SideDependentList;

public interface WalkingControllerParameters
{

   public abstract SideDependentList<Transform3D> getDesiredHandPosesWithRespectToChestFrame();

   public abstract double getDesiredCoMHeight();

}