package us.ihmc.commonWalkingControlModules.configurations;

import javax.media.j3d.Transform3D;

import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

import java.util.Map;

public interface WalkingControllerParameters extends HeadOrientationControllerParameters, ManipulationControllerParameters
{
   public abstract SideDependentList<Transform3D> getDesiredHandPosesWithRespectToChestFrame();

   public abstract String[] getChestOrientationControlJointNames();

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

   public abstract boolean doToeOffIfPossible();

   public abstract double getFinalToeOffPitchAngularVelocity();
}