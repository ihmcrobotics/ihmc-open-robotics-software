package us.ihmc.commonWalkingControlModules.configurations;

import javax.media.j3d.Transform3D;

import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

import java.util.Map;

public interface WalkingControllerParameters extends HeadOrientationControllerParameters
{
   public abstract SideDependentList<Transform3D> getDesiredHandPosesWithRespectToChestFrame();

   public abstract String[] getChestOrientationControlJointNames();

   public abstract boolean checkOrbitalEnergyCondition();

   public abstract double getGroundReactionWrenchBreakFrequencyHertz();

   public abstract boolean resetDesiredICPToCurrentAtStartOfSwing();
   
   public abstract double getFootForwardOffset();
   
   public abstract double getFootBackwardOffset();
   
   public abstract double getAnkleHeight();
   
   public abstract double nominalHeightAboveAnkle();
   
   public abstract SideDependentList<Transform3D> getHandControlFramesWithRespectToFrameAfterWrist();

   public abstract boolean finishSwingWhenTrajectoryDone();

   public abstract Map<OneDoFJoint, Double> getDefaultArmJointPositions(FullRobotModel fullRobotModel, RobotSide robotSide);

   public abstract boolean doToeOffIfPossible();

   public abstract double getFinalToeOffPitchAngularVelocity();
}