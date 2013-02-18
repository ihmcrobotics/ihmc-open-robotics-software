package us.ihmc.darpaRoboticsChallenge;

import javax.media.j3d.Transform3D;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotParameters;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;

public class DRCRobotWalkingControllerParameters implements WalkingControllerParameters
{
   public SideDependentList<Transform3D> getDesiredHandPosesWithRespectToChestFrame()
   {
      SideDependentList<Transform3D> handPoseWithRespectToChestFrame = new SideDependentList<Transform3D>();
      for(RobotSide robotSide : RobotSide.values)
      {
         double[] mat = { 0, robotSide.negateIfLeftSide(-0.7), 0.71, 0.25,
               robotSide.negateIfLeftSide(1), 0, 0, robotSide.negateIfLeftSide(-0.30),
               0, robotSide.negateIfLeftSide(0.71), 0.7, -0.44,
               0.0, 0.0, 0.0, 1.0 };
         
         final Transform3D transform3d = new Transform3D(mat);
         
         handPoseWithRespectToChestFrame.put(robotSide, transform3d);
      }
      
      return handPoseWithRespectToChestFrame;
   }
   
   public double getDesiredCoMHeight()
   {
      return 0.88;
   }

   public boolean doStrictPelvisControl()
   {
      return true;
   }

   public String[] getHeadOrientationControlJointNames()
   {
      return new String[] {"back_lbz", "back_ubx", "neck_ay"};
   }
   
   public String[] getChestOrientationControlJointNames()
   {
//      return new String[] {"back_mby"};
      return new String[] {};
   }
   
   public boolean checkOrbitalCondition() 
   {
      return false;
   }

   public double nominalHeightAboveGround()
   {
      return 0.9;
   }

   private double initialHeightAboveGround = 0.86;
   
   public double initialHeightAboveGround()
   {
      return initialHeightAboveGround;
   }
   
   public void setInitialHeightAboveGround(double initialHeightAboveGround)
   {
      this.initialHeightAboveGround = initialHeightAboveGround;
   }

   public double getGroundReactionWrenchBreakFrequencyHertz()
   {
      return 7.0;
   }

   public boolean resetDesiredICPToCurrentAtStartOfSwing()
   {
      return false;
   }

   public double getUpperNeckPitchLimit()
   {
      return DRCRobotParameters.DRC_ROBOT_UPPER_NECK_PITCH_LIMIT;
   }

   public double getLowerNeckPitchLimit()
   {
      return DRCRobotParameters.DRC_ROBOT_LOWER_NECK_PITCH_LIMIT;
   }

   public double getHeadYawLimit()
   {
      return DRCRobotParameters.DRC_ROBOT_HEAD_YAW_LIMIT;
   }

   public double getHeadRollLimit()
   {
      return DRCRobotParameters.DRC_ROBOT_HEAD_ROLL_LIMIT;
   }

   public String getJointNameForExtendedPitchRange()
   {
      return "back_mby";
   }
}
