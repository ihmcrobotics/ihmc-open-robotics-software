package us.ihmc.darpaRoboticsChallenge;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotParameters;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.RotationFunctions;

public class DRCRobotWalkingControllerParameters implements WalkingControllerParameters
{
   public SideDependentList<Transform3D> getDesiredHandPosesWithRespectToChestFrame()
   {
      SideDependentList<Transform3D> handPosesWithRespectToChestFrame = new SideDependentList<Transform3D>();

      for (RobotSide robotSide : RobotSide.values())
      {
         Transform3D transform = new Transform3D();

         double x = 0.15;
         double y = robotSide.negateIfRightSide(0.35);
         double z = -0.35;
         transform.setTranslation(new Vector3d(x, y, z));

         Matrix3d rotation = new Matrix3d();
         double yaw = robotSide.negateIfRightSide(-1.7);
         double pitch = 0.1;
         double roll = robotSide.negateIfRightSide(-0.8);
         RotationFunctions.setYawPitchRoll(rotation, yaw, pitch, roll);
         transform.setRotation(rotation);

         handPosesWithRespectToChestFrame.put(robotSide, transform);
      }

      return handPosesWithRespectToChestFrame;
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
//    return new String[] {"back_mby"};
      return new String[]
      {
      };
   }

   public boolean checkOrbitalCondition()
   {
      return false;
   }

   double nominalHeightAboveGround = 0.78;

   public double nominalHeightAboveAnkle()
   {
      return nominalHeightAboveGround;
   }

   public void setNominalHeightAboveAnkle(double nominalHeightAboveAnkle)
   {
      this.nominalHeightAboveGround = nominalHeightAboveAnkle;
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
      return DRCRobotParameters.DRC_ROBOT_NECK_PITCH_UPPER_LIMIT;
   }

   public double getLowerNeckPitchLimit()
   {
      return DRCRobotParameters.DRC_ROBOT_NECK_PITCH_LOWER_LIMIT;
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
