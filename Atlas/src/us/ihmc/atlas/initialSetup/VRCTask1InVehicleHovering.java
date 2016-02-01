package us.ihmc.atlas.initialSetup;

import static us.ihmc.atlas.ros.AtlasOrderedJointMap.back_bkx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.back_bky;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.back_bkz;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.jointNames;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_elx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_ely;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_shx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_shz;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_wrx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_wry;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_leg_akx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_leg_aky;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_leg_hpx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_leg_hpy;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_leg_hpz;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_leg_kny;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_arm_elx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_arm_ely;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_arm_shx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_arm_shz;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_arm_wrx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_arm_wry;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_leg_akx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_leg_aky;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_leg_hpx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_leg_hpy;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_leg_hpz;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_leg_kny;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class VRCTask1InVehicleHovering implements DRCRobotInitialSetup<SDFHumanoidRobot>
{

   private double groundZ;

   public VRCTask1InVehicleHovering(double groundZ)
   {
      this.groundZ = groundZ;
   }

   public void initializeRobot(SDFHumanoidRobot robot, DRCRobotJointMap jointMap)
   {
      // Avoid singularities at startup
      robot.getOneDegreeOfFreedomJoint(jointNames[l_arm_shz]).setQ(-1.0518);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_arm_shx]).setQ(-0.4177);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_arm_ely]).setQ(0.8856);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_arm_elx]).setQ(1.616);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_arm_wry]).setQ(-0.4238);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_arm_wrx]).setQ(0.9266);
      
      robot.getOneDegreeOfFreedomJoint(jointNames[r_arm_shz]).setQ(-0.297);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_arm_shx]).setQ(1.1054);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_arm_ely]).setQ(1.5975);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_arm_elx]).setQ(-1.1719);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_arm_wry]).setQ(-0.5188);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_arm_wrx]).setQ(-0.8051);
      
      robot.getOneDegreeOfFreedomJoint(jointNames[l_leg_hpz]).setQ(0.0405);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_leg_hpx]).setQ(0.0214);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_leg_hpy]).setQ(-1.1232);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_leg_kny]).setQ(1.2086);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_leg_aky]).setQ(0.0724);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_leg_akx]).setQ(-0.0005);

      robot.getOneDegreeOfFreedomJoint(jointNames[r_leg_hpz]).setQ(-0.1527);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_leg_hpx]).setQ(-0.1526);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_leg_hpy]).setQ(-1.2337);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_leg_kny]).setQ(1.3334);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_leg_aky]).setQ(0.048);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_leg_akx]).setQ(0.1072);

      robot.getOneDegreeOfFreedomJoint(jointNames[back_bkz]).setQ(-0.1519);
      robot.getOneDegreeOfFreedomJoint(jointNames[back_bky]).setQ(0.164);
      robot.getOneDegreeOfFreedomJoint(jointNames[back_bkx]).setQ(-0.0049);

      robot.getOneDegreeOfFreedomJoint("left_f0_j0").setQ(0.1098);
      robot.getOneDegreeOfFreedomJoint("left_f1_j0").setQ(0.1060);
      robot.getOneDegreeOfFreedomJoint("left_f2_j0").setQ(0.1040);
      robot.getOneDegreeOfFreedomJoint("left_f3_j0").setQ(-0.0376);

      robot.getOneDegreeOfFreedomJoint("left_f0_j1").setQ(1.1659);
      robot.getOneDegreeOfFreedomJoint("left_f1_j1").setQ(1.1627);
      robot.getOneDegreeOfFreedomJoint("left_f2_j1").setQ(1.1598);
      robot.getOneDegreeOfFreedomJoint("left_f3_j1").setQ(1.0352);
      
      robot.getOneDegreeOfFreedomJoint("left_f0_j2").setQ(1.0046);
      robot.getOneDegreeOfFreedomJoint("left_f1_j2").setQ(1.0054);
      robot.getOneDegreeOfFreedomJoint("left_f2_j2").setQ(1.0060);
      robot.getOneDegreeOfFreedomJoint("left_f3_j2").setQ(0.0231);

      robot.getOneDegreeOfFreedomJoint("right_f3_j1").setQ(-1.57);

      robot.setPositionInWorld(new Vector3d(-0.096, 0.3726, 1.1064));
      robot.setOrientation(new Quat4d(-0.0076, -0.0814, 0.0747, 0.9938));
   }
   
   public void getOffset(Vector3d offsetToPack)
   {
   }

   public void setOffset(Vector3d offset)
   {
   }

   @Override
   public void setInitialYaw(double yaw)
   {
   }

   @Override
   public void setInitialGroundHeight(double groundHeight)
   {
      groundZ = groundHeight;
   }

   @Override
   public double getInitialYaw()
   {
      return 0;
   }

   @Override
   public double getInitialGroundHeight()
   {
      return groundZ;
   }
}