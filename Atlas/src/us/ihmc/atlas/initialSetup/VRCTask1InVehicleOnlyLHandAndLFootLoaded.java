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

import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class VRCTask1InVehicleOnlyLHandAndLFootLoaded implements DRCRobotInitialSetup<HumanoidFloatingRootJointRobot>
{

   private double groundZ;
   private final RigidBodyTransform rootToWorld = new RigidBodyTransform();
   private final Vector3D offset = new Vector3D();

   public VRCTask1InVehicleOnlyLHandAndLFootLoaded(double groundZ)
   {
      this.groundZ = groundZ;
   }

   public void initializeRobot(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      // Avoid singularities at startup
      robot.getOneDegreeOfFreedomJoint(jointNames[l_arm_shz]).setQ(-0.8528);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_arm_shx]).setQ(0.1144);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_arm_ely]).setQ(0.9796);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_arm_elx]).setQ(1.6769);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_arm_wry]).setQ(-1.13);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_arm_wrx]).setQ(0.6748);
      
      robot.getOneDegreeOfFreedomJoint(jointNames[r_arm_shz]).setQ(-0.1573);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_arm_shx]).setQ(0.9835);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_arm_ely]).setQ(1.5037);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_arm_elx]).setQ(-1.3852);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_arm_wry]).setQ(-0.4969);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_arm_wrx]).setQ(-0.2671);
      
      robot.getOneDegreeOfFreedomJoint(jointNames[l_leg_hpz]).setQ(-0.0308);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_leg_hpx]).setQ(0.1414);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_leg_hpy]).setQ(-1.5865);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_leg_kny]).setQ(1.7287);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_leg_aky]).setQ(-0.0256);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_leg_akx]).setQ(-0.0064);

      robot.getOneDegreeOfFreedomJoint(jointNames[r_leg_hpz]).setQ(-0.1052);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_leg_hpx]).setQ(-0.2278);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_leg_hpy]).setQ(-1.5537);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_leg_kny]).setQ(1.6453);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_leg_aky]).setQ(-0.0256);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_leg_akx]).setQ(0.3884);

      robot.getOneDegreeOfFreedomJoint(jointNames[back_bkz]).setQ(-0.0312);
      robot.getOneDegreeOfFreedomJoint(jointNames[back_bky]).setQ(-0.2424);
      robot.getOneDegreeOfFreedomJoint(jointNames[back_bkx]).setQ(0.1817);

      robot.getOneDegreeOfFreedomJoint("left_f0_j0").setQ(0.1157);
      robot.getOneDegreeOfFreedomJoint("left_f1_j0").setQ(0.1112);
      robot.getOneDegreeOfFreedomJoint("left_f2_j0").setQ(0.1091);
      robot.getOneDegreeOfFreedomJoint("left_f3_j0").setQ(0.0043);

      robot.getOneDegreeOfFreedomJoint("left_f0_j1").setQ(1.1612);
      robot.getOneDegreeOfFreedomJoint("left_f1_j1").setQ(1.158);
      robot.getOneDegreeOfFreedomJoint("left_f2_j1").setQ(1.155);
      robot.getOneDegreeOfFreedomJoint("left_f3_j1").setQ(1.0436);
      
      robot.getOneDegreeOfFreedomJoint("left_f0_j2").setQ(1.0156);
      robot.getOneDegreeOfFreedomJoint("left_f1_j2").setQ(1.0162);
      robot.getOneDegreeOfFreedomJoint("left_f2_j2").setQ(1.017);
      robot.getOneDegreeOfFreedomJoint("left_f3_j2").setQ(0.0011);

      robot.getOneDegreeOfFreedomJoint("right_f3_j1").setQ(-1.57);

      robot.setPositionInWorld(new Vector3D(-0.079, 0.3955, 0.9872));
      robot.setOrientation(new Quaternion(-0.0672, -0.0334, 0.0296, 0.9967));
   }
   
   public void getOffset(Vector3D offsetToPack)
   {
      offsetToPack.set(offset);
   }

   public void setOffset(Vector3D offset)
   {
      this.offset.set(offset);
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