package us.ihmc.darpaRoboticsChallenge.initialSetup;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.projectM.R2Sim02.initialSetup.RobotInitialSetup;
import static us.ihmc.darpaRoboticsChallenge.ros.ROSAtlasJointMap.*;

import javax.media.j3d.Transform3D;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

public class VRCTask1InVehicleOnlyLHandAndLFootLoaded implements RobotInitialSetup<SDFRobot>
{

   private final double groundZ;
   private final Transform3D rootToWorld = new Transform3D();
   private final Vector3d offset = new Vector3d();

   public VRCTask1InVehicleOnlyLHandAndLFootLoaded(double groundZ)
   {
      this.groundZ = groundZ;
   }

   public void initializeRobot(SDFRobot robot)
   {
      // Avoid singularities at startup
      robot.getOneDoFJoint(jointNames[l_arm_usy]).setQ(-0.8528);
      robot.getOneDoFJoint(jointNames[l_arm_shx]).setQ(0.1144);
      robot.getOneDoFJoint(jointNames[l_arm_ely]).setQ(0.9796);
      robot.getOneDoFJoint(jointNames[l_arm_elx]).setQ(1.6769);
      robot.getOneDoFJoint(jointNames[l_arm_uwy]).setQ(-1.13);
      robot.getOneDoFJoint(jointNames[l_arm_mwx]).setQ(0.6748);
      
      robot.getOneDoFJoint(jointNames[r_arm_usy]).setQ(-0.1573);
      robot.getOneDoFJoint(jointNames[r_arm_shx]).setQ(0.9835);
      robot.getOneDoFJoint(jointNames[r_arm_ely]).setQ(1.5037);
      robot.getOneDoFJoint(jointNames[r_arm_elx]).setQ(-1.3852);
      robot.getOneDoFJoint(jointNames[r_arm_uwy]).setQ(-0.4969);
      robot.getOneDoFJoint(jointNames[r_arm_mwx]).setQ(-0.2671);
      
      robot.getOneDoFJoint(jointNames[l_leg_uhz]).setQ(-0.0308);
      robot.getOneDoFJoint(jointNames[l_leg_mhx]).setQ(0.1414);
      robot.getOneDoFJoint(jointNames[l_leg_lhy]).setQ(-1.5865);
      robot.getOneDoFJoint(jointNames[l_leg_kny]).setQ(1.7287);
      robot.getOneDoFJoint(jointNames[l_leg_uay]).setQ(-0.0256);
      robot.getOneDoFJoint(jointNames[l_leg_lax]).setQ(-0.0064);

      robot.getOneDoFJoint(jointNames[r_leg_uhz]).setQ(-0.1052);
      robot.getOneDoFJoint(jointNames[r_leg_mhx]).setQ(-0.2278);
      robot.getOneDoFJoint(jointNames[r_leg_lhy]).setQ(-1.5537);
      robot.getOneDoFJoint(jointNames[r_leg_kny]).setQ(1.6453);
      robot.getOneDoFJoint(jointNames[r_leg_uay]).setQ(-0.0256);
      robot.getOneDoFJoint(jointNames[r_leg_lax]).setQ(0.3884);

      robot.getOneDoFJoint(jointNames[back_lbz]).setQ(-0.0312);
      robot.getOneDoFJoint(jointNames[back_mby]).setQ(-0.2424);
      robot.getOneDoFJoint(jointNames[back_ubx]).setQ(0.1817);

      robot.getOneDoFJoint("left_f0_j0").setQ(0.1157);
      robot.getOneDoFJoint("left_f1_j0").setQ(0.1112);
      robot.getOneDoFJoint("left_f2_j0").setQ(0.1091);
      robot.getOneDoFJoint("left_f3_j0").setQ(0.0043);

      robot.getOneDoFJoint("left_f0_j1").setQ(1.1612);
      robot.getOneDoFJoint("left_f1_j1").setQ(1.158);
      robot.getOneDoFJoint("left_f2_j1").setQ(1.155);
      robot.getOneDoFJoint("left_f3_j1").setQ(1.0436);
      
      robot.getOneDoFJoint("left_f0_j2").setQ(1.0156);
      robot.getOneDoFJoint("left_f1_j2").setQ(1.0162);
      robot.getOneDoFJoint("left_f2_j2").setQ(1.017);
      robot.getOneDoFJoint("left_f3_j2").setQ(0.0011);

      robot.getOneDoFJoint("right_f3_j1").setQ(-1.57);

      robot.setPositionInWorld(new Vector3d(-0.079, 0.3955, 0.9872));
      robot.setOrientation(new Quat4d(-0.0672, -0.0334, 0.0296, 0.9967));
   }
}