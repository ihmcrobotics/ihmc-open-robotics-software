package us.ihmc.darpaRoboticsChallenge.initialSetup;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.projectM.R2Sim02.initialSetup.RobotInitialSetup;

import javax.media.j3d.Transform3D;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

public class VRCTask1InVehicleHovering implements RobotInitialSetup<SDFRobot>
{

   private final double groundZ;

   public VRCTask1InVehicleHovering(double groundZ)
   {
      this.groundZ = groundZ;
   }

   public void initializeRobot(SDFRobot robot)
   {
      // Avoid singularities at startup
      robot.getOneDoFJoint("l_arm_usy").setQ(-1.3363);
      robot.getOneDoFJoint("l_arm_shx").setQ(-0.751);
      robot.getOneDoFJoint("l_arm_ely").setQ(0.6108);
      robot.getOneDoFJoint("l_arm_elx").setQ(2.0035);
      robot.getOneDoFJoint("l_arm_uwy").setQ(-0.1013);
      robot.getOneDoFJoint("l_arm_mwx").setQ(0.5174);
      
      robot.getOneDoFJoint("r_arm_usy").setQ(-0.2486);
      robot.getOneDoFJoint("r_arm_shx").setQ(1.1101);
      robot.getOneDoFJoint("r_arm_ely").setQ(1.6088);
      robot.getOneDoFJoint("r_arm_elx").setQ(-1.1552);
      robot.getOneDoFJoint("r_arm_uwy").setQ(-0.5442);
      robot.getOneDoFJoint("r_arm_mwx").setQ(-0.8353);
      
      robot.getOneDoFJoint("l_leg_uhz").setQ(0.0111);
      robot.getOneDoFJoint("l_leg_mhx").setQ(-0.0207);
      robot.getOneDoFJoint("l_leg_lhy").setQ(-1.1163);
      robot.getOneDoFJoint("l_leg_kny").setQ(1.2062);
      robot.getOneDoFJoint("l_leg_uay").setQ(0.0659);
      robot.getOneDoFJoint("l_leg_lax").setQ(0.0126);

      robot.getOneDoFJoint("r_leg_uhz").setQ(-0.0804);
      robot.getOneDoFJoint("r_leg_mhx").setQ(-0.1971);
      robot.getOneDoFJoint("r_leg_lhy").setQ(-1.2435);
      robot.getOneDoFJoint("r_leg_kny").setQ(1.3467);
      robot.getOneDoFJoint("r_leg_uay").setQ(0.0605);
      robot.getOneDoFJoint("r_leg_lax").setQ(0.1517);

      robot.getOneDoFJoint("back_lbz").setQ(-0.0739);
      robot.getOneDoFJoint("back_mby").setQ(0.1544);
      robot.getOneDoFJoint("back_ubx").setQ(-0.0588);

      robot.getOneDoFJoint("left_f0_j0").setQ(0.1233);
      robot.getOneDoFJoint("left_f1_j0").setQ(0.1018);
      robot.getOneDoFJoint("left_f2_j0").setQ(0.0894);
      robot.getOneDoFJoint("left_f3_j0").setQ(0.0381);

      robot.getOneDoFJoint("left_f0_j1").setQ(1.1508);
      robot.getOneDoFJoint("left_f1_j1").setQ(1.1687);
      robot.getOneDoFJoint("left_f2_j1").setQ(1.1756);
      robot.getOneDoFJoint("left_f3_j1").setQ(1.0582);
      
      robot.getOneDoFJoint("left_f0_j2").setQ(1.0354);
      robot.getOneDoFJoint("left_f1_j2").setQ(0.9934);
      robot.getOneDoFJoint("left_f2_j2").setQ(0.9725);
      robot.getOneDoFJoint("left_f3_j2").setQ(-0.0363);

      robot.getOneDoFJoint("right_f3_j1").setQ(-1.57);

      robot.setPositionInWorld(new Vector3d(-0.0955, 0.3763, 1.1063));
      robot.setOrientation(new Quat4d(0.0091, -0.0797, 0.053, 0.9954));
   }
}