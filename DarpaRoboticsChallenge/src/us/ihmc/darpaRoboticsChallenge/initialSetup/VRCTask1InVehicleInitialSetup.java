package us.ihmc.darpaRoboticsChallenge.initialSetup;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.projectM.R2Sim02.initialSetup.RobotInitialSetup;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

public class VRCTask1InVehicleInitialSetup implements RobotInitialSetup<SDFRobot>
{

   private final double groundZ;
   private final Transform3D rootToWorld = new Transform3D();
   private final Vector3d offset = new Vector3d();

   public VRCTask1InVehicleInitialSetup(double groundZ)
   {
      this.groundZ = groundZ;
   }

   public void initializeRobot(SDFRobot robot)
   {
      double thighPitch = 0.0;
      double forwardLean = 0.0;
      double hipBend = -Math.PI / 2.0 - forwardLean;
      double kneeBend = 1.22; //Math.PI / 2.0;

      // Avoid singularities at startup
      robot.getOneDoFJoint("l_arm_shx").setQ(-1.57);

      robot.getOneDoFJoint("r_arm_shx").setQ(1.57);

      robot.getOneDoFJoint("l_arm_ely").setQ(1.57);
      robot.getOneDoFJoint("l_arm_elx").setQ(1.57);

      robot.getOneDoFJoint("r_arm_ely").setQ(1.57);
      robot.getOneDoFJoint("r_arm_elx").setQ(-1.57);

      robot.getOneDoFJoint("l_arm_uwy").setQ(0);
      robot.getOneDoFJoint("r_arm_uwy").setQ(0);

      robot.getOneDoFJoint("l_leg_lhy").setQ(hipBend + thighPitch);
      robot.getOneDoFJoint("r_leg_lhy").setQ(hipBend + thighPitch);

      robot.getOneDoFJoint("l_leg_kny").setQ(kneeBend);
      robot.getOneDoFJoint("r_leg_kny").setQ(kneeBend);

      robot.getOneDoFJoint("l_leg_uay").setQ(thighPitch - .3);  //0.087 + thighPitch);
      robot.getOneDoFJoint("r_leg_uay").setQ(thighPitch - .3); //0.087 + thighPitch);

      offset.setX(-0.06);
      offset.setY(0.28);
      offset.setZ(groundZ + 1.08); // 1.04);
      robot.setPositionInWorld(offset);
      robot.setOrientation(0.0, forwardLean, 0.0);
   }
}