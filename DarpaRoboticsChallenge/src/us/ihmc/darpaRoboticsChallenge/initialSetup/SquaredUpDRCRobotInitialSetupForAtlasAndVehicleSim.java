package us.ihmc.darpaRoboticsChallenge.initialSetup;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFRobot;

public class SquaredUpDRCRobotInitialSetupForAtlasAndVehicleSim extends SquaredUpDRCRobotInitialSetup
{

   private final double groundZ;
   private final Transform3D rootToWorld = new Transform3D();
   private final Vector3d offset = new Vector3d();

   public SquaredUpDRCRobotInitialSetupForAtlasAndVehicleSim(double groundZ)
   {
      this.groundZ = groundZ;
   }

   public void initializeRobot(SDFRobot robot)
   {
      double thighPitch = 0.0;
      double forwardLean = 0.0;
      double hipBend = -Math.PI / 2.0 - forwardLean;
      double kneeBend = Math.PI / 2.0;

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

      robot.getOneDoFJoint("l_leg_uay").setQ(0.087 + thighPitch);
      robot.getOneDoFJoint("r_leg_uay").setQ(0.087 + thighPitch);

      robot.update();
      robot.getRootJointToWorldTransform(rootToWorld);
      rootToWorld.get(offset);
      

      offset.setX(0.7);
      offset.setY(-0.1);
      offset.setZ(groundZ + 0.03);
      offset.add(robot.getPositionInWorld());
      robot.setPositionInWorld(offset);
      robot.setOrientation(Math.PI / 2.0, forwardLean, 0.0);
   }

}
