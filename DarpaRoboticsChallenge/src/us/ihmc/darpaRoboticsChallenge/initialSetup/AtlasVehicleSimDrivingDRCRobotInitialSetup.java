package us.ihmc.darpaRoboticsChallenge.initialSetup;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.projectM.R2Sim02.initialSetup.RobotInitialSetup;
import static us.ihmc.darpaRoboticsChallenge.ros.ROSAtlasJointMap.*;

public class AtlasVehicleSimDrivingDRCRobotInitialSetup implements RobotInitialSetup<SDFRobot>
{

   private final double groundZ;
   private final Transform3D rootToWorld = new Transform3D();
   private final Vector3d offset = new Vector3d();

   public AtlasVehicleSimDrivingDRCRobotInitialSetup(double groundZ)
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
      robot.getOneDoFJoint(jointNames[l_arm_shx]).setQ(-1.57);

      robot.getOneDoFJoint(jointNames[r_arm_shx]).setQ(1.57);

      robot.getOneDoFJoint(jointNames[l_arm_ely]).setQ(1.57);
      robot.getOneDoFJoint(jointNames[l_arm_elx]).setQ(1.57);

      robot.getOneDoFJoint(jointNames[r_arm_ely]).setQ(1.57);
      robot.getOneDoFJoint(jointNames[r_arm_elx]).setQ(-1.57);

      robot.getOneDoFJoint(jointNames[l_arm_wry]).setQ(0);
      robot.getOneDoFJoint(jointNames[r_arm_wry]).setQ(0);

      robot.getOneDoFJoint(jointNames[l_leg_hpy]).setQ(hipBend + thighPitch);
      robot.getOneDoFJoint(jointNames[r_leg_hpy]).setQ(hipBend + thighPitch);

      robot.getOneDoFJoint(jointNames[l_leg_kny]).setQ(kneeBend);
      robot.getOneDoFJoint(jointNames[r_leg_kny]).setQ(kneeBend);

      robot.getOneDoFJoint(jointNames[l_leg_aky]).setQ(thighPitch - .3);  //0.087 + thighPitch);
      robot.getOneDoFJoint(jointNames[r_leg_aky]).setQ(thighPitch - .3); //0.087 + thighPitch);

      offset.setX(0.7);
      offset.setY(-0.07);
      offset.setZ(groundZ + 1.04);
      robot.setPositionInWorld(offset);
      robot.setOrientation(Math.PI / 2.0, forwardLean, 0.0);
   }
}