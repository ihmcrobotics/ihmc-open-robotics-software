package us.ihmc.atlas.initialSetup;

import static us.ihmc.atlas.ros.AtlasOrderedJointMap.jointNames;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_elx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_ely;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_shx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_wry;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_leg_aky;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_leg_hpy;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_leg_kny;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_arm_elx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_arm_ely;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_arm_shx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_arm_wry;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_leg_aky;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_leg_hpy;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_leg_kny;

import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class AtlasVehicleSimDrivingDRCRobotInitialSetup implements DRCRobotInitialSetup<HumanoidFloatingRootJointRobot>
{

   private double groundZ;
   private final RigidBodyTransform rootToWorld = new RigidBodyTransform();
   private final Vector3D offset = new Vector3D();

   public AtlasVehicleSimDrivingDRCRobotInitialSetup(double groundZ)
   {
      this.groundZ = groundZ;
   }

   public void initializeRobot(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      double thighPitch = 0.0;
      double forwardLean = 0.0;
      double hipBend = -Math.PI / 2.0 - forwardLean;
      double kneeBend = 1.22; //Math.PI / 2.0;

      // Avoid singularities at startup
      robot.getOneDegreeOfFreedomJoint(jointNames[l_arm_shx]).setQ(-1.57);

      robot.getOneDegreeOfFreedomJoint(jointNames[r_arm_shx]).setQ(1.57);

      robot.getOneDegreeOfFreedomJoint(jointNames[l_arm_ely]).setQ(1.57);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_arm_elx]).setQ(1.57);

      robot.getOneDegreeOfFreedomJoint(jointNames[r_arm_ely]).setQ(1.57);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_arm_elx]).setQ(-1.57);

      robot.getOneDegreeOfFreedomJoint(jointNames[l_arm_wry]).setQ(0);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_arm_wry]).setQ(0);

      robot.getOneDegreeOfFreedomJoint(jointNames[l_leg_hpy]).setQ(hipBend + thighPitch);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_leg_hpy]).setQ(hipBend + thighPitch);

      robot.getOneDegreeOfFreedomJoint(jointNames[l_leg_kny]).setQ(kneeBend);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_leg_kny]).setQ(kneeBend);

      robot.getOneDegreeOfFreedomJoint(jointNames[l_leg_aky]).setQ(thighPitch - .3);  //0.087 + thighPitch);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_leg_aky]).setQ(thighPitch - .3); //0.087 + thighPitch);

      offset.setX(0.7);
      offset.setY(-0.07);
      offset.setZ(groundZ + 1.04);
      robot.setPositionInWorld(offset);
      robot.setOrientation(Math.PI / 2.0, forwardLean, 0.0);
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