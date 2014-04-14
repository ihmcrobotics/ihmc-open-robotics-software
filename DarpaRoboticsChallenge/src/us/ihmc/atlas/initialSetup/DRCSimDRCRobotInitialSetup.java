package us.ihmc.atlas.initialSetup;

import static us.ihmc.atlas.ros.AtlasOrderedJointMap.jointNames;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_elx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_ely;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_shx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_shy;
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
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_arm_shy;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_arm_wrx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_arm_wry;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_leg_akx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_leg_aky;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_leg_hpx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_leg_hpy;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_leg_hpz;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_leg_kny;

import javax.media.j3d.Transform3D;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.GroundContactPoint;

public class DRCSimDRCRobotInitialSetup implements DRCRobotInitialSetup<SDFRobot>
{
   private final double groundZ;
   private final double initialYaw;
   private final Transform3D rootToWorld = new Transform3D();
   private final Vector3d offset = new Vector3d();
   private final Quat4d rotation = new Quat4d();

   public DRCSimDRCRobotInitialSetup()
   {
      this(0.0, 0.0);
   }

   public DRCSimDRCRobotInitialSetup(double groundZ, double initialYaw)
   {
      this.groundZ = groundZ;
      this.initialYaw = initialYaw;
   }

   public void initializeRobot(SDFRobot robot, DRCRobotJointMap jointMap)
   {
      // Avoid singularities at startup
           
      robot.getOneDegreeOfFreedomJoint(jointNames[l_leg_hpz]).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_leg_hpx]).setQ(0.062);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_leg_hpy]).setQ(-0.233);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_leg_kny]).setQ(0.518);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_leg_aky]).setQ(-0.276);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_leg_akx]).setQ(-0.062);
      
      
      robot.getOneDegreeOfFreedomJoint(jointNames[r_leg_hpz]).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_leg_hpx]).setQ(-0.062);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_leg_hpy]).setQ(-0.233);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_leg_kny]).setQ(0.518);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_leg_aky]).setQ(-0.276);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_leg_akx]).setQ(0.062);

      
      robot.getOneDegreeOfFreedomJoint(jointNames[l_arm_shy]).setQ(0.300);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_arm_shx]).setQ(-1.30);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_arm_ely]).setQ(2.00);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_arm_elx]).setQ(0.498);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_arm_wry]).setQ(0.000);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_arm_wrx]).setQ(-0.004);
      
      robot.getOneDegreeOfFreedomJoint(jointNames[r_arm_shy]).setQ(0.300);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_arm_shx]).setQ(1.30);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_arm_ely]).setQ(2.00);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_arm_elx]).setQ(-0.498);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_arm_wry]).setQ(0.000);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_arm_wrx]).setQ(0.004);
      
      robot.update();
      robot.getRootJointToWorldTransform(rootToWorld);
      rootToWorld.get(rotation, offset);

    GroundContactPoint gc1 = robot.getFootGroundContactPoints(RobotSide.LEFT).get(0);
    double pelvisToFoot = offset.getZ() - gc1.getPositionPoint().getZ();

      // Hardcoded for gazebo integration
//      double pelvisToFoot = 0.887;


      offset.setZ(groundZ + pelvisToFoot);

//    offset.add(robot.getPositionInWorld());
      robot.setPositionInWorld(offset);
      
      FrameOrientation frameOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), rotation);
      double[] yawPitchRoll = frameOrientation.getYawPitchRoll();
      yawPitchRoll[0] = initialYaw;
      frameOrientation.setYawPitchRoll(yawPitchRoll);
      
      robot.setOrientation(frameOrientation.getQuaternion());
      
      robot.update();
   }

   public void getOffset(Vector3d offsetToPack)
   {
      offsetToPack.set(offset);
   }

   public void setOffset(Vector3d offset)
   {
      this.offset.set(offset);
   }
}
