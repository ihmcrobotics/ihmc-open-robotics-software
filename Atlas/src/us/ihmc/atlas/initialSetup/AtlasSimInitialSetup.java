package us.ihmc.atlas.initialSetup;

import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class AtlasSimInitialSetup implements DRCRobotInitialSetup<HumanoidFloatingRootJointRobot>
{
   private double groundZ;
   private double initialYaw;
   private final RigidBodyTransform rootToWorld = new RigidBodyTransform();
   private final Vector3D positionInWorld = new Vector3D();
   private final Vector3D offset = new Vector3D();
   private final Quaternion rotation = new Quaternion();
   private boolean robotInitialized = false;

   public AtlasSimInitialSetup()
   {
      this(0.0, 0.0);
   }

   public AtlasSimInitialSetup(double groundZ, double initialYaw)
   {
      this.groundZ = groundZ;
      this.initialYaw = initialYaw;
   }

   @Override
   public void initializeRobot(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      if(!robotInitialized)
      {
         // Avoid singularities at startup
         for (RobotSide robotSide : RobotSide.values)
         {
            robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW)).setQ(0.0); //leg_hpz
            robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL)).setQ(robotSide.negateIfRightSide(0.062)); //leg_hpx
            robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH)).setQ(-0.233); //leg_hpy
            robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH)).setQ(0.518); //leg_kny
            robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH)).setQ(-0.276); //leg_aky
            robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL)).setQ(robotSide.negateIfRightSide(-0.062)); //leg_akx
            
            robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW)).setQ(robotSide.negateIfRightSide(0.785398)); //arm_shz
            robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL)).setQ(robotSide.negateIfRightSide(-0.1)); //arm_shx
            robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH)).setQ(3.00); //arm_ely
            robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_ROLL)).setQ(robotSide.negateIfRightSide(1.8)); //arm_elx
            robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.FIRST_WRIST_PITCH)).setQ(-0.30); //arm_wry
            robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.WRIST_ROLL)).setQ(robotSide.negateIfRightSide(0.70)); //arm_wrx
            robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SECOND_WRIST_PITCH)).setQ(0.15); //arm_wry2
         }
         
         robot.update();
         robot.getRootJointToWorldTransform(rootToWorld);
         rootToWorld.get(rotation, positionInWorld);
         
         GroundContactPoint gc1 = robot.getFootGroundContactPoints(RobotSide.LEFT).get(0);
         double pelvisToFoot = positionInWorld.getZ() - gc1.getPositionPoint().getZ();
         
         // Hardcoded for gazebo integration
         //      double pelvisToFoot = 0.887;
         
         positionInWorld.setZ(groundZ + pelvisToFoot);
         positionInWorld.add(offset);
         
         robot.setPositionInWorld(positionInWorld);
         
         FrameOrientation frameOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), rotation);
         double[] yawPitchRoll = frameOrientation.getYawPitchRoll();
         yawPitchRoll[0] = initialYaw;
         frameOrientation.setYawPitchRoll(yawPitchRoll);
         
         robot.setOrientation(frameOrientation.getQuaternionCopy());
         robot.update();
         robotInitialized = true;
      }
   }

   @Override
   public void getOffset(Vector3D offsetToPack)
   {
      offsetToPack.set(offset);
   }

   @Override
   public void setOffset(Vector3D offset)
   {
      this.offset.set(offset);
   }

   @Override
   public void setInitialYaw(double yaw)
   {
      initialYaw = yaw;
   }

   @Override
   public void setInitialGroundHeight(double groundHeight)
   {
      groundZ = groundHeight;
   }

   @Override
   public double getInitialYaw()
   {
      return initialYaw;
   }

   @Override
   public double getInitialGroundHeight()
   {
      return groundZ;
   }
}
