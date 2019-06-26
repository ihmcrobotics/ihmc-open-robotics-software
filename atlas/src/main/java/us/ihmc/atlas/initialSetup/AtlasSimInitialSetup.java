package us.ihmc.atlas.initialSetup;

import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class AtlasSimInitialSetup implements DRCRobotInitialSetup<HumanoidFloatingRootJointRobot>
{
   private double groundZ;
   private double initialYaw;
   private final RigidBodyTransform rootToWorld = new RigidBodyTransform();
   private final Vector3D positionInWorld = new Vector3D();
   private final Vector3D offset = new Vector3D();
   private final Quaternion rotation = new Quaternion();

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
      OneDegreeOfFreedomJoint[] joints = robot.getOneDegreeOfFreedomJoints();
      FloatingJoint rootJoint = robot.getRootJoint();

      for (int i = 0; i < joints.length; i++)
      {
         joints[i].setQ(0.0);
         joints[i].setQd(0.0);
         joints[i].setQdd(0.0);
         joints[i].setTau(0.0);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW)).setQ(0.0); //leg_hpz
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL)).setQ(robotSide.negateIfRightSide(0.062)); //leg_hpx
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH)).setQ(-0.233); //leg_hpy
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH)).setQ(0.518); //leg_kny
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH)).setQ(-0.276); //leg_aky
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL)).setQ(robotSide.negateIfRightSide(-0.062)); //leg_akx

         setArmJointPosition(robot, jointMap, robotSide, ArmJointName.SHOULDER_YAW, robotSide.negateIfRightSide(0.785398)); //arm_shz
         setArmJointPosition(robot, jointMap, robotSide, ArmJointName.SHOULDER_ROLL, robotSide.negateIfRightSide(-0.52379)); //arm_shx
         setArmJointPosition(robot, jointMap, robotSide, ArmJointName.ELBOW_PITCH, 2.33708); //arm_ely
         setArmJointPosition(robot, jointMap, robotSide, ArmJointName.ELBOW_ROLL, robotSide.negateIfRightSide(2.35619)); //arm_elx
         setArmJointPosition(robot, jointMap, robotSide, ArmJointName.FIRST_WRIST_PITCH, -0.337807); //arm_wry
         setArmJointPosition(robot, jointMap, robotSide, ArmJointName.WRIST_ROLL, robotSide.negateIfRightSide(0.207730)); //arm_wrx
         setArmJointPosition(robot, jointMap, robotSide, ArmJointName.SECOND_WRIST_PITCH, -0.026599); //arm_wry2
      }

      rootToWorld.setIdentity();
      rootJoint.setRotationAndTranslation(rootToWorld);
      rootJoint.setVelocity(new Vector3D());
      rootJoint.setAngularVelocityInBody(new Vector3D());
      rootJoint.setAcceleration(new Vector3D());
      rootJoint.setAngularAccelerationInBody(new Vector3D());
      robot.update();

      double pelvisToFoot = computeHeightOffset(robot);

      rootToWorld.setTranslationZ(pelvisToFoot);
      rootToWorld.appendTranslation(offset);
      rootToWorld.appendYawRotation(initialYaw);
      rootJoint.setRotationAndTranslation(rootToWorld);
      robot.update();

      robot.getGroundContactPoints(0).forEach(point -> point.setTouchdownToCurrentLocation());
   }

   private double computeHeightOffset(HumanoidFloatingRootJointRobot robot)
   {
      GroundContactPoint lowestGroundContactPoint = null;
      double minZ = Double.NaN;

      for (RobotSide robotSide : RobotSide.values)
      {
         for (GroundContactPoint groundContactPoint : robot.getFootGroundContactPoints(robotSide))
         {
            if (Double.isNaN(minZ) || Double.isInfinite(minZ))
            {
               minZ = groundContactPoint.getPositionPoint().getZ();
               lowestGroundContactPoint = groundContactPoint;
            }
            else if (groundContactPoint.getPositionPoint().getZ() <= minZ)
            {
               minZ = groundContactPoint.getPositionPoint().getZ();
               lowestGroundContactPoint = groundContactPoint;
            }
         }
      }

      double pelvisToFoot;
      if (lowestGroundContactPoint == null)
         pelvisToFoot = 0.9286147465454951;
      else
         pelvisToFoot = positionInWorld.getZ() - lowestGroundContactPoint.getPositionPoint().getZ();
      return pelvisToFoot;
   }

   private void setArmJointPosition(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap, RobotSide robotSide, ArmJointName armJointName, double q)
   {
      String armJointString = jointMap.getArmJointName(robotSide, armJointName);
      if (armJointString == null)
         return;
      OneDegreeOfFreedomJoint joint = robot.getOneDegreeOfFreedomJoint(armJointString);
      if (joint == null)
         return;
      joint.setQ(q);
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
