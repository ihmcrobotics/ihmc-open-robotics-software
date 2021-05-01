package us.ihmc.atlas.initialSetup;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public class AtlasSimInitialSetup implements DRCRobotInitialSetup<HumanoidFloatingRootJointRobot>
{
   public static final Pose3D PELVIS_POSE = new Pose3D();
   public static final ArrayList<Double> JOINT_Qs = new ArrayList<>(31);
   static
   {
      PELVIS_POSE.getPosition().set(0.0, 0.0, 0.9286147465454951);
      PELVIS_POSE.getOrientation().set(0.0, 0.0, 0.841, 0.540); // not sure about these two values

      JOINT_Qs.add(+0.0             );
      JOINT_Qs.add(+0.0             );
      JOINT_Qs.add(+0.0             );
      JOINT_Qs.add(+0.785398        );
      JOINT_Qs.add(-0.52379         );
      JOINT_Qs.add(+2.33708         );
      JOINT_Qs.add(+2.35619         );
      JOINT_Qs.add(-0.337807        );
      JOINT_Qs.add(+0.20773         );
      JOINT_Qs.add(-0.026599        );
      JOINT_Qs.add(+0.0             );
      JOINT_Qs.add(+0.0             );
      JOINT_Qs.add(-JOINT_Qs.get(3) );
      JOINT_Qs.add(-JOINT_Qs.get(4) );
      JOINT_Qs.add(+JOINT_Qs.get(5) );
      JOINT_Qs.add(-JOINT_Qs.get(6) );
      JOINT_Qs.add(+JOINT_Qs.get(7) );
      JOINT_Qs.add(-JOINT_Qs.get(8) );
      JOINT_Qs.add(+JOINT_Qs.get(9) );
      JOINT_Qs.add(+0.0             );
      JOINT_Qs.add(+0.062           );
      JOINT_Qs.add(-0.233           );
      JOINT_Qs.add(+0.518           );
      JOINT_Qs.add(-0.276           );
      JOINT_Qs.add(-0.062           );
      JOINT_Qs.add(+JOINT_Qs.get(19));
      JOINT_Qs.add(-JOINT_Qs.get(20));
      JOINT_Qs.add(+JOINT_Qs.get(21));
      JOINT_Qs.add(+JOINT_Qs.get(22));
      JOINT_Qs.add(+JOINT_Qs.get(23));
      JOINT_Qs.add(-JOINT_Qs.get(24));
   }

   private double groundZ;
   private double initialYaw;
   private double initialX;
   private double initialY;
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
      this(groundZ, initialYaw, 0.0, 0.0);
   }

   public AtlasSimInitialSetup(double groundZ, double initialYaw, double initialX, double initialY)
   {
      this.groundZ = groundZ;
      this.initialYaw = initialYaw;
      this.initialX = initialX;
      this.initialY = initialY;
   }

   @Override
   public void initializeRobot(HumanoidFloatingRootJointRobot robot, HumanoidJointNameMap jointMap)
   {
      setActuatorPositions(robot, jointMap);
      positionRobotInWorld(robot);
   }

   public void setActuatorPositions(HumanoidFloatingRootJointRobot robot, HumanoidJointNameMap jointMap)
   {
      // Avoid singularities at startup
      for (RobotSide robotSide : RobotSide.values)
      {
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW)).setQ(JOINT_Qs.get(19)); //leg_hpz
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL)).setQ(robotSide.negateIfRightSide(JOINT_Qs.get(20))); //leg_hpx
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH)).setQ(JOINT_Qs.get(21)); //leg_hpy
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH)).setQ(JOINT_Qs.get(22)); //leg_kny
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH)).setQ(JOINT_Qs.get(23)); //leg_aky
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL)).setQ(robotSide.negateIfRightSide(JOINT_Qs.get(24))); //leg_akx

         setArmJointPosition(robot, jointMap, robotSide, ArmJointName.SHOULDER_YAW, robotSide.negateIfRightSide(JOINT_Qs.get(3))); //arm_shz
         setArmJointPosition(robot, jointMap, robotSide, ArmJointName.SHOULDER_ROLL, robotSide.negateIfRightSide(JOINT_Qs.get(4))); //arm_shx
         setArmJointPosition(robot, jointMap, robotSide, ArmJointName.ELBOW_PITCH, JOINT_Qs.get(5)); //arm_ely
         setArmJointPosition(robot, jointMap, robotSide, ArmJointName.ELBOW_ROLL, robotSide.negateIfRightSide(JOINT_Qs.get(6))); //arm_elx
         setArmJointPosition(robot, jointMap, robotSide, ArmJointName.FIRST_WRIST_PITCH, JOINT_Qs.get(7)); //arm_wry
         setArmJointPosition(robot, jointMap, robotSide, ArmJointName.WRIST_ROLL, robotSide.negateIfRightSide(JOINT_Qs.get(8))); //arm_wrx
         setArmJointPosition(robot, jointMap, robotSide, ArmJointName.SECOND_WRIST_PITCH, JOINT_Qs.get(9)); //arm_wry2
      }

      robot.update();
   }

   private void setArmJointPosition(HumanoidFloatingRootJointRobot robot, HumanoidJointNameMap jointMap, RobotSide robotSide, ArmJointName armJointName, double q)
   {
      String armJointString = jointMap.getArmJointName(robotSide, armJointName);
      if (armJointString == null)
         return;
      OneDegreeOfFreedomJoint joint = robot.getOneDegreeOfFreedomJoint(armJointString);
      if (joint == null)
         return;
      joint.setQ(q);
   }

   public void positionRobotInWorld(HumanoidFloatingRootJointRobot robot)
   {
      robot.getRootJoint().setPosition(0.0, 0.0, 0.0);
      robot.update();
      robot.getRootJointToWorldTransform(rootToWorld);
      rootToWorld.get(rotation, positionInWorld);
      positionInWorld.add(offset);
      positionInWorld.addZ(groundZ - getLowestFootContactPointHeight(robot));
      robot.setPositionInWorld(positionInWorld);

      FrameQuaternion frameOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame(), rotation);
      YawPitchRoll yawPitchRoll = new YawPitchRoll(frameOrientation);
      yawPitchRoll.setYaw(initialYaw);
      frameOrientation.set(yawPitchRoll);

      robot.setOrientation(frameOrientation);
      robot.update();
   }

   private double getLowestFootContactPointHeight(HumanoidFloatingRootJointRobot robot)
   {
      List<GroundContactPoint> contactPoints = robot.getFootGroundContactPoints(RobotSide.LEFT);
      double height = Double.POSITIVE_INFINITY;

      if (contactPoints.size() == 0)
      {
         height = -PELVIS_POSE.getZ();
      }
      else
      {
         for (GroundContactPoint gc : contactPoints)
         {
            height = Math.min(height, gc.getPositionCopy().getZ());
         }
      }

      return height;
   }

   @Override
   public List<Double> getInitialJointAngles()
   {
      return JOINT_Qs;
   }

   @Override
   public Pose3DReadOnly getInitialPelvisPose()
   {
      Pose3D resultPose = new Pose3D();
      resultPose.appendTranslation(initialX, initialY, groundZ + PELVIS_POSE.getZ());
      resultPose.appendTranslation(offset);
      resultPose.appendYawRotation(initialYaw);

      return resultPose;
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

   @Override
   public boolean supportsReset()
   {
      return true;
   }
}
