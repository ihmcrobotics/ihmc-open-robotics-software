package us.ihmc.escher;

import java.util.List;

import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class EscherInitialSetup implements DRCRobotInitialSetup<HumanoidFloatingRootJointRobot>
{
   private double groundZ;
   private double initialYaw;
   private final RigidBodyTransform rootToWorld = new RigidBodyTransform();
   private final Vector3D positionInWorld = new Vector3D();
   private final Vector3D offset = new Vector3D();
   private final Quaternion rotation = new Quaternion();
   private boolean robotInitialized = false;

   public EscherInitialSetup(double groundZ, double initialYaw)
   {
      this.groundZ = groundZ;
      this.initialYaw = initialYaw;
   }

   @Override
   public void initializeRobot(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      if(!robotInitialized)
      {
         setActuatorPositions(robot, jointMap);
         positionRobotInWorld(robot);
         robotInitialized = true;
      }
   }
   
   private void setActuatorPositions(FloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         String hipPitch = jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH);
         String knee = jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH);
         String anklePitch = jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH);
         String hipRoll = jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL);
         String ankleRoll = jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL);
         
         robot.getOneDegreeOfFreedomJoint(hipPitch).setQ(-0.5);
         robot.getOneDegreeOfFreedomJoint(knee).setQ(1.1);
         robot.getOneDegreeOfFreedomJoint(anklePitch).setQ(-0.55);
         robot.getOneDegreeOfFreedomJoint(hipRoll).setQ(0.0);
         robot.getOneDegreeOfFreedomJoint(ankleRoll).setQ(0.0);

         String shoulderRoll = jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL);
         String shoulderPitch = jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH);
         String elbowPitch = jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH);

         if (shoulderPitch != null)
            robot.getOneDegreeOfFreedomJoint(shoulderPitch).setQ(0.5);
         if (shoulderRoll != null)
            robot.getOneDegreeOfFreedomJoint(shoulderRoll).setQ(robotSide.negateIfRightSide(0.2));//inv
         if (elbowPitch != null)
            robot.getOneDegreeOfFreedomJoint(elbowPitch).setQ(0.5);//inv
      }
      robot.update();
   }
   
   private void positionRobotInWorld(HumanoidFloatingRootJointRobot robot)
   {
      robot.getRootJointToWorldTransform(rootToWorld);
      rootToWorld.get(rotation, positionInWorld);
      positionInWorld.setZ(groundZ + getPelvisToFoot(robot));
      positionInWorld.add(offset);
      robot.setPositionInWorld(positionInWorld);
      
      FrameQuaternion frameOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame(), rotation);
      double[] yawPitchRoll = new double[3];
      frameOrientation.getYawPitchRoll(yawPitchRoll);
      yawPitchRoll[0] = initialYaw;
      frameOrientation.setYawPitchRoll(yawPitchRoll);
      
      robot.setOrientation(frameOrientation);
      robot.update();
   }
   
   private double getPelvisToFoot(HumanoidFloatingRootJointRobot robot)
   {
      List<GroundContactPoint> contactPoints = robot.getFootGroundContactPoints(RobotSide.LEFT);
      double height = Double.POSITIVE_INFINITY;
      for(GroundContactPoint gc : contactPoints)
      {
         if(gc.getPositionPoint().getZ() < height)
         {
            height = gc.getPositionPoint().getZ();
         }
      }
      return offset.getZ() - height;
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
