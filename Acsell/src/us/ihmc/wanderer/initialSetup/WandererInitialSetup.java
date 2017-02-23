package us.ihmc.wanderer.initialSetup;

import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

/**
 * Created by dstephen on 2/14/14.
 */
public class WandererInitialSetup implements DRCRobotInitialSetup<HumanoidFloatingRootJointRobot>
{
   private double groundZ;
   private double initialYaw;
   private final RigidBodyTransform rootToWorld = new RigidBodyTransform();
   private final Vector3D offset = new Vector3D();
   private final Quaternion rotation = new Quaternion();

   public WandererInitialSetup(double groundZ, double initialYaw)
   {
      this.groundZ = groundZ;
      this.initialYaw = initialYaw;
   }

   public void initializeRobot(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      for(RobotSide robotSide : RobotSide.values())
      {
         String prefix = robotSide.getSideNameFirstLetter().toLowerCase();

         robot.getOneDegreeOfFreedomJoint(prefix + "_leg_lhy").setQ(-0.4);
         robot.getOneDegreeOfFreedomJoint(prefix + "_leg_kny").setQ(0.8);
         robot.getOneDegreeOfFreedomJoint(prefix + "_leg_uay").setQ((-0.4));
      }

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

      robot.setOrientation(frameOrientation.getQuaternionCopy());

      robot.update();
   }
   
   public void getOffset(Vector3D offsetToPack)
   {
      offsetToPack.set(offset);
   }

   public void setOffset(Vector3D offset)
   {
      this.offset.set(offset);
   }

   public void setInitialYaw(double yaw)
   {
      initialYaw = yaw;
   }

   public void setInitialGroundHeight(double groundHeight)
   {
      groundZ = groundHeight;
   }

   public double getInitialYaw()
   {
      return initialYaw;
   }

   public double getInitialGroundHeight()
   {
      return groundZ;
   }
}
