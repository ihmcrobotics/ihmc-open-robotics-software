package us.ihmc.humanoidBehaviors.behaviors.scripts.engine;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.AdjustFootstepMessage;
import controller_msgs.msg.dds.EuclideanTrajectoryPointMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.PelvisHeightTrajectoryMessage;
import controller_msgs.msg.dds.QuadrupedBodyHeightMessage;
import controller_msgs.msg.dds.SE3TrajectoryPointMessage;
import controller_msgs.msg.dds.VehiclePosePacket;
import controller_msgs.msg.dds.VideoPacket;
import controller_msgs.msg.dds.WrenchTrajectoryPointMessage;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.packets.RandomHumanoidMessages;

public class MessageTransformerTest
{
   @Test
   public void testHandTrajectoryMessage()
   {
      Random random = new Random(6543);

      HandTrajectoryMessage original = RandomHumanoidMessages.nextHandTrajectoryMessage(random);

      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      HandTrajectoryMessage expected = new HandTrajectoryMessage(original);
      for (int i = 0; i < expected.getSe3Trajectory().getTaskspaceTrajectoryPoints().size(); i++)
      {
         SE3TrajectoryPointMessage trajectoryPoint = expected.getSe3Trajectory().getTaskspaceTrajectoryPoints().get(i);
         trajectoryPoint.getPosition().applyTransform(transform);
         trajectoryPoint.getOrientation().applyTransform(transform);
         trajectoryPoint.getLinearVelocity().applyTransform(transform);
         trajectoryPoint.getAngularVelocity().applyTransform(transform);
      }
      for (int i = 0; i < expected.getWrenchTrajectory().getWrenchTrajectoryPoints().size(); i++)
      {
         WrenchTrajectoryPointMessage trajectoryPoint = expected.getWrenchTrajectory().getWrenchTrajectoryPoints().get(i);
         trajectoryPoint.getWrench().getForce().applyTransform(transform);
         trajectoryPoint.getWrench().getTorque().applyTransform(transform);
      }

      HandTrajectoryMessage actual = new HandTrajectoryMessage(original);

      MessageTransformer.transform(actual, transform);

      assertTrue(expected.epsilonEquals(actual, 1.0e-5));
   }

   @Test
   public void testPelvisHeightTrajectoryMessage()
   {
      Random random = new Random(6543);

      PelvisHeightTrajectoryMessage original = RandomHumanoidMessages.nextPelvisHeightTrajectoryMessage(random);

      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      PelvisHeightTrajectoryMessage expected = new PelvisHeightTrajectoryMessage(original);
      for (int i = 0; i < expected.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().size(); i++)
      {
         EuclideanTrajectoryPointMessage trajectoryPoint = expected.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().get(i);
         trajectoryPoint.getPosition().applyTransform(transform);
         trajectoryPoint.getLinearVelocity().applyTransform(transform);
      }

      PelvisHeightTrajectoryMessage actual = new PelvisHeightTrajectoryMessage(original);

      MessageTransformer.transform(actual, transform);

      assertTrue(expected.epsilonEquals(actual, 1.0e-5));
   }

   @Test
   public void testQuadrupedBodyHeightMessage()
   {
      Random random = new Random(6543);

      QuadrupedBodyHeightMessage original = RandomHumanoidMessages.nextQuadrupedBodyHeightMessage(random);

      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      QuadrupedBodyHeightMessage expected = new QuadrupedBodyHeightMessage(original);
      for (int i = 0; i < expected.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().size(); i++)
      {
         EuclideanTrajectoryPointMessage trajectoryPoint = expected.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().get(i);
         trajectoryPoint.getPosition().applyTransform(transform);
         trajectoryPoint.getLinearVelocity().applyTransform(transform);
      }

      QuadrupedBodyHeightMessage actual = new QuadrupedBodyHeightMessage(original);

      MessageTransformer.transform(actual, transform);

      assertTrue(expected.epsilonEquals(actual, 1.0e-5));
   }

   @Test
   public void testAdjustFootstepMessage()
   {
      Random random = new Random(6543);

      AdjustFootstepMessage original = RandomHumanoidMessages.nextAdjustFootstepMessage(random);

      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      AdjustFootstepMessage expected = new AdjustFootstepMessage(original);
      expected.getLocation().applyTransform(transform);
      expected.getOrientation().applyTransform(transform);

      AdjustFootstepMessage actual = new AdjustFootstepMessage(original);
      MessageTransformer.transform(actual, transform);

      assertTrue(expected.epsilonEquals(actual, 1.0e-5));
   }

   @Test
   public void testFootstepDataMessage()
   {
      Random random = new Random(6543);

      FootstepDataMessage original = RandomHumanoidMessages.nextFootstepDataMessage(random);

      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      FootstepDataMessage expected = new FootstepDataMessage(original);
      expected.getLocation().applyTransform(transform);
      expected.getOrientation().applyTransform(transform);
      List<Point3D> positionWaypoints = expected.getCustomPositionWaypoints();
      for (int i = 0; i < positionWaypoints.size(); i++)
      {
         Point3D waypoint = positionWaypoints.get(i);
         waypoint.applyTransform(transform);
      }

      FootstepDataMessage actual = new FootstepDataMessage(original);
      MessageTransformer.transform(actual, transform);

      assertTrue(expected.epsilonEquals(actual, 1.0e-5));
   }

   @Test
   public void testFootstepDataListMessage()
   {
      Random random = new Random(6543);

      FootstepDataListMessage original = RandomHumanoidMessages.nextFootstepDataListMessage(random);

      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      FootstepDataListMessage expected = new FootstepDataListMessage(original);
      List<FootstepDataMessage> footstepDataList = expected.getFootstepDataList();
      for (int i = 0; i < footstepDataList.size(); i++)
      {
         FootstepDataMessage footstepDataMessage = footstepDataList.get(i);
         footstepDataMessage.getLocation().applyTransform(transform);
         footstepDataMessage.getOrientation().applyTransform(transform);
         for (Point3D waypoint : footstepDataMessage.getCustomPositionWaypoints())
            waypoint.applyTransform(transform);
      }

      FootstepDataListMessage actual = new FootstepDataListMessage(original);
      MessageTransformer.transform(actual, transform);

      assertTrue(expected.epsilonEquals(actual, 1.0e-5));
   }

   @Test
   public void testVehiclePosePacket()
   {
      Random random = new Random(6543);

      VehiclePosePacket original = RandomHumanoidMessages.nextVehiclePosePacket(random);

      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      VehiclePosePacket expected = new VehiclePosePacket(original);
      expected.getPosition().applyTransform(transform);
      expected.getOrientation().applyTransform(transform);

      VehiclePosePacket actual = new VehiclePosePacket(original);
      MessageTransformer.transform(actual, transform);

      assertTrue(expected.epsilonEquals(actual, 1.0e-5));
   }

   @Test
   public void testVideoPacket()
   {
      Random random = new Random(6543);

      VideoPacket original = RandomHumanoidMessages.nextVideoPacket(random);

      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      VideoPacket expected = new VideoPacket(original);
      expected.getPosition().applyTransform(transform);
      expected.getOrientation().applyTransform(transform);

      VideoPacket actual = new VideoPacket(original);
      MessageTransformer.transform(actual, transform);

      assertTrue(expected.epsilonEquals(actual, 1.0e-5));
   }
}
