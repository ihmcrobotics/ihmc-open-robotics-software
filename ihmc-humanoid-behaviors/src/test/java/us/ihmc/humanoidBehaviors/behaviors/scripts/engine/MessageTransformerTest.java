package us.ihmc.humanoidBehaviors.behaviors.scripts.engine;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.packets.EuclideanTrajectoryPointMessage;
import us.ihmc.humanoidRobotics.communication.packets.SE3TrajectoryPointMessage;
import us.ihmc.humanoidRobotics.communication.packets.driving.VehiclePosePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.AdjustFootstepMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;

public class MessageTransformerTest
{
   @Test(timeout = 30000)
   public void testHandTrajectoryMessage()
   {
      Random random = new Random(6543);

      HandTrajectoryMessage original = new HandTrajectoryMessage(random);

      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      
      HandTrajectoryMessage expected = new HandTrajectoryMessage(original);
      for (SE3TrajectoryPointMessage trajectoryPoint : expected.se3Trajectory.taskspaceTrajectoryPoints)
      {
         trajectoryPoint.position.applyTransform(transform);
         trajectoryPoint.orientation.applyTransform(transform);
         trajectoryPoint.linearVelocity.applyTransform(transform);
         trajectoryPoint.angularVelocity.applyTransform(transform);
      }

      HandTrajectoryMessage actual = new HandTrajectoryMessage(original);

      MessageTransformer.transform(actual, transform);

      assertTrue(expected.epsilonEquals(actual, 1.0e-5));
   }

   @Test(timeout = 30000)
   public void testPelvisHeightTrajectoryMessage()
   {
      Random random = new Random(6543);

      PelvisHeightTrajectoryMessage original = new PelvisHeightTrajectoryMessage(random);

      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      
      PelvisHeightTrajectoryMessage expected = new PelvisHeightTrajectoryMessage(original);
      for (EuclideanTrajectoryPointMessage trajectoryPoint : expected.euclideanTrajectory.taskspaceTrajectoryPoints)
      {
         trajectoryPoint.position.applyTransform(transform);
         trajectoryPoint.linearVelocity.applyTransform(transform);
      }

      PelvisHeightTrajectoryMessage actual = new PelvisHeightTrajectoryMessage(original);

      MessageTransformer.transform(actual, transform);

      assertTrue(expected.epsilonEquals(actual, 1.0e-5));
   }

   @Test(timeout = 30000)
   public void testAdjustFootstepMessage()
   {
      Random random = new Random(6543);

      AdjustFootstepMessage original = new AdjustFootstepMessage(random);

      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      
      AdjustFootstepMessage expected = new AdjustFootstepMessage(original);
      expected.location.applyTransform(transform);
      expected.orientation.applyTransform(transform);

      AdjustFootstepMessage actual = new AdjustFootstepMessage(original);
      MessageTransformer.transform(actual, transform);

      assertTrue(expected.epsilonEquals(actual, 1.0e-5));
   }

   @Test(timeout = 30000)
   public void testFootstepDataMessage()
   {
      Random random = new Random(6543);

      FootstepDataMessage original = new FootstepDataMessage(random);

      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      
      FootstepDataMessage expected = new FootstepDataMessage(original);
      expected.location.applyTransform(transform);
      expected.orientation.applyTransform(transform);
      for (Point3D waypoint : expected.positionWaypoints)
         waypoint.applyTransform(transform);

      FootstepDataMessage actual = new FootstepDataMessage(original);
      MessageTransformer.transform(actual, transform);

      assertTrue(expected.epsilonEquals(actual, 1.0e-5));
   }

   @Test(timeout = 30000)
   public void testFootstepDataListMessage()
   {
      Random random = new Random(6543);

      FootstepDataListMessage original = new FootstepDataListMessage(random);

      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      
      FootstepDataListMessage expected = new FootstepDataListMessage(original);
      for (FootstepDataMessage footstepDataMessage : expected.footstepDataList)
      {
         footstepDataMessage.location.applyTransform(transform);
         footstepDataMessage.orientation.applyTransform(transform);
         for (Point3D waypoint : footstepDataMessage.positionWaypoints)
            waypoint.applyTransform(transform);
      }

      FootstepDataListMessage actual = new FootstepDataListMessage(original);
      MessageTransformer.transform(actual, transform);

      assertTrue(expected.epsilonEquals(actual, 1.0e-5));
   }

   @Test(timeout = 30000)
   public void testVehiclePosePacket()
   {
      Random random = new Random(6543);

      VehiclePosePacket original = new VehiclePosePacket(random);

      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      
      VehiclePosePacket expected = new VehiclePosePacket(original);
      expected.position.applyTransform(transform);
      expected.orientation.applyTransform(transform);

      VehiclePosePacket actual = new VehiclePosePacket(original);
      MessageTransformer.transform(actual, transform);

      assertTrue(expected.epsilonEquals(actual, 1.0e-5));
   }

   @Test(timeout = 30000)
   public void testVideoPacket()
   {
      Random random = new Random(6543);

      VideoPacket original = new VideoPacket(random);

      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      
      VideoPacket expected = new VideoPacket(original);
      expected.position.applyTransform(transform);
      expected.orientation.applyTransform(transform);

      VideoPacket actual = new VideoPacket(original);
      MessageTransformer.transform(actual, transform);

      assertTrue(expected.epsilonEquals(actual, 1.0e-5));
   }
}
