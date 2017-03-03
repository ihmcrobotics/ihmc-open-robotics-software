package us.ihmc.humanoidRobotics.communication.packets;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;

public class StampedPosePacket extends Packet<StampedPosePacket>
{
   public TimeStampedTransform3D transform;
   public double confidenceFactor;
   public String frameId;

   public StampedPosePacket()
   {
      // Empty constructor for deserialization
   }

   public StampedPosePacket(String frameId, TimeStampedTransform3D transform, double confidenceFactor)
   {
      this.frameId = frameId;
      this.transform = transform;
      this.confidenceFactor = confidenceFactor;
   }

   public String getFrameId()
   {
      return frameId;
   }

   public TimeStampedTransform3D getTransform()
   {
      return transform;
   }

   public double getConfidenceFactor()
   {
      return confidenceFactor;
   }

   @Override
   public boolean epsilonEquals(StampedPosePacket other, double epsilon)
   {
      boolean ret = frameId.equals(other.getFrameId());
      ret &= transform.epsilonEquals(other.getTransform(), epsilon);
      return ret;
   }

   public StampedPosePacket(Random random)
   {
      this("/pelvis", new TimeStampedTransform3D(EuclidCoreRandomTools.generateRandomRigidBodyTransform(random), random.nextLong()), 1.0);
   }
}
