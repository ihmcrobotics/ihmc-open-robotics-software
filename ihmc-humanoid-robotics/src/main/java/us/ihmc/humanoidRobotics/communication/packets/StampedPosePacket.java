package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.geometry.Pose3D;

public class StampedPosePacket extends Packet<StampedPosePacket>
{
   public Pose3D pose;
   public long timeStamp;
   public double confidenceFactor;
   public String frameId;

   public StampedPosePacket()
   {
      // Empty constructor for deserialization
   }

   @Override
   public void set(StampedPosePacket other)
   {
      pose = new Pose3D(other.pose);
      timeStamp = other.timeStamp;
      confidenceFactor = other.confidenceFactor;
      frameId = other.frameId;
      setPacketInformation(other);
   }

   public String getFrameId()
   {
      return frameId;
   }

   public Pose3D getPose()
   {
      return pose;
   }

   public long getTimeStamp()
   {
      return timeStamp;
   }

   public double getConfidenceFactor()
   {
      return confidenceFactor;
   }

   @Override
   public boolean epsilonEquals(StampedPosePacket other, double epsilon)
   {
      boolean ret = frameId.equals(other.getFrameId());
      ret &= pose.epsilonEquals(other.getPose(), epsilon);
      return ret;
   }
}
