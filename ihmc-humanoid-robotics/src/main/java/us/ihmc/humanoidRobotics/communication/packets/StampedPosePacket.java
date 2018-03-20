package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.geometry.Pose3D;

public class StampedPosePacket extends Packet<StampedPosePacket>
{
   public Pose3D pose;
   public long timeStamp;
   public double confidenceFactor;
   public StringBuilder frameId = new StringBuilder();

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
      frameId.setLength(0);
      frameId.append(other.frameId);
      setPacketInformation(other);
   }
   
   public String getFrameIdAsString()
   {
      return frameId.toString();
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
      boolean ret = frameId.equals(other.frameId);
      ret &= pose.epsilonEquals(other.getPose(), epsilon);
      return ret;
   }
}
