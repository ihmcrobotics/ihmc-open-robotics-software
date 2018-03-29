package us.ihmc.humanoidRobotics.communication.packets;

import java.awt.image.BufferedImage;

import us.ihmc.communication.packets.Packet;

public class LocalVideoPacket extends Packet<LocalVideoPacket>
{
   public long timeStamp;
   public BufferedImage image;
   public IntrinsicParametersMessage intrinsicParameters;

   public LocalVideoPacket()
   {

   }

   @Override
   public void set(LocalVideoPacket other)
   {
      timeStamp = other.timeStamp;
      image = other.image;
      intrinsicParameters =  other.intrinsicParameters;
      setPacketInformation(other);
   }

   public long getTimeStamp()
   {
      return timeStamp;
   }

   public BufferedImage getImage()
   {
      return image;
   }

   public IntrinsicParametersMessage getIntrinsicParameters()
   {
      return intrinsicParameters;
   }

   public void setIntrinsicParameters(IntrinsicParametersMessage intrinsicParameters)
   {
      this.intrinsicParameters = intrinsicParameters;
   }

   @Override
   public boolean epsilonEquals(LocalVideoPacket other, double epsilon)
   {
      boolean ret = timeStamp == other.timeStamp;
      ret &= image.equals(other.image);
      ret &= intrinsicParameters.equals(other.intrinsicParameters);
      return ret;
   }
}
