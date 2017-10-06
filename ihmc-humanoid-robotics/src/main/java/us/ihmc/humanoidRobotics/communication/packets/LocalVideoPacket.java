package us.ihmc.humanoidRobotics.communication.packets;

import java.awt.image.BufferedImage;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.packets.Packet;

public class LocalVideoPacket extends Packet<LocalVideoPacket>
{
   public long timeStamp;
   public BufferedImage image;
   public IntrinsicParameters intrinsicParameters;

   public LocalVideoPacket()
   {

   }

   public long getTimeStamp()
   {
      return timeStamp;
   }

   public BufferedImage getImage()
   {
      return image;
   }

   public IntrinsicParameters getIntrinsicParameters()
   {
      return intrinsicParameters;
   }

   public void setIntrinsicParameters(IntrinsicParameters intrinsicParameters)
   {
      this.intrinsicParameters = intrinsicParameters;
   }

   public LocalVideoPacket(long timeStamp, BufferedImage image, IntrinsicParameters intrinsicParameters)
   {
      this.timeStamp = timeStamp;
      this.image = image;
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

   @Override
   public boolean isClonable()
   {
      return false;
   }

}
