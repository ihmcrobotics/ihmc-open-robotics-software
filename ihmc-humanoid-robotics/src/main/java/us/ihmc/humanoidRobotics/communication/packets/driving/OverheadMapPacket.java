package us.ihmc.humanoidRobotics.communication.packets.driving;

import java.awt.image.BufferedImage;
import java.util.Random;

import us.ihmc.communication.packets.Packet;

public class OverheadMapPacket extends Packet<OverheadMapPacket>
{
   public BufferedImage image;
   public int[] sequence;

   public OverheadMapPacket()
   {
   }

   public OverheadMapPacket(BufferedImage image, int[] sequence)
   {
      this.image = image;
      this.sequence = sequence;
   }

   public BufferedImage getImage()
   {
      return image;
   }

   public int[] getSequence()
   {
      return sequence;
   }

   public boolean equals(Object other)
   {
      if (other instanceof OverheadMapPacket)
      {
         return epsilonEquals((OverheadMapPacket) other, 0);
      }
      else
      {
         return false;
      }
   }

   @Override
   public boolean epsilonEquals(OverheadMapPacket other, double epsilon)
   {
      int[] sequence2 = ((OverheadMapPacket) other).getSequence();
      if (sequence2.length != getSequence().length)
      {
         return false;
      }

      for (int i = 0; i < getSequence().length; i++)
      {
         if (sequence2[i] != getSequence()[i])
         {
            return false;
         }
      }

      BufferedImage otherImg = ((OverheadMapPacket) other).getImage();

      return (otherImg.getWidth() == image.getWidth()) && (otherImg.getHeight() == image.getHeight());
   }

   public OverheadMapPacket(Random random)
   {
      int[] sequence = new int[random.nextInt(255)];
      for (int i = 0; i < sequence.length; i++)
      {
         sequence[i] = random.nextInt(255);
      }

      this.image = new BufferedImage(200, 300, BufferedImage.TYPE_3BYTE_BGR);
      this.sequence = sequence;

   }
}
