package us.ihmc.humanoidRobotics.communication.packets.heightQuadTree;

import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.Packet;

public class HeightQuadTreeLeafMessage extends Packet<HeightQuadTreeLeafMessage>
{
   public float height;
   public float centerX;
   public float centerY;

   public HeightQuadTreeLeafMessage()
   {
   }

   @Override
   public void set(HeightQuadTreeLeafMessage other)
   {
      height = other.height;
      centerX = other.centerX;
      centerY = other.centerY;
   }

   @Override
   public boolean epsilonEquals(HeightQuadTreeLeafMessage other, double epsilon)
   {
      return MathTools.epsilonCompare(height, other.height, epsilon) && MathTools.epsilonCompare(centerX, other.centerX, epsilon)
            && MathTools.epsilonCompare(centerY, other.centerY, epsilon);
   }
}
