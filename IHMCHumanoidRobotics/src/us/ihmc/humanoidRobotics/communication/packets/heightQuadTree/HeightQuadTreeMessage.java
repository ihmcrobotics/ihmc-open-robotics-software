package us.ihmc.humanoidRobotics.communication.packets.heightQuadTree;

import us.ihmc.communication.packets.StatusPacket;

public class HeightQuadTreeMessage extends StatusPacket<HeightQuadTreeMessage>
{
   public HeightQuadTreeNodeMessage root;
   public float defaultHeight = Float.NaN;

   public HeightQuadTreeMessage()
   {
   }

   @Override
   public void set(HeightQuadTreeMessage other)
   {
      root = new HeightQuadTreeNodeMessage();
      root.set(other.root);
      defaultHeight = other.defaultHeight;
   }

   @Override
   public boolean epsilonEquals(HeightQuadTreeMessage other, double epsilon)
   {
      if (Float.compare(defaultHeight, other.defaultHeight) != 0)
         return false;
      return root.epsilonEquals(other.root, epsilon);
   }
}
