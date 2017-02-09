package us.ihmc.humanoidRobotics.communication.packets.heightQuadTree;

import us.ihmc.communication.packets.StatusPacket;

public class HeightQuadTreeNodeMessage extends StatusPacket<HeightQuadTreeNodeMessage>
{
   public float height = Float.NaN;

   public HeightQuadTreeNodeMessage[] children;

   public HeightQuadTreeNodeMessage()
   {
   }

   public HeightQuadTreeNodeMessage(HeightQuadTreeNodeMessage other)
   {
      set(other);
   }

   @Override
   public void set(HeightQuadTreeNodeMessage other)
   {
      height = other.height;

      if (other.children != null)
      {
         children = new HeightQuadTreeNodeMessage[4];
         for (int childIndex = 0; childIndex < 4; childIndex++)
         {
            if (other.children[childIndex] == null)
               continue;

            children[childIndex] = new HeightQuadTreeNodeMessage();
            children[childIndex].set(other.children[childIndex]);
         }
      }
   }

   public int getNumberOfChildren()
   {
      if (children == null)
         return 0;
      int numberOfChildren = 0;

      for (int childIndex = 0; childIndex < 4; childIndex++)
         numberOfChildren += children[childIndex] == null ? 0 : 1;

      return numberOfChildren;
   }

   @Override
   public boolean epsilonEquals(HeightQuadTreeNodeMessage other, double epsilon)
   {
      // Float.compare for NaNs
      if (Float.compare(height, other.height) != 0)
         return false;
      if (getNumberOfChildren() != other.getNumberOfChildren())
         return false;

      for (int childIndex = 0; childIndex < 4; childIndex++)
      {
         HeightQuadTreeNodeMessage thisChild = children[childIndex];
         HeightQuadTreeNodeMessage otherChild = other.children[childIndex];
         if (thisChild == null && otherChild != null)
            return false;
         if (thisChild != null && otherChild == null)
            return false;
         if (!thisChild.epsilonEquals(otherChild, epsilon))
            return false;
      }

      return true;
   }
}
