package us.ihmc.robotEnvironmentAwareness.communication.packets;

import org.apache.commons.math3.util.Precision;

import us.ihmc.communication.packets.Packet;

public class NormalOcTreeNodeMessage extends Packet<NormalOcTreeNodeMessage>
{
   public float size = Float.NaN;
   public int depth = 0;
   public int key0 = -1;
   public int key1 = -1;
   public int key2 = -1;
   public float normalX = Float.NaN;
   public float normalY = Float.NaN;
   public float normalZ = Float.NaN;
   public float normalAverageDeviation = Float.NaN;
   public int normalConsensusSize = 0;
   public float hitLocationX = Float.NaN;
   public float hitLocationY = Float.NaN;
   public float hitLocationZ = Float.NaN;
   public long numberOfHits = 0;

   public NormalOcTreeNodeMessage[] children = null;

   public NormalOcTreeNodeMessage()
   {
   }

   public int getNumberOfChildren()
   {
      if (children == null)
         return 0;

      int numberOfChildren = 0;

      for (NormalOcTreeNodeMessage child : children)
         numberOfChildren += child == null ? 0 : 1;

      return numberOfChildren;
   }

   @Override
   public boolean epsilonEquals(NormalOcTreeNodeMessage other, double epsilon)
   {
      if (!Precision.equals(size, other.size, epsilon))
         return false;
      if (depth != other.depth)
         return false;
      if (key0 != other.key0)
         return false;
      if (key1 != other.key1)
         return false;
      if (key2 != other.key2)
         return false;
      if (!Precision.equals(normalX, other.normalX, epsilon))
         return false;
      if (!Precision.equals(normalY, other.normalY, epsilon))
         return false;
      if (!Precision.equals(normalZ, other.normalZ, epsilon))
         return false;
      if (!Precision.equals(hitLocationX, other.hitLocationX, epsilon))
         return false;
      if (!Precision.equals(hitLocationY, other.hitLocationY, epsilon))
         return false;
      if (!Precision.equals(hitLocationZ, other.hitLocationZ, epsilon))
         return false;
      if (numberOfHits != other.numberOfHits)
         return false;

      if (getNumberOfChildren() != other.getNumberOfChildren())
         return false;

      for (int childIndex = 0; childIndex < 8; childIndex++)
      {
         if (!children[childIndex].epsilonEquals(other.children[childIndex], epsilon))
            return false;
      }

      return true;
   }
}
