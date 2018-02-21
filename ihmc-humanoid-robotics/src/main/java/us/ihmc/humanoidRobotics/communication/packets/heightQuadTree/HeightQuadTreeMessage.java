package us.ihmc.humanoidRobotics.communication.packets.heightQuadTree;

import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.idl.PreallocatedList;

public class HeightQuadTreeMessage extends Packet<HeightQuadTreeMessage>
{
   public float defaultHeight = Float.NaN;
   public float resolution = Float.NaN;
   public float sizeX = Float.NaN;
   public float sizeY = Float.NaN;
   public PreallocatedList<HeightQuadTreeLeafMessage> leaves = new PreallocatedList<>(HeightQuadTreeLeafMessage.class, HeightQuadTreeLeafMessage::new, 1000);

   public HeightQuadTreeMessage()
   {
   }

   @Override
   public void set(HeightQuadTreeMessage other)
   {
      MessageTools.copyData(other.leaves, leaves);
      defaultHeight = other.defaultHeight;
      resolution = other.resolution;
      sizeX = other.sizeX;
      sizeY = other.sizeY;
      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(HeightQuadTreeMessage other, double epsilon)
   {
      if (Float.compare(defaultHeight, other.defaultHeight) != 0)
         return false;
      if (Float.compare(resolution, other.resolution) != 0)
         return false;
      return MessageTools.epsilonEquals(leaves, other.leaves, epsilon);
   }
}
