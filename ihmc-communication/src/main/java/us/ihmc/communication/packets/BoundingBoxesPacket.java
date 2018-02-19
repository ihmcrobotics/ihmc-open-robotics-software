package us.ihmc.communication.packets;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.idl.PreallocatedList;

/**
 *
 */
public class BoundingBoxesPacket extends Packet<BoundingBoxesPacket>
{
   public TIntArrayList boundingBoxXCoordinates = new TIntArrayList();
   public TIntArrayList boundingBoxYCoordinates = new TIntArrayList();
   public TIntArrayList boundingBoxWidths = new TIntArrayList();
   public TIntArrayList boundingBoxHeights = new TIntArrayList();
   public PreallocatedList<StringBuilder> labels = new PreallocatedList<>(StringBuilder.class, StringBuilder::new, 10);

   public BoundingBoxesPacket()
   {

   }

   public BoundingBoxesPacket(BoundingBoxesPacket other)
   {
      MessageTools.copyData(other.labels, labels);
      MessageTools.copyData(other.boundingBoxXCoordinates, boundingBoxXCoordinates);
      MessageTools.copyData(other.boundingBoxYCoordinates, boundingBoxYCoordinates);
      MessageTools.copyData(other.boundingBoxWidths, boundingBoxWidths);
      MessageTools.copyData(other.boundingBoxHeights, boundingBoxHeights);
   }

   @Override
   public void set(BoundingBoxesPacket other)
   {
      setPacketInformation(other);
      MessageTools.copyData(other.labels, labels);
      MessageTools.copyData(other.boundingBoxXCoordinates, boundingBoxXCoordinates);
      MessageTools.copyData(other.boundingBoxYCoordinates, boundingBoxYCoordinates);
      MessageTools.copyData(other.boundingBoxWidths, boundingBoxWidths);
      MessageTools.copyData(other.boundingBoxHeights, boundingBoxHeights);
   }

   @Override
   public boolean epsilonEquals(BoundingBoxesPacket other, double epsilon)
   {
      return labels.equals(other.labels) && this.boundingBoxHeights.equals(other.boundingBoxHeights)
            && this.boundingBoxWidths.equals(other.boundingBoxWidths) && this.boundingBoxXCoordinates.equals(other.boundingBoxXCoordinates)
            && this.boundingBoxYCoordinates.equals(other.boundingBoxYCoordinates);
   }
}
