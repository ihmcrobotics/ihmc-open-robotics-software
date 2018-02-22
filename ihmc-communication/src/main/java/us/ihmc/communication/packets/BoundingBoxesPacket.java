package us.ihmc.communication.packets;

import java.util.Arrays;

/**
 *
 */
public class BoundingBoxesPacket extends Packet<BoundingBoxesPacket>
{
   public int[] boundingBoxXCoordinates, boundingBoxYCoordinates, boundingBoxWidths, boundingBoxHeights;
   public StringBuilder[] labels;

   public BoundingBoxesPacket()
   {

   }

   public BoundingBoxesPacket(BoundingBoxesPacket other)
   {
      this.labels = other.labels;
      this.boundingBoxXCoordinates = other.boundingBoxXCoordinates;
      this.boundingBoxYCoordinates = other.boundingBoxYCoordinates;
      this.boundingBoxWidths = other.boundingBoxWidths;
      this.boundingBoxHeights = other.boundingBoxHeights;
   }

   @Override
   public void set(BoundingBoxesPacket other)
   {
      setPacketInformation(other);
      this.labels = other.labels;
      this.boundingBoxXCoordinates = other.boundingBoxXCoordinates;
      this.boundingBoxYCoordinates = other.boundingBoxYCoordinates;
      this.boundingBoxWidths = other.boundingBoxWidths;
      this.boundingBoxHeights = other.boundingBoxHeights;
   }

   @Override
   public boolean epsilonEquals(BoundingBoxesPacket other, double epsilon)
   {
      return Arrays.equals(labels, other.labels) && Arrays.equals(this.boundingBoxHeights, other.boundingBoxHeights)
            && Arrays.equals(this.boundingBoxWidths, other.boundingBoxWidths) && Arrays.equals(this.boundingBoxXCoordinates, other.boundingBoxXCoordinates)
            && Arrays.equals(this.boundingBoxYCoordinates, other.boundingBoxYCoordinates);
   }
}
