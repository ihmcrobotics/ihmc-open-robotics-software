package us.ihmc.communication.packets;

import java.util.Arrays;

/**
 *
 */
public class BoundingBoxesPacket extends Packet<BoundingBoxesPacket>
{
   public int[] boundingBoxXCoordinates, boundingBoxYCoordinates, boundingBoxWidths, boundingBoxHeights;
   public String[] labels;

   public BoundingBoxesPacket()
   {

   }

   public BoundingBoxesPacket(int[] packedBoxes, String[] labels)
   {
      this.labels = labels;
      int n = packedBoxes.length / 4;
      boundingBoxXCoordinates = new int[n];
      boundingBoxYCoordinates = new int[n];
      boundingBoxWidths = new int[n];
      boundingBoxHeights = new int[n];
      for (int i = 0; i < n; i++)
      {
         boundingBoxXCoordinates[i] = packedBoxes[i * 4];
         boundingBoxYCoordinates[i] = packedBoxes[i * 4 + 1];
         boundingBoxWidths[i] = packedBoxes[i * 4 + 2];
         boundingBoxHeights[i] = packedBoxes[i * 4 + 3];
      }
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
