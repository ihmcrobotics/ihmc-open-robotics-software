package us.ihmc.communication.packets;

import java.util.Arrays;
import java.util.Random;

import us.ihmc.robotics.random.RandomTools;

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

   public BoundingBoxesPacket(Random random)
   {
      int boxesToGenerate = random.nextInt(20);

      this.labels = new String[boxesToGenerate];
      Arrays.fill(labels, Integer.toHexString(random.nextInt()));

      for (int i = 0; i < boxesToGenerate; i++)
      {
         boundingBoxXCoordinates = new int[boxesToGenerate];
         boundingBoxYCoordinates = new int[boxesToGenerate];
         boundingBoxWidths = new int[boxesToGenerate];
         boundingBoxHeights = new int[boxesToGenerate];

         boundingBoxXCoordinates[i] = RandomTools.generateRandomInt(random, -1000, 1000);
         boundingBoxYCoordinates[i] = RandomTools.generateRandomInt(random, -1000, 1000);
         boundingBoxWidths[i] = RandomTools.generateRandomInt(random, 0, 1000);
         boundingBoxHeights[i] = RandomTools.generateRandomInt(random, 0, 1000);
      }
   }

   @Override public boolean epsilonEquals(BoundingBoxesPacket other, double epsilon)
   {
      return Arrays.equals(labels, other.labels) && Arrays.equals(this.boundingBoxHeights, other.boundingBoxHeights) && Arrays.equals(this.boundingBoxWidths, other.boundingBoxWidths) && Arrays
            .equals(this.boundingBoxXCoordinates, other.boundingBoxXCoordinates) && Arrays.equals(this.boundingBoxYCoordinates, other.boundingBoxYCoordinates);
   }
}
