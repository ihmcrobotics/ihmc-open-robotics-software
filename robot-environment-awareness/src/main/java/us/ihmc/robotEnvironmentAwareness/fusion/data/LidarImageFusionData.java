package us.ihmc.robotEnvironmentAwareness.fusion.data;

import java.util.ArrayList;

import gnu.trove.list.array.TIntArrayList;

/**
 * This data set is to hold a list of SegmentationRawData.
 */
public class LidarImageFusionData
{
   private final ArrayList<SegmentationRawData> fusionDataSegments = new ArrayList<SegmentationRawData>();

   public LidarImageFusionData(ArrayList<SegmentationRawData> fusionDataSegments)
   {
      this.fusionDataSegments.addAll(fusionDataSegments);
   }

   public int getNumberOfLabels()
   {
      return fusionDataSegments.size();
   }

   public SegmentationRawData getFusionDataSegment(int label)
   {
      return fusionDataSegments.get(label);
   }

   public int[] getAdjacentLabels(TIntArrayList labels)
   {
      TIntArrayList uncompressedAdjacentLabels = new TIntArrayList();
      TIntArrayList adjacentLabels = new TIntArrayList();

      for (int label : labels.toArray())
      {
         uncompressedAdjacentLabels.addAll(fusionDataSegments.get(label).getAdjacentSegmentLabels());
      }

      for (int i = 0; i < fusionDataSegments.size(); i++)
      {
         if (uncompressedAdjacentLabels.contains(i) && !labels.contains(i))
         {
            adjacentLabels.add(i);
         }
      }

      return adjacentLabels.toArray();
   }

   public boolean allIdentified()
   {
      for (SegmentationRawData fusionDataSegment : fusionDataSegments)
      {
         if (fusionDataSegment.getId() == -1)
            return false;
      }
      return true;
   }
}
