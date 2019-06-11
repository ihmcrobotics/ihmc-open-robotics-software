package us.ihmc.robotEnvironmentAwareness.fusion.data;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TIntArrayList;

/**
 * This data set is to hold a list of SegmentationRawData.
 */
public class LidarImageFusionData
{
   private final int imageWidth;
   private final int imageHeight;
   private final ArrayList<SegmentationRawData> fusionDataSegments = new ArrayList<SegmentationRawData>();

   public LidarImageFusionData(List<SegmentationRawData> fusionDataSegments, int imageWidth, int imageHeight)
   {
      this.fusionDataSegments.addAll(fusionDataSegments);
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
   }

   public int getNumberOfImageSegments()
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
   
   public int getImageWidth()
   {
      return imageWidth;
   }
   
   public int getImageHeight()
   {
      return imageHeight;
   }
}
