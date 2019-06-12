package us.ihmc.robotEnvironmentAwareness.fusion.data;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.PlanarRegionPropagationParameters;

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

      for(int label : uncompressedAdjacentLabels.toArray())
      {
         if(!labels.contains(label) && !adjacentLabels.contains(label))
            adjacentLabels.add(label);
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

   /**
    * Scaled threshold is used according to the v value of the segment center.
    */
   public void updateSparsity(PlanarRegionPropagationParameters propagationParameters)
   {
      double sparseUpperThreshold = propagationParameters.getSparseUpperThreshold();
      double sparseLowerThreshold = propagationParameters.getSparseLowerThreshold();
      for (SegmentationRawData fusionDataSegment : fusionDataSegments)
      {
         double alpha = 1 - fusionDataSegment.getSegmentCenter().getY() / imageHeight;
         double threshold = alpha * (sparseUpperThreshold - sparseLowerThreshold) + sparseLowerThreshold;
         fusionDataSegment.updateSparsity(threshold);
      }
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
