package us.ihmc.robotEnvironmentAwareness.fusion;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.fusion.data.LidarImageFusionData;
import us.ihmc.robotEnvironmentAwareness.fusion.data.SegmentationNodeData;
import us.ihmc.robotEnvironmentAwareness.fusion.data.SegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.PlanarRegionPropagationParameters;

public class StereoREAPlanarRegionFeatureUpdater
{
   private PlanarRegionPropagationParameters planarRegionPropagationParameters = new PlanarRegionPropagationParameters();

   private static final int maximumNumberOfTrialsToFindUnIdLabel = 100;

   private final AtomicReference<LidarImageFusionData> data = new AtomicReference<LidarImageFusionData>(null);
   private int numberOfLabels = 0;
   private final List<SegmentationNodeData> segments = new ArrayList<SegmentationNodeData>();

   private final Random random = new Random(0612L);

   public StereoREAPlanarRegionFeatureUpdater()
   {
      
   }
   
   public void update(LidarImageFusionData lidarImageFusionData)
   {
      data.set(lidarImageFusionData);
      numberOfLabels = lidarImageFusionData.getNumberOfLabels();
   }

   public int getNumberOfSegments()
   {
      return segments.size();
   }

   public SegmentationNodeData getSegmentationNodeData(int index)
   {
      return segments.get(index);
   }

   public void initialize()
   {
      segments.clear();
   }

   public boolean iterateSegmenataionPropagation(int segmentId)
   {
      int nonIDLabel = selectRandomNonIdentifiedLabel();
      LogTools.info("" + segmentId + " randomSeedLabel " + nonIDLabel);

      if (nonIDLabel == -1)
         return false;
      else
         segments.add(createSegmentNodeData(nonIDLabel, segmentId));

      return true;
   }

   public void addSegmentNodeData(int seedLabel, int segmentId)
   {
      segments.add(createSegmentNodeData(seedLabel, segmentId));
   }

   /**
    * iterate computation until there is no more candidate to try merge.
    */
   public SegmentationNodeData createSegmentNodeData(int seedLabel, int segmentId)
   {
      //LogTools.info("createSegmentNodeData " + seedLabel + " " + data.getFusionDataSegment(seedLabel).standardDeviation.getZ());
      SegmentationRawData seedImageSegment = data.get().getFusionDataSegment(seedLabel);
      seedImageSegment.setID(segmentId);
      SegmentationNodeData newSegment = new SegmentationNodeData(seedImageSegment);

      boolean isPropagating = true;

      while (isPropagating)
      {
         isPropagating = false;

         int[] adjacentLabels = data.get().getAdjacentLabels(newSegment.getLabels());
         //LogTools.info("propagating " + adjacentLabels.length);
         for (int adjacentLabel : adjacentLabels)
         {
            //LogTools.info("   candidate label is " + adjacentLabels[i]);
            SegmentationRawData candidate = data.get().getFusionDataSegment(adjacentLabel);

            if (candidate.isSparse(planarRegionPropagationParameters.getSparseThreshold()))
            {
               //LogTools.info("is too sparce "+candidate.getImageSegmentLabel());
               continue;
            }

            boolean isParallel = false;
            boolean isCoplanar = false;
            if (newSegment.isParallel(candidate, planarRegionPropagationParameters.getPlanarityThreshold()))
               isParallel = true;
            if (newSegment.isCoplanar(candidate, planarRegionPropagationParameters.getProximityThreshold()))
               isCoplanar = true;

            //LogTools.info("connectivity test result is ## " + (isParallel && isCoplanar) + " ## isParallel " + isParallel + " isCoplanar " + isCoplanar);
            if (isParallel && isCoplanar)
            {
               candidate.setID(segmentId);
               newSegment.merge(candidate);
               isPropagating = true;
            }
         }
      }

      LogTools.info("allLablesInNewSegment");
      TIntArrayList allLablesInNewSegment = newSegment.getLabels();
      for (int labelNumber : allLablesInNewSegment.toArray())
      {
         //LogTools.info("" + labelNumber);
      }

      int[] adjacentLabels = data.get().getAdjacentLabels(newSegment.getLabels());
      //LogTools.info("extending for " + adjacentLabels.length + " segments");
      for (int adjacentLabel : adjacentLabels)
      {
         SegmentationRawData adjacentData = data.get().getFusionDataSegment(adjacentLabel);
         newSegment.extend(adjacentData, planarRegionPropagationParameters.getExtendingDistanceThreshold(),
                           planarRegionPropagationParameters.isUpdateExtendedData(), planarRegionPropagationParameters.getExtendingRadiusThreshold());
      }
      return newSegment;
   }

   private int selectRandomNonIdentifiedLabel()
   {
      int randomSeedLabel = -1;
      for (int i = 0; i < maximumNumberOfTrialsToFindUnIdLabel; i++)
      {
         randomSeedLabel = random.nextInt(numberOfLabels - 1);
         if (data.get().getFusionDataSegment(randomSeedLabel).getId() == -1
               && !data.get().getFusionDataSegment(randomSeedLabel).isSparse(planarRegionPropagationParameters.getSparseThreshold()))
            return randomSeedLabel;
      }
      return -1;
   }
   
}
