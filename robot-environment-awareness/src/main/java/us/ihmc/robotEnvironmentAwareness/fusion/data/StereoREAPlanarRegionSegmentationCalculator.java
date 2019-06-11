package us.ihmc.robotEnvironmentAwareness.fusion.data;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.PlanarRegionPropagationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;

public class StereoREAPlanarRegionSegmentationCalculator
{
   private PlanarRegionPropagationParameters planarRegionPropagationParameters = new PlanarRegionPropagationParameters();

   private static final int numberOfIterate = 1000;
   private static final int maximumNumberOfTrialsToFindUnIdLabel = 500;

   private final AtomicReference<LidarImageFusionData> data = new AtomicReference<LidarImageFusionData>(null);
   private int numberOfLabels = 0;
   private final List<SegmentationNodeData> segments = new ArrayList<SegmentationNodeData>();
   private List<PlanarRegionSegmentationRawData> regionsNodeData = new ArrayList<>();

   private final Random random = new Random(0612L);

   public void updateFusionData(LidarImageFusionData lidarImageFusionData)
   {
      data.set(lidarImageFusionData);
      numberOfLabels = lidarImageFusionData.getNumberOfImageSegments();
   }

   public void initialize()
   {
      segments.clear();
      regionsNodeData.clear();
   }

   public boolean calculate()
   {
      for (int i = 0; i < numberOfIterate; i++)
      {
         if (!iterateSegmenataionPropagation(i))
         {
            LogTools.info("iterative is terminated " + i);
            break;
         }
      }
      convertNodeDataToPlanarRegionSegmentationRawData();
      return true;
   }

   public List<PlanarRegionSegmentationRawData> getSegmentationRawData()
   {
      return regionsNodeData;
   }

   private void convertNodeDataToPlanarRegionSegmentationRawData()
   {
      for (SegmentationNodeData segmentationNodeData : segments)
      {
         // TODO: id of PlanarRegionSegmentationRawData is color.
         // id of SegmentationNodeData is started from 0.
         // PlanarRegionSegmentationRawData planarRegionSegmentationRawData = new PlanarRegionSegmentationRawData(segmentationNodeData.getId(),
         PlanarRegionSegmentationRawData planarRegionSegmentationRawData = new PlanarRegionSegmentationRawData(random.nextInt(),
                                                                                                               segmentationNodeData.getNormal(),
                                                                                                               segmentationNodeData.getCenter(),
                                                                                                               segmentationNodeData.getPointsInSegment());
         regionsNodeData.add(planarRegionSegmentationRawData);
      }
   }

   private boolean iterateSegmenataionPropagation(int segmentId)
   {
      int nonIDLabel = selectRandomNonIdentifiedLabel();

      if (nonIDLabel == -1)
         return false;
      else
         segments.add(createSegmentNodeData(nonIDLabel, segmentId));

      return true;
   }

   /**
    * iterate computation until there is no more candidate to try merge.
    */
   private SegmentationNodeData createSegmentNodeData(int seedLabel, int segmentId)
   {
      //LogTools.info("createSegmentNodeData " + seedLabel + " " + data.getFusionDataSegment(seedLabel).standardDeviation.getZ());
      SegmentationRawData seedImageSegment = data.get().getFusionDataSegment(seedLabel);
      seedImageSegment.setId(segmentId);
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

            if (candidate.isSparse(planarRegionPropagationParameters.getSparseLowerThreshold(), planarRegionPropagationParameters.getSparseUpperThreshold(),
                                   data.get().getImageHeight()))
            {
               //LogTools.info("is too sparse "+candidate.getImageSegmentLabel());
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
               candidate.setId(segmentId);
               newSegment.merge(candidate);
               isPropagating = true;
            }
         }
      }

      //LogTools.info("allLablesInNewSegment");
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
         SegmentationRawData fusionDataSegment = data.get().getFusionDataSegment(randomSeedLabel);
         if (fusionDataSegment.getId() == -1
               && !fusionDataSegment.isSparse(planarRegionPropagationParameters.getSparseLowerThreshold(),
                                                                             planarRegionPropagationParameters.getSparseUpperThreshold(),
                                                                             data.get().getImageHeight()))
            return randomSeedLabel;
      }
      return -1;
   }
}
