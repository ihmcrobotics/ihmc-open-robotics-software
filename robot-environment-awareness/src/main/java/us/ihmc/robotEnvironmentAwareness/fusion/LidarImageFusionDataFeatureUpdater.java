package us.ihmc.robotEnvironmentAwareness.fusion;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;

public class LidarImageFusionDataFeatureUpdater
{
   private static final double proximityThreshold = 0.1;
   private static final double planarityThreshold = 0.1;

   private final LidarImageFusionRawData data;
   private final int numberOfLabels;
   private final List<SegmentedPlane> segments = new ArrayList<SegmentedPlane>();

   private final Random random = new Random(0612L);

   public LidarImageFusionDataFeatureUpdater(LidarImageFusionRawData lidarImageFusionData)
   {
      data = lidarImageFusionData;
      numberOfLabels = data.numberOfLabels();
   }
   
   public void initialize()
   {
      segments.clear();
   }

   public void iterateSegmenataionPropagation()
   {
      SegmentedPlane segment = new SegmentedPlane();
      int randomSeedLabel = -1;
      while (true)
      {
         randomSeedLabel = random.nextInt(numberOfLabels - 1);
         if (segments[randomSeedLabel].planeID == -1)
            break;
      }
      LogTools.info("randomSeedLabel " + randomSeedLabel);
      
      computePropagation(randomSeedLabel);
   }
   
   /**
    * iterate computation until there is no more candidate to try merge.
    */
   private void computePropagation(int label)
   {
      SegmentedPlane segment = segments[label];
      segment.planeID = iterateSegmentationPropagation;
      
      TDoubleArrayList
   }
   
   private int selectRandomNonSegmentedLabel()
   {
      int randomSeedLabel = -1;
      while (true)
      {
         randomSeedLabel = random.nextInt(numberOfLabels - 1);
         if (data. == -1)
            break;
      }
      LogTools.info("randomSeedLabel " + randomSeedLabel);
      return randomSeedLabel;
   }

   private class SegmentedPlane
   {
      private TIntArrayList labels = new TIntArrayList();
      private final Vector3D normal = new Vector3D();
      private final Point3D center = new Point3D();

      private void merge()
      {

      }

      private void update()
      {

      }

      private int[] getAdjacentLabels()
      {
         return adjacentLabels.toArray();
      }
   }
}
