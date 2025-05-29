package us.ihmc.behaviors.tools.perception;

import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TIntArrayList;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.commons.MathTools;
import us.ihmc.perception.depthData.PointCloudData;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.sensorProcessing.heightMap.HeightMapManager;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * This height map updater was created to satisfy the needs of
 * look and step but was experimental and probably no longer works.
 *
 * @deprecated Needs work
 */
public class AlternateHeightMapUpdater
{
   private final HeightMapParameters parameters;
   private final HeightMapManager heightMap;
   private final TIntArrayList holeKeyList = new TIntArrayList();
   private final TFloatArrayList holeHeights = new TFloatArrayList();
   private final AtomicInteger totalUpdateCount = new AtomicInteger();

   public AlternateHeightMapUpdater()
   {
      parameters = new HeightMapParameters();
      heightMap = new HeightMapManager(parameters, parameters.getGridResolutionXY(), parameters.getGridSizeXY());
   }

   public void clear()
   {
      heightMap.clear();
   }

   public HeightMapMessage update(Point3D[] scanPoints, ReferenceFrame ousterFrame, Point3D center)
   {
      heightMap.setMaxHeight(center.getZ() + 2.0); // Set max to basically just over the robot's head.
      heightMap.translateToNewGridCenter(center.getX(), center.getY(), parameters.getVarianceAddedWhenTranslating());

      PointCloudData pointCloud = new PointCloudData(System.nanoTime(), scanPoints, null);

      // Transform ouster data
      for (int i = 0; i < pointCloud.getPointCloud().length; i++)
      {
         FramePoint3D point = new FramePoint3D(ousterFrame, pointCloud.getPointCloud()[i]);
         point.changeFrame(ReferenceFrame.getWorldFrame());
         pointCloud.getPointCloud()[i].set(point);
      }

      // Update height map
      heightMap.update(pointCloud.getPointCloud());
      totalUpdateCount.incrementAndGet();

      /* estimate ground height */
      double estimatedGroundHeight = estimateGroundHeight(pointCloud.getPointCloud());

      /* filter near and below ground height and outliers that seem too high */
      performFiltering(estimatedGroundHeight);

      /* pack ROS message */
      HeightMapMessage message = buildMessage();
      message.setEstimatedGroundHeight(estimatedGroundHeight);
      return message;
   }

   private HeightMapMessage buildMessage()
   {
      // Copy and report over messager
      HeightMapMessage message = new HeightMapMessage();
      message.setGridSizeXy(parameters.getGridSizeXY());
      message.setXyResolution(parameters.getGridResolutionXY());
      message.setGridCenterX(heightMap.getGridCenterXY().getX());
      message.setGridCenterY(heightMap.getGridCenterXY().getY());

      for (int i = 0; i < heightMap.getNumberOfCells(); i++)
      {
         if (heightMap.cellHasUnfilteredData(i))
         {
            int key = heightMap.getKey(i);
            message.getKeys().add(key);
            message.getHeights().add((float) heightMap.getHeightAt(i));
         }
      }

      message.getKeys().addAll(holeKeyList);
      message.getHeights().addAll(holeHeights);

      return message;
   }

   private double estimateGroundHeight(Point3D[] pointCloud)
   {
      double[] sortedHeights = Arrays.stream(pointCloud).mapToDouble(Point3D::getZ).sorted().toArray();
      double minPercentile = 0.02;
      double maxPercentile = 0.06;
      int minIndex = (int) (minPercentile * sortedHeights.length);
      int maxIndex = (int) (maxPercentile * sortedHeights.length);

      double estimatedGroundHeight = 0.0;
      for (int i = minIndex; i < maxIndex; i++)
      {
         estimatedGroundHeight += sortedHeights[i];
      }

      return estimatedGroundHeight / (maxIndex - minIndex);
   }

   private static final int[] xOffsetEightConnectedGrid = new int[]{-1, -1, 0, 1, 1,  1, 0,  -1};
   private static final int[] yOffsetEightConnectedGrid = new int[]{0,  1,  1, 1, 0, -1, -1 , -1};

   private void performFiltering(double estimatedGroundHeight)
   {
      /* Remove cells near ground height */
      double groundEpsilonToFilter = 0.03;
      double groundHeightThreshold = groundEpsilonToFilter + estimatedGroundHeight;

      for (int i = heightMap.getNumberOfCells() - 1; i >= 0; i--)
      {
         heightMap.setGroundCell(i, heightMap.getHeightAt(i) < groundHeightThreshold);
      }

      /* Remove cells with no neighbors and reset height of cells which are higher than all neighbors */
      int minNumberOfNeighbors = 2;
      int minNumberOfNeighborsToResetHeight = 3;
      double epsilonHeightReset = 0.04;

      outerLoop:
      for (int i = heightMap.getNumberOfCells() - 1; i >= 0; i--)
      {
         int xIndex = heightMap.getXIndex(i);
         int yIndex = heightMap.getYIndex(i);

         if (xIndex == 0 || yIndex == 0 || xIndex == heightMap.getCellsPerAxis() - 1 || yIndex == heightMap.getCellsPerAxis() - 1)
         {
            continue;
         }

         boolean muchHigherThanAllNeighbors = true;
         double heightThreshold = heightMap.getHeightAt(i) - epsilonHeightReset;
         int numberOfNeighbors = 0;
         for (int j = 0; j < xOffsetEightConnectedGrid.length; j++)
         {
            int xNeighbor = xIndex + xOffsetEightConnectedGrid[j];
            int yNeighbor = yIndex + yOffsetEightConnectedGrid[j];
            if (heightMap.cellHasData(xNeighbor, yNeighbor))
            {
               numberOfNeighbors++;
               muchHigherThanAllNeighbors = muchHigherThanAllNeighbors && heightMap.getHeightAt(xNeighbor, yNeighbor) < heightThreshold;
            }

            if (numberOfNeighbors >= minNumberOfNeighbors && !muchHigherThanAllNeighbors)
            {
               heightMap.setHasSufficientNeighbors(i, true);
               continue outerLoop;
            }
         }

         if (numberOfNeighbors < minNumberOfNeighbors)
         {
            heightMap.setHasSufficientNeighbors(i, false);
         }
         else if (numberOfNeighbors >= minNumberOfNeighborsToResetHeight && muchHigherThanAllNeighbors)
         {
            double resetHeight = 0.0;
            for (int j = 0; j < xOffsetEightConnectedGrid.length; j++)
            {
               int xNeighbor = xIndex + xOffsetEightConnectedGrid[j];
               int yNeighbor = yIndex + yOffsetEightConnectedGrid[j];
               if (heightMap.cellHasData(xNeighbor, yNeighbor))
               {
                  resetHeight += heightMap.getHeightAt(xNeighbor, yNeighbor);
               }
            }
            heightMap.resetAtHeight(i, resetHeight / numberOfNeighbors, MathTools.square(parameters.getNominalStandardDeviation()));
         }
      }

      /* Once the map has had a chance to initialize, fill in any holes. This is a very ad-hoc way to fill them in,
      *  holes are detected as cells without data that does have data within both sides along either the x or y axis.
      */
      int holeProximityThreshold = 3;

      if (totalUpdateCount.get() > 50)
      {
         holeKeyList.clear();
         holeHeights.clear();

         /* For any cell without data, populate with average of neighbors */
         for (int xIndex = holeProximityThreshold; xIndex < heightMap.getCellsPerAxis() - holeProximityThreshold; xIndex++)
         {
            for (int yIndex = holeProximityThreshold; yIndex < heightMap.getCellsPerAxis() - holeProximityThreshold; yIndex++)
            {
               if (heightMap.cellHasData(xIndex, yIndex))
                  continue;

               double heightSearch;

               if (!Double.isNaN(heightSearch = hasDataInDirection(xIndex, yIndex, true, holeProximityThreshold)))
               {
                  holeKeyList.add(HeightMapTools.indicesToKey(xIndex, yIndex, heightMap.getCenterIndex()));
                  holeHeights.add((float) heightSearch);
               }
               else if (!Double.isNaN(heightSearch = hasDataInDirection(xIndex, yIndex, false, holeProximityThreshold)))
               {
                  holeKeyList.add(HeightMapTools.indicesToKey(xIndex, yIndex, heightMap.getCenterIndex()));
                  holeHeights.add((float) heightSearch);
               }
            }
         }
      }
   }

   /**
    * Searches along x or y axis for neighboring data. If data is found, returns average, otherwise returns Double.NaN
    */
   private double hasDataInDirection(int xIndex, int yIndex, boolean searchX, int maxDistanceToCheck)
   {
      double posHeight = Double.NaN;

      for (int i = 1; i <= maxDistanceToCheck; i++)
      {
         int xQuery = xIndex + (searchX ? i : 0);
         int yQuery = yIndex + (searchX ? 0 : i);
         if (heightMap.cellHasUnfilteredData(xQuery, yQuery))
         {
            posHeight = heightMap.getHeightAt(xQuery, yQuery);
            break;
         }
      }

      if (Double.isNaN(posHeight))
      {
         return Double.NaN;
      }

      double negHeight = Double.NaN;

      for (int i = 1; i <= maxDistanceToCheck; i++)
      {
         int xQuery = xIndex - (searchX ? i : 0);
         int yQuery = yIndex - (searchX ? 0 : i);
         if (heightMap.cellHasUnfilteredData(xQuery, yQuery))
         {
            negHeight = heightMap.getHeightAt(xQuery, yQuery);
            break;
         }
      }

      if (Double.isNaN(negHeight) || (Math.abs(posHeight - negHeight) > 0.2))
      {
         return Double.NaN;
      }

      double averageHeight = 0.5 * (posHeight + negHeight);
      if (averageHeight < 0.07)
      {
         return Double.NaN;
      }
      else
      {
         return averageHeight;
      }
   }
}
