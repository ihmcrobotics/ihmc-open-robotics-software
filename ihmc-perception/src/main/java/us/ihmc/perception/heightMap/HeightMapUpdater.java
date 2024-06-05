package us.ihmc.perception.heightMap;

import com.google.common.util.concurrent.AtomicDouble;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.HeightMapMessagePubSubType;
import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.log.LogTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;
import us.ihmc.sensorProcessing.heightMap.*;
import us.ihmc.tools.property.StoredPropertySet;

import java.io.*;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

public class HeightMapUpdater
{
   private static final boolean snapCenterToGrid = true;

   private static final boolean printFrequency = false;
   private static final boolean printQueueSize = false;
   private static final int maxQueueLength = 5;

   private static final long sleepTimeMillis = 20;
   private static final long maxIdleTimeMillis = 1000;

   public static final boolean USE_OUSTER_FRAME = true;
   public static final RigidBodyTransform APPROX_OUSTER_TRANSFORM = new RigidBodyTransform();
   static
   {
//      double ousterPitch = Math.toRadians(21.5);
//      APPROX_OUSTER_TRANSFORM.getRotation().setToPitchOrientation(ousterPitch);

      double ousterPitch = Math.toRadians(30.0);
      APPROX_OUSTER_TRANSFORM.getRotation().setToPitchOrientation(ousterPitch);
      APPROX_OUSTER_TRANSFORM.getTranslation().set(-0.2, 0.0, 1.0);
   }

   private final HeightMapParameters parameters;
   private final HeightMapFilterParameters filterParameters;

   private final HeightMapManager heightMap;
   private final List<Consumer<HeightMapMessage>> heightMapConsumers = new ArrayList<>();
   private Consumer<Point2DReadOnly> gridCenterConsumer;
   private final AtomicReference<HeightMapMessage> latestMessage = new AtomicReference<>();

   private final AtomicBoolean enableUpdates = new AtomicBoolean();
   private final AtomicBoolean clearRequested = new AtomicBoolean();
   private final AtomicDouble gridCenterX = new AtomicDouble();
   private final AtomicDouble gridCenterY = new AtomicDouble();
   private final AtomicDouble maxHeight = new AtomicDouble(0.4);

   private final AtomicBoolean isPaused = new AtomicBoolean(false);

   private final TIntArrayList holeKeyList = new TIntArrayList();
   private final TFloatArrayList holeHeights = new TFloatArrayList();

   private final ConcurrentLinkedQueue<HeightMapInputData> pointCloudQueue = new ConcurrentLinkedQueue<>();

   private int publishFrequencyCounter = 0;
   private final AtomicInteger publishFrequency = new AtomicInteger();
   private final AtomicInteger totalUpdateCount = new AtomicInteger();

   public HeightMapUpdater()
   {
      parameters = new HeightMapParameters();
      filterParameters = new HeightMapFilterParameters();
      heightMap = new HeightMapManager(parameters, parameters.getGridResolutionXY(), parameters.getGridSizeXY());
   }

   public HeightMapParametersBasics getHeightMapParameters()
   {
      return parameters;
   }

   public StoredPropertySet getHeightMapFilterParameters()
   {
      return filterParameters;
   }

   public void attachHeightMapConsumer(Consumer<HeightMapMessage> heightMapConsumer)
   {
      heightMapConsumers.add(heightMapConsumer);
   }

   public void setEnableUpdates(boolean enableUpdates)
   {
      this.enableUpdates.set(enableUpdates);
   }

   public boolean updatesAreEnabled()
   {
      return enableUpdates.get();
   }

   public void requestClear()
   {
      this.clearRequested.set(true);
   }

   public void setGridCenterX(double gridCenterX)
   {
      this.gridCenterX.set(gridCenterX);
   }

   public void setGridCenterY(double gridCenterY)
   {
      this.gridCenterY.set(gridCenterY);
   }

   public void setMaxHeight(double maxHeight)
   {
      this.maxHeight.set(maxHeight);
   }

   public void setPublishFrequency(int publishFrequency)
   {
      this.publishFrequency.set(publishFrequency);
   }

   public void setParameters(HeightMapParameters parameters)
   {
      this.parameters.set(parameters);
   }

   public void setGridCenterConsumer(Consumer<Point2DReadOnly> gridCenterConsumer)
   {
      this.gridCenterConsumer = gridCenterConsumer;
   }

   public void requestPause()
   {
      isPaused.set(true);
   }

   public void requestResume()
   {
      isPaused.set(false);
   }

   public void exportOnThread()
   {
      ThreadTools.startAThread(this::export, "Height map exporter");
   }

   public void addPointCloudToQueue(HeightMapInputData inputData)
   {
      if (!isPaused.get())
         this.pointCloudQueue.add(inputData);
   }

   public boolean runUpdateThread()
   {
      int updatesWithoutDataCounter = 0;


      while (updatesWithoutDataCounter < maxIdleTimeMillis / sleepTimeMillis)
      {
         HeightMapInputData data = pointCloudQueue.poll();
         if (data == null)
         {
            updatesWithoutDataCounter++;
         }
         else
         {
            updatesWithoutDataCounter = 0;
            update(data);
         }

         if (pointCloudQueue.isEmpty())
         {
            ThreadTools.sleep(sleepTimeMillis);
         }
      }

      return true;
   }

   public void runFullUpdate(long maxUpdatePeriod)
   {
      long estimatedUpdatePeriod = 0;
      long cumulativeUpdateDuration = 0;
      int totalUpdates = 0;
      long startMillis = System.currentTimeMillis();

      while (cumulativeUpdateDuration + estimatedUpdatePeriod < maxUpdatePeriod && !pointCloudQueue.isEmpty())
      {
         HeightMapInputData data = pointCloudQueue.poll();
         if (data == null)
         {
            break;
         }
         else
         {
            update(data);
         }

         cumulativeUpdateDuration = System.currentTimeMillis() - startMillis;
         totalUpdates++;
         estimatedUpdatePeriod = cumulativeUpdateDuration / totalUpdates;
      }
   }

   private void update(HeightMapInputData pointCloudData)
   {
      update(pointCloudData.pointCloud.getPointCloud(), pointCloudData.sensorPose, pointCloudData.gridCenter, pointCloudData.verticalMeasurementVariance);
   }

   private void update(Point3D[] pointCloud, FramePose3DReadOnly pointCloudFramePose, Point3DReadOnly gridCenter, double verticalVarianceMeasurement)
   {
      if (printFrequency)
      {
         updateFrequency();
      }

      RigidBodyTransformReadOnly ousterTransform = USE_OUSTER_FRAME ? pointCloudFramePose : APPROX_OUSTER_TRANSFORM;
      boolean isNonZeroTransform = ousterTransform.hasTranslation() && ousterTransform.hasRotation();

      // Transform ouster data
      if (isNonZeroTransform)
      {
         for (int i = 0; i < pointCloud.length; i++)
         {
            pointCloud[i].applyTransform(ousterTransform);
         }
      }

      Point2D snappedGridCenter = new Point2D(gridCenter);
      if (snapCenterToGrid)
         snapPointToGrid(snappedGridCenter, parameters.getGridResolutionXY());

      if (clearRequested.getAndSet(false))
      {
         heightMap.resetAtGridCenter(gridCenterX.get(), gridCenterY.get());
      }
      else
      {
         gridCenterX.set(snappedGridCenter.getX());
         gridCenterY.set(snappedGridCenter.getY());
         heightMap.translateToNewGridCenter(snappedGridCenter.getX(), snappedGridCenter.getY(), parameters.getVarianceAddedWhenTranslating());
         if (gridCenterConsumer != null)
            gridCenterConsumer.accept(new Point2D(snappedGridCenter));
      }

      // Update height map
      heightMap.setMaxHeight(gridCenter.getZ() + parameters.getMaxZ());
      heightMap.updateGridSizeXY(parameters.getGridSizeXY());
      heightMap.updateGridResolutionXY(parameters.getGridResolutionXY());
      heightMap.update(pointCloud, verticalVarianceMeasurement);
      totalUpdateCount.incrementAndGet();

      if (--publishFrequencyCounter <= 0)
      {
         /* estimate ground height */
         double estimatedGroundHeight = Double.NaN;
         if (filterParameters.getEstimateGroundHeight())
            estimatedGroundHeight = estimateGroundHeight(pointCloud);

         /* filter near and below ground height and outliers that seem too high */
         performFiltering(estimatedGroundHeight);

         /* pack ROS message */
         HeightMapMessage message = buildMessage();
         message.setEstimatedGroundHeight(estimatedGroundHeight);

         for (Consumer<HeightMapMessage> heightMapConsumer : heightMapConsumers)
         {
            heightMapConsumer.accept(message);
         }

         publishFrequencyCounter = publishFrequency.get();
         latestMessage.set(message);
      }

      if (printQueueSize)
         LogTools.info("Point cloud queue: " + pointCloudQueue.size());
   }

   public HeightMapData getLatestHeightMap()
   {
      return HeightMapMessageTools.unpackMessage(buildMessage());
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
      if (filterParameters.getEstimateGroundHeight())
      {
         /* Remove cells near ground height */
         double groundEpsilonToFilter = 0.03;
         double groundHeightThreshold = groundEpsilonToFilter + estimatedGroundHeight;

         for (int i = heightMap.getNumberOfCells() - 1; i >= 0; i--)
         {
            heightMap.setGroundCell(i, heightMap.getHeightAt(i) < groundHeightThreshold);
         }
      }
      else
      {
         for (int i = heightMap.getNumberOfCells() - 1; i >= 0; i--)
         {
            heightMap.setGroundCell(i, false);
         }
      }

      /* Remove cells with no neighbors and reset height of cells which are higher than all neighbors */
      for (int cellNumber = heightMap.getNumberOfCells() - 1; cellNumber >= 0; cellNumber--)
      {
         updateIfCellIsOutlier(cellNumber);
      }

      if (filterParameters.getFillHoles())
      {
         /* Once the map has had a chance to initialize, fill in any holes. This is a very ad-hoc way to fill them in,
          *  holes are detected as cells without data that does have data within both sides along either the x or y axis.
          */
         int holeProximityThreshold = filterParameters.getHoleProximityThreshold();

         if (totalUpdateCount.get() > 50)
         {
            holeKeyList.clear();
            holeHeights.clear();

            /* For any cell without data, populate with average of neighbors */
            for (int xIndex = holeProximityThreshold; xIndex < heightMap.getCellsPerAxis() - holeProximityThreshold; xIndex++)
            {
               for (int yIndex = holeProximityThreshold; yIndex < heightMap.getCellsPerAxis() - holeProximityThreshold; yIndex++)
               {
                  float height = getHeightOfHole(xIndex, yIndex);
                  if (!Float.isNaN(height))
                  {
                     holeKeyList.add(HeightMapTools.indicesToKey(xIndex, yIndex, heightMap.getCenterIndex()));
                     holeHeights.add(height);
                  }
               }
            }
         }
      }
   }

   private void updateIfCellIsOutlier(int cellNumber)
   {
      if (!filterParameters.getRemoveOutlierCells())
      {
         heightMap.setHasSufficientNeighbors(cellNumber, true);
         return;
      }

      int xIndex = heightMap.getXIndex(cellNumber);
      int yIndex = heightMap.getYIndex(cellNumber);

      if (xIndex == 0 || yIndex == 0 || xIndex == heightMap.getCellsPerAxis() - 1 || yIndex == heightMap.getCellsPerAxis() - 1)
      {
         return;
      }

      double heightThreshold = heightMap.getHeightAt(cellNumber) - filterParameters.getOutlierCellHeightResetEpsilon();
      int numberOfNeighbors = 0;
      int neighborsAtSameHeight = 0;
      for (int j = 0; j < xOffsetEightConnectedGrid.length; j++)
      {
         int xNeighbor = xIndex + xOffsetEightConnectedGrid[j];
         int yNeighbor = yIndex + yOffsetEightConnectedGrid[j];
         if (heightMap.cellHasData(xNeighbor, yNeighbor))
         {
            numberOfNeighbors++;
            if (heightMap.getHeightAt(xNeighbor, yNeighbor) > heightThreshold)
               neighborsAtSameHeight++;
         }

         if (neighborsAtSameHeight >= filterParameters.getMinNeighborsAtSameHeightForValid())
         {
            heightMap.setHasSufficientNeighbors(cellNumber, true);
            return;
         }
      }

      if (numberOfNeighbors < filterParameters.getMinNeighborsAtSameHeightForValid())
      {
         heightMap.setHasSufficientNeighbors(cellNumber, false);
      }
      else if (numberOfNeighbors >= filterParameters.getMinNeighborsToDetermineOutliers() && neighborsAtSameHeight < filterParameters.getMinNeighborsAtSameHeightForValid())
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
         heightMap.resetAtHeight(cellNumber, resetHeight / numberOfNeighbors, MathTools.square(parameters.getNominalStandardDeviation()));
      }
   }

   private float getHeightOfHole(int xIndex, int yIndex)
   {
      if (heightMap.cellHasData(xIndex, yIndex))
         return Float.NaN;

      float heightSearch;

      if (!Float.isNaN(heightSearch = (float) hasDataInDirection(xIndex, yIndex, true, filterParameters.getHoleProximityThreshold())))
      {
         return heightSearch;
      }
      else if (!Float.isNaN(heightSearch = (float) hasDataInDirection(xIndex, yIndex, false, filterParameters.getHoleProximityThreshold())))
      {
         return heightSearch;
      }

      return Float.NaN;
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

   private boolean firstTick = true;
   private long startTime;
   private int updateFrequencyCounter = 0;

   private void updateFrequency()
   {
      if (firstTick)
      {
         startTime = System.currentTimeMillis();
         firstTick = false;
      }
      else
      {
         updateFrequencyCounter++;
         double timePerCallSeconds = 1e-3 * ((double) (System.currentTimeMillis() - startTime) / updateFrequencyCounter);
         System.out.println("Frequency: " + (1 / timePerCallSeconds) + " Hz");
      }
   }

   private void export()
   {
      HeightMapMessage message = latestMessage.get();
      if (message == null)
      {
         LogTools.info("No height map available, cannot export.");
         return;
      }

      try
      {
         JSONSerializer<HeightMapMessage> serializer = new JSONSerializer<>(new HeightMapMessagePubSubType());
         byte[] serializedHeightMap = serializer.serializeToBytes(message);

         SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
         String fileName = "HeightMap" + dateFormat.format(new Date()) + ".json";
         String file = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator + fileName;

         FileTools.ensureFileExists(new File(file).toPath());
         FileOutputStream outputStream = new FileOutputStream(file);
         PrintStream printStream = new PrintStream(outputStream);

         printStream.write(serializedHeightMap);
         printStream.flush();
         outputStream.close();
         printStream.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private static double snapValueToGrid(double value, double gridResolution)
   {
      return ((int) Math.round(value / gridResolution)) * gridResolution;
   }

   private static void snapPointToGrid(Point2DBasics pointToSnap, double gridResolution)
   {
      pointToSnap.setX(snapValueToGrid(pointToSnap.getX(), gridResolution));
      pointToSnap.setY(snapValueToGrid(pointToSnap.getY(), gridResolution));
   }
}
