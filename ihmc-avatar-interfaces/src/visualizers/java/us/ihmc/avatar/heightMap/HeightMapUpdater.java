package us.ihmc.avatar.heightMap;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import controller_msgs.msg.dds.HeightMapMessage;
import controller_msgs.msg.dds.HeightMapMessagePubSubType;
import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TIntArrayList;
import javafx.stage.FileChooser;
import javafx.stage.Stage;
import org.apache.commons.lang3.tuple.Pair;
import sensor_msgs.PointCloud2;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.PointCloudData;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.heightMap.HeightMapManager;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

import java.io.*;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

public class HeightMapUpdater
{
   private static final boolean printFrequency = false;
   private static final boolean printQueueSize = false;

   static final boolean USE_OUSTER_FRAME = true;
   static final RigidBodyTransform APPROX_OUSTER_TRANSFORM = new RigidBodyTransform();
   static
   {
//      double ousterPitch = Math.toRadians(21.5);
//      APPROX_OUSTER_TRANSFORM.getRotation().setToPitchOrientation(ousterPitch);

      double ousterPitch = Math.toRadians(30.0);
      APPROX_OUSTER_TRANSFORM.getRotation().setToPitchOrientation(ousterPitch);
      APPROX_OUSTER_TRANSFORM.getTranslation().set(-0.2, 0.0, 1.0);
   }

   private final Stage stage;
   private final Messager messager;
   private final HeightMapParameters parameters;

   private final PoseReferenceFrame ousterFrame = new PoseReferenceFrame("ousterFrame", ReferenceFrame.getWorldFrame());
   private final HeightMapManager heightMap;
   private final IHMCROS2Publisher<HeightMapMessage> publisher;
   private final AtomicReference<Point2D> gridCenter = new AtomicReference<>(new Point2D());
   private final AtomicReference<HeightMapMessage> latestMessage = new AtomicReference<>();

   private final TIntArrayList holeKeyList = new TIntArrayList();
   private final TFloatArrayList holeHeights = new TFloatArrayList();

   private final ConcurrentLinkedQueue<Pair<PointCloud2, FramePose3D>> pointCloudQueue = new ConcurrentLinkedQueue<>();
   private final ExecutorService heightMapUpdater = Executors.newSingleThreadExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()));
   private final AtomicBoolean updateThreadIsRunning = new AtomicBoolean();

   private int publishFrequencyCounter = 0;
   private final AtomicInteger publishFrequency = new AtomicInteger();
   private final AtomicInteger totalUpdateCount = new AtomicInteger();

   public HeightMapUpdater(Messager messager, ROS2Node ros2Node, Stage stage)
   {
      this.stage = stage;
      this.messager = messager;
      publisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.HEIGHT_MAP_OUTPUT);

      parameters = new HeightMapParameters();
      heightMap = new HeightMapManager(parameters.getGridResolutionXY(), parameters.getGridSizeXY());
      AtomicReference<Boolean> enableUpdates = messager.createInput(HeightMapMessagerAPI.EnableUpdates);

      messager.registerTopicListener(HeightMapMessagerAPI.PointCloudData, pointCloudData ->
      {
         if (enableUpdates.get())
         {
            pointCloudQueue.add(pointCloudData);

            if (!updateThreadIsRunning.getAndSet(true))
            {
               heightMapUpdater.execute(this::runUpdateThread);
            }
         }
      });

      messager.registerTopicListener(HeightMapMessagerAPI.GridCenterX, x -> gridCenter.set(new Point2D(x, gridCenter.get().getY())));
      messager.registerTopicListener(HeightMapMessagerAPI.GridCenterY, y -> gridCenter.set(new Point2D(gridCenter.get().getX(), y)));

      messager.registerTopicListener(HeightMapMessagerAPI.PublishFrequency, publishFrequency::set);
      messager.registerTopicListener(HeightMapMessagerAPI.Export, e -> ThreadTools.startAThread(this::export, "Height map exporter"));
      messager.registerTopicListener(HeightMapMessagerAPI.Import, i -> importHeightMap());
   }

   private void runUpdateThread()
   {
      int updatesWithoutDataCounter = 0;
      long sleepTimeMillis = 20;
      long maxIdleTimeMillis = 1000;

      while (updatesWithoutDataCounter < maxIdleTimeMillis / sleepTimeMillis)
      {
         Pair<PointCloud2, FramePose3D> data = pointCloudQueue.poll();
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

      updateThreadIsRunning.set(false);
   }

   private void update(Pair<PointCloud2, FramePose3D> pointCloudData)
   {
      if (printFrequency)
      {
         updateFrequency();
      }

      PointCloudData pointCloud = new PointCloudData(pointCloudData.getKey(), 1000000, false);
      ousterFrame.setPoseAndUpdate(pointCloudData.getRight());

      if (USE_OUSTER_FRAME)
      {
         // Transform ouster data
         for (int i = 0; i < pointCloud.getPointCloud().length; i++)
         {
            FramePoint3D point = new FramePoint3D(ousterFrame, pointCloud.getPointCloud()[i]);
            point.changeFrame(ReferenceFrame.getWorldFrame());
            pointCloud.getPointCloud()[i].set(point);
         }
      }
      else
      {
         for (int i = 0; i < pointCloud.getPointCloud().length; i++)
         {
            pointCloud.getPointCloud()[i].applyTransform(APPROX_OUSTER_TRANSFORM);
         }
      }

      Point2D gridCenter = this.gridCenter.getAndSet(null);
      if (gridCenter != null)
      {
         heightMap.setGridCenter(gridCenter.getX(), gridCenter.getY());
      }

      // Update height map
      heightMap.update(pointCloud.getPointCloud());
      totalUpdateCount.incrementAndGet();

      if (--publishFrequencyCounter <= 0)
      {
         /* estimate ground height */
         double estimatedGroundHeight = estimateGroundHeight(pointCloud.getPointCloud());

         /* filter near and below ground height and outliers that seem too high */
         performFiltering(estimatedGroundHeight);

         /* pack ROS message */
         HeightMapMessage message = buildMessage();
         message.setEstimatedGroundHeight(estimatedGroundHeight);

         messager.submitMessage(HeightMapMessagerAPI.HeightMapData, message);
         publisher.publish(message);

         publishFrequencyCounter = publishFrequency.get();
         latestMessage.set(message);
      }

      if (printQueueSize)
         LogTools.info("Point cloud queue: " + pointCloudQueue.size());
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
         if (!heightMap.isGroundCell(i))
         {
            int key = heightMap.getKey(i);
            message.getKeys().add(key);
            message.getHeights().add((float) heightMap.getHeightAt(i));
         }
         else
         {
            int key = heightMap.getKey(i);
            message.getGroundKeys().add(key);
            message.getGroundHeights().add((float) heightMap.getHeightAt(i));
         }
      }

      LogTools.info(holeKeyList.size() + " holes found");
      message.getHoleKeys().addAll(holeKeyList);
      message.getHoleHeights().addAll(holeHeights);

//      while (message.getHeights().size() >= HeightMapManager.maxCellCount)
//      {
//         message.getHeights().removeAt(message.getHeights().size() - 1);
//         message.getKeys().removeAt(message.getKeys().size() - 1);
//      }

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

   private static final int[] xOffsetFourConnectedGrid = new int[]{-1, 0, 1, 0};
   private static final int[] yOffsetFourConnectedGrid = new int[]{0, 1, 0, -1};

   private static final int[] xOffsetEightConnectedGrid = new int[]{-1, -1, 0, 1, 1,  1, 0,  -1};
   private static final int[] yOffsetEightConnectedGrid = new int[]{0,  1,  1, 1, 0, -1, -1 , -1};

   private void performFiltering(double estimatedGroundHeight)
   {
      /* Remove cells near ground height */
      double groundEpsilonToFilter = 0.03;
      double groundHeightThreshold = groundEpsilonToFilter + estimatedGroundHeight;

      for (int i = heightMap.getNumberOfCells() - 1; i >= 0; i--)
      {
         if (heightMap.getHeightAt(i) < groundHeightThreshold)
            heightMap.setGroundCell(i, true);
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
               heightMap.setGroundCell(i, false);
               continue outerLoop;
            }
         }

         if (numberOfNeighbors < minNumberOfNeighbors)
         {
            heightMap.setGroundCell(i, true);
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
            heightMap.resetAtHeight(i, resetHeight / numberOfNeighbors);
         }
      }

      /* Once the map has had a chance to initialize, fill in any holes */
      if (totalUpdateCount.get() > 50)
      {
         holeKeyList.clear();
         holeHeights.clear();

         /* For any cell without data, populate with average of neighbors */
         for (int xIndex = 1; xIndex < heightMap.getCellsPerAxis() - 1; xIndex++)
         {
            for (int yIndex = 1; yIndex < heightMap.getCellsPerAxis() - 1; yIndex++)
            {
               if (heightMap.cellHasData(xIndex, yIndex))
                  continue;

               double heightSum = 0.0;
               int numberOfNeighbors = 0;

               for (int j = 0; j < xOffsetFourConnectedGrid.length; j++)
               {
                  int neighborX = xIndex + xOffsetFourConnectedGrid[j];
                  int neighborY = yIndex + yOffsetFourConnectedGrid[j];

                  if (heightMap.cellHasData(neighborX, neighborY) && heightMap.getHeightAt(neighborX, neighborY) - estimatedGroundHeight > 0.5)
                  {
                     heightSum += heightMap.getHeightAt(neighborX, neighborY);
                     numberOfNeighbors++;
                  }
               }

               if (numberOfNeighbors > 0)
               {
                  holeKeyList.add(HeightMapTools.indicesToKey(xIndex, yIndex, heightMap.getCenterIndex()));
                  holeHeights.add((float) (heightSum / numberOfNeighbors));
               }
            }
         }

//         for (int key = 0; key < totalGridSize; key++)
//         {
//            if (heightMap.isCellPresent(key))
//               continue;
//
//            int xIndex = HeightMapTools.keyToXIndex(key, heightMap.getCenterIndex());
//            int yIndex = HeightMapTools.keyToYIndex(key, heightMap.getCenterIndex());
//
//            float heightSum = 0.0f;
//            int numberOfNeighbors = 0;
//
//            for (int j = 0; j < xOffsetFourConnectedGrid.length; j++)
//            {
//               int neighborX = xIndex + xOffsetFourConnectedGrid[j];
//               int neighborY = yIndex + yOffsetFourConnectedGrid[j];
//
//               if (heightMap.isCellPresent(neighborX, neighborY) && heightMap.getHeightAt(neighborX, neighborY) - estimatedGroundHeight > 0.5)
//               {
//                  heightSum += heightMap.getHeightAt(neighborX, neighborY);
//                  numberOfNeighbors++;
//               }
//            }
//
//            if (numberOfNeighbors > 0)
//            {
//               holeKeyList.add(key);
//               holeHeights.add(heightSum / numberOfNeighbors);
//            }
//         }
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

   private final AtomicBoolean importing = new AtomicBoolean();

   private void importHeightMap()
   {
      ThreadTools.startAThread(() ->
                               {
                                  if (importing.getAndSet(true))
                                  {
                                     return;
                                  }

                                  messager.submitMessage(HeightMapMessagerAPI.EnableUpdates, false);

                                  FileChooser fileChooser = new FileChooser();
                                  fileChooser.setTitle("Import Height Map");
                                  fileChooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("Height Map Files (*.json)", "*.json"));
                                  File file = fileChooser.showOpenDialog(stage);

                                  ObjectMapper objectMapper = new ObjectMapper();
                                  try
                                  {
                                     JSONSerializer<HeightMapMessage> serializer = new JSONSerializer<>(new HeightMapMessagePubSubType());
                                     InputStream inputStream = new FileInputStream(file);
                                     JsonNode jsonNode = objectMapper.readTree(inputStream);
                                     HeightMapMessage heightMapMessage = serializer.deserialize(jsonNode.toString());
                                     inputStream.close();

                                     messager.submitMessage(HeightMapMessagerAPI.HeightMapData, heightMapMessage);
                                  }
                                  catch (IOException e)
                                  {
                                     e.printStackTrace();
                                  }

                                  importing.set(false);
                               }, "Height map importer");
   }
}
