package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.ControllerFootstepQueueMonitor;
import us.ihmc.log.LogTools;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.gpuHeightMap.worldModel.ChunkedMapManager;
import us.ihmc.perception.heightMap.HeightMapMessageTools;
import us.ihmc.perception.heightMap.HeightMapTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.perception.heightMap.HeightMapData;
import us.ihmc.perception.heightMap.HeightMapParameters;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;

/**
 * This class takes care of managing the {@link RapidHeightMapExtractor}. This class can be used in a remote process, or locally as well.
 */
public class RapidHeightMapManager
{
   private final ReferenceFrame heightMapCenter;
   private final HeightMapParameters heightMapParameters;
   private final RapidHeightMapExtractor rapidHeightMapExtractor;

   private final List<ReferenceFrame> footSoleFrames = new ArrayList<>();

   private final Notification resetHeightMapRequested = new Notification();
   private final Notification lowerHeightMapBackdropRequested = new Notification();

   private final RapidHeightMapDriftOffset rapidHeightMapDriftOffset;

   private final ROS2Publisher<HeightMapMessage> heightMapMessagePublisher;
   private final ROS2Publisher<HeightMapMessage> controllerHeightMapMessagePublisher;
   private final BytePointer compressedHeightMapPointer = new BytePointer();
   private final HeightMapData lastestGlobalHeightMapData;
   private final HeightMapData latestTerrainHeightMapData;
   private final Point3D heightMapCenterPoint = new Point3D();

   // These fields are created globally cause it takes compute time to create it in the update loop
   private final HeightMapMessage heightMapMessage;
   private final float[] heightsArray;
   private final ChunkedMapManager chunkedMapManager;
   private long sequenceId = 0;
   private FileOutputStream heightMapOutputStream;

   public RapidHeightMapManager(String robotName,
                                ROS2Node ros2Node,
                                ReferenceFrame leftFootSoleFrame,
                                ReferenceFrame rightFootSoleFrame,
                                ReferenceFrame heightMapCenter,
                                ControllerFootstepQueueMonitor controllerFootstepQueueMonitor,
                                HeightMapParameters heightMapParameters)
   {
      this.heightMapCenter = heightMapCenter;
      this.heightMapParameters = heightMapParameters;

      lastestGlobalHeightMapData = new HeightMapData((float) heightMapParameters.getCellSize(), (float) heightMapParameters.getGlobalWidthInMeters(), 0.0, 0.0);
      latestTerrainHeightMapData = new HeightMapData((float) heightMapParameters.getCellSize(),
                                                     (float) heightMapParameters.getTerrainWidthInMeters(),
                                                     0.0,
                                                     0.0);

      footSoleFrames.add(leftFootSoleFrame);
      footSoleFrames.add(rightFootSoleFrame);

      rapidHeightMapDriftOffset = new RapidHeightMapDriftOffset(controllerFootstepQueueMonitor);
      rapidHeightMapExtractor = new RapidHeightMapExtractor(heightMapParameters);
      chunkedMapManager = new ChunkedMapManager(ros2Node, heightMapParameters);

      // Again we do this to optimize the speed of the rapid height map
      heightMapMessage = new HeightMapMessage();
      int centerIndexGlobal = HeightMapTools.computeCenterIndex(heightMapParameters.getGlobalWidthInMeters(), heightMapParameters.getCellSize());
      int cellsPerAxisGlobal = 2 * centerIndexGlobal + 1;
      int totalCells = cellsPerAxisGlobal * cellsPerAxisGlobal;
      heightsArray = new float[totalCells];

      // We use a notification to only call resetting the height map in one place
      heightMapMessagePublisher = ros2Node.createPublisher(PerceptionAPI.HEIGHT_MAP_MESSAGE);
      ros2Node.createSubscription2(PerceptionAPI.RESET_HEIGHT_MAP, message -> resetHeightMapRequested.set());
      ros2Node.createSubscription2(PerceptionAPI.LOWER_HEIGHT_MAP_BACKDROP, message -> lowerHeightMapBackdropRequested.set());

      controllerHeightMapMessagePublisher = ros2Node.createPublisher(HumanoidControllerAPI.getTopic(HeightMapMessage.class, robotName));
   }

   public static void logHeightMapToFile(FileOutputStream fos, float[] packedArray, double timestamp) throws IOException
   {
      int frameSize = 8 + packedArray.length * Float.BYTES;

      ByteBuffer buffer = ByteBuffer.allocate(4 + frameSize);
      buffer.order(ByteOrder.LITTLE_ENDIAN);

      // Write frame size (int)
      buffer.putInt(frameSize);

      // Write timestamp (double)
      buffer.putDouble(timestamp);

      // Write packed float data
      for (float f : packedArray)
      {
         buffer.putFloat(f);
      }

      // Write to file
      fos.write(buffer.array());
   }

   public void updateAndPublish(GpuMat latestDepthImage, CameraIntrinsics depthIntrinsics, ReferenceFrame cameraFrame, ReferenceFrame cameraZUpFrame)
   {
      // Update the sensor origin here with the latest reference frame
      RigidBodyTransform heightMapFrameToWorldFrame = heightMapCenter.getTransformToWorldFrame();
      Point3D heightMapCenterOrigin = new Point3D(heightMapFrameToWorldFrame.getTranslation());

      updateInternal(latestDepthImage, depthIntrinsics, cameraFrame, cameraZUpFrame, heightMapCenterOrigin);

      // Publish the height map to anyone who is subscribing
      Mat hostGlobalHeightMap = new Mat();

      // Don't close this mat as its being used in the extractor till that finish's
      GpuMat deviceGlobalHeightMap = rapidHeightMapExtractor.getHeightMap();
      deviceGlobalHeightMap.download(hostGlobalHeightMap);

      // The center of this map should be centered in the world grid
      // The sensor origin isn't always at the center of a grid point, in fact it's often not in the center
      double currentCellX = (int) Math.round(heightMapCenterOrigin.getX32() / heightMapParameters.getCellSize()) * heightMapParameters.getCellSize();
      double currentCellY = (int) Math.round(heightMapCenterOrigin.getY32() / heightMapParameters.getCellSize()) * heightMapParameters.getCellSize();
      heightMapCenterPoint.set(currentCellX, currentCellY, 0.0);

      publishHeightMap(hostGlobalHeightMap);

      if (heightMapParameters.getEnableChunkedMap())
      {
         chunkedMapManager.updateAndPublish(hostGlobalHeightMap, heightMapCenterPoint);
      }

      hostGlobalHeightMap.close();
   }

   private void publishHeightMap(Mat globalHeightMap)
   {
      HeightMapTools.convertToHeightMapData(globalHeightMap,
                                            lastestGlobalHeightMapData,
                                            heightMapCenterPoint,
                                            (float) heightMapParameters.getGlobalWidthInMeters(),
                                            (float) heightMapParameters.getCellSize());

      HeightMapMessageTools.toMessage(lastestGlobalHeightMapData, heightMapMessage);

      if (heightMapParameters.getLogHeightMap())
      {
         float[] floatsToLog = HeightMapTools.packArrayForFile(globalHeightMap,
                                                               heightMapCenterPoint,
                                                               (float) heightMapParameters.getGlobalWidthInMeters(),
                                                               (float) heightMapParameters.getCellSize());
         try
         {
            String timestamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss_SSS"));

            Path heightMapDirectory = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY;

            Path binaryLogPath = heightMapDirectory.resolve(timestamp + "_HeightMapLog.bin");
            if (heightMapOutputStream == null)
            {
               heightMapOutputStream = new FileOutputStream(binaryLogPath.toFile(), true);
               LogTools.info("Writing height map log to " + binaryLogPath);
            }
            if (!Files.exists(heightMapDirectory))
            {
               Files.createDirectory(heightMapDirectory);
            }
         }
         catch (FileNotFoundException e)
         {
            throw new RuntimeException(e);
         }
         catch (IOException ignored)
         {
         }

         try
         {
            logHeightMapToFile(heightMapOutputStream, floatsToLog, System.currentTimeMillis());
         }
         catch (IOException e)
         {
            throw new RuntimeException(e);
         }
      }

      if (!heightMapParameters.getLogHeightMap())
      {
         if (heightMapOutputStream != null)
         {
            try
            {
               heightMapOutputStream.close();
               heightMapOutputStream = null;
            }
            catch (IOException e)
            {
               throw new RuntimeException(e);
            }
         }
      }

      sequenceId++;
      heightMapMessage.setSequenceId(sequenceId);
      heightMapMessagePublisher.publish(heightMapMessage);
      controllerHeightMapMessagePublisher.publish(heightMapMessage);
   }

   /**
    * Update the Height Map with the latest depth image from the sensor
    */
   private void updateInternal(GpuMat latestDepthImage,
                               CameraIntrinsics depthIntrinsicsCopy,
                               ReferenceFrame cameraFrame,
                               ReferenceFrame cameraZUpFrame,
                               Point3D heightMapFrameToWorldFrame)
   {
      // Option that gets triggered from a message sent from the user
      if (lowerHeightMapBackdropRequested.poll())
      {
         double footHeight = computeFootHeight();
         float loweredFootHeight = 1.0f;
         rapidHeightMapExtractor.reset(footHeight, loweredFootHeight);
         if (heightMapParameters.getDriftOffsetFilter())
         {
            rapidHeightMapDriftOffset.reset();
         }
      }

      // Option that gets triggered from a message sent from the user
      if (resetHeightMapRequested.poll())
      {
         double footHeight = computeFootHeight();
         rapidHeightMapExtractor.reset(footHeight);
         if (heightMapParameters.getDriftOffsetFilter())
         {
            rapidHeightMapDriftOffset.reset();
         }
      }

      // -------- Update the Height Map with the latest depth image from the sensor --------------
      // We expect to have knowledge of where the camera is in relation to the world so we can accurately display the height map
      RigidBodyTransform sensorToWorld = cameraFrame.getTransformToWorldFrame();
      RigidBodyTransform sensorToGround = cameraFrame.getTransformToDesiredFrame(cameraZUpFrame);
      RigidBodyTransform groundToWorld = cameraZUpFrame.getTransformToWorldFrame();

      // Update the Z translation of the sensor to match the world transform (to handle the sensor's vertical position)
      sensorToGround.getTranslation().setZ(sensorToWorld.getTranslation().getZ());

      // The controller can publish a status letting anyone listening know that the controller is aware of some amount of drift in the Z direction
      // If we have that parameter set to true, we update the heights of the height map to account for that drift
      if (heightMapParameters.getDriftOffsetFilter())
      {
         float driftOffsetInZ = rapidHeightMapDriftOffset.getUpdateDriftOffset();
         if (!Float.isNaN(driftOffsetInZ))
         {
            rapidHeightMapExtractor.updateHeightOffset(groundToWorld, driftOffsetInZ, depthIntrinsicsCopy, computeFootHeight());
         }
      }

      // Perform update, this actually creates the height map
      rapidHeightMapExtractor.update(latestDepthImage,
                                     depthIntrinsicsCopy,
                                     sensorToWorld,
                                     sensorToGround,
                                     groundToWorld,
                                     heightMapFrameToWorldFrame,
                                     computeFootHeight());
   }

   public HeightMapData getLatestHeightMapData()
   {
      GpuMat terrainCroppedHeightMap = rapidHeightMapExtractor.getTerrainCroppedHeightMap();
      Mat terrainHeightMap = new Mat();
      terrainCroppedHeightMap.download(terrainHeightMap);

      HeightMapTools.convertToHeightMapData(terrainHeightMap,
                                            latestTerrainHeightMapData,
                                            heightMapCenterPoint,
                                            (float) heightMapParameters.getTerrainWidthInMeters(),
                                            (float) heightMapParameters.getCellSize());
      return latestTerrainHeightMapData;
   }

   private double computeFootHeight()
   {
      double thicknessOfTheFoot = 0.02;
      double height = Double.POSITIVE_INFINITY;

      for (int i = 0; i < footSoleFrames.size(); i++)
      {
         height = Math.min(footSoleFrames.get(i).getTransformToWorldFrame().getTranslationZ(), height);
      }
      if (Double.isInfinite(height))
         height = 0.0;

      height -= thicknessOfTheFoot;

      return height;
   }

   public void destroy()
   {
      compressedHeightMapPointer.close();
      heightMapMessagePublisher.remove();
      rapidHeightMapExtractor.destroy();
      chunkedMapManager.destroy();
   }
}