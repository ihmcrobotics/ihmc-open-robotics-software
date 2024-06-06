package us.ihmc.rdx.ui.graphics.ros2;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Heartbeat;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.perception.tools.NativeMemoryTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.rdx.RDXHeightMapRenderer;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.imgui.ImGuiFrequencyPlot;
import us.ihmc.rdx.ui.graphics.RDXHeightMapGraphicNew;
import us.ihmc.rdx.ui.graphics.RDXVisualizer;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;

import java.nio.ByteBuffer;
import java.util.Set;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class RDXHeightMapVisualizer extends RDXVisualizer
{
   private final RDXHeightMapRenderer heightMapRenderer = new RDXHeightMapRenderer();
   private final RDXHeightMapGraphicNew heightMapGraphicNew = new RDXHeightMapGraphicNew();
   private final ExecutorService executorService;
   private final ImGuiFrequencyPlot frequencyPlot = new ImGuiFrequencyPlot();

   private final ImBoolean enableHeightMapVisualizer = new ImBoolean(true);
   private final ImBoolean enableHeightMapRenderer = new ImBoolean(false);
   private final ImBoolean displayGlobalHeightMapImage = new ImBoolean(false);

   private final RigidBodyTransform zUpToWorldTransform = new RigidBodyTransform();
   private final TerrainMapData terrainMapData = new TerrainMapData(RapidHeightMapExtractor.getHeightMapParameters().getCropWindowSize(),
                                                                    RapidHeightMapExtractor.getHeightMapParameters().getCropWindowSize());

   private HeightMapMessage latestHeightMapMessage = new HeightMapMessage();
   private HeightMapData latestHeightMapData;
   private Mat heightMapImage;
   private Mat compressedBytesMat;
   private ByteBuffer incomingCompressedImageBuffer;
   private BytePointer incomingCompressedImageBytePointer;

   private int compressedBufferDefaultSize = 100000;

   private ROS2Heartbeat activeHeartbeat;

   private float pixelScalingFactor = 10000.0f;
   private boolean heightMapMessageGenerated = false;

   public RDXHeightMapVisualizer()
   {
      super("Height Map");

      executorService = Executors.newFixedThreadPool(4);
   }

   public void setupForHeightMapMessage(ROS2PublishSubscribeAPI ros2)
   {
      ros2.subscribeViaCallback(PerceptionAPI.HEIGHT_MAP_OUTPUT, this::acceptHeightMapMessage);
      activeHeartbeat = new ROS2Heartbeat(ros2, PerceptionAPI.REQUEST_HEIGHT_MAP);
   }

   public void setupForImageMessage(ROS2Helper ros2)
   {
      ros2.subscribeViaCallback(PerceptionAPI.HEIGHT_MAP_CROPPED, this::acceptImageMessage);
      activeHeartbeat = new ROS2Heartbeat(ros2, PerceptionAPI.REQUEST_HEIGHT_MAP);
   }

   @Override
   public void create()
   {
      super.create();
      int cellsPerAxis = RapidHeightMapExtractor.getHeightMapParameters().getCropWindowSize();
      heightMapRenderer.create(cellsPerAxis * cellsPerAxis);
   }

   public void acceptHeightMapMessage(HeightMapMessage heightMapMessage)
   {
      frequencyPlot.recordEvent();
      if (isActive())
      {
         updateGridMapGraphic(heightMapMessage);
      }
   }

   public void updateGridMapGraphic(HeightMapMessage heightMapMessage)
   {
      executorService.submit(() ->
        {
           if (enableHeightMapVisualizer.get())
           {
              heightMapGraphicNew.generateMeshesAsync(heightMapMessage);
           }
        });
   }

   public void acceptImageMessage(ImageMessage imageMessage)
   {
      frequencyPlot.recordEvent();
      if (isActive())
      {
         executorService.submit(() ->
           {
              pixelScalingFactor = imageMessage.getDepthDiscretization();
              zUpToWorldTransform.set(imageMessage.getOrientation(), imageMessage.getPosition());

              if (heightMapImage == null)
              {
                 heightMapImage = new Mat(imageMessage.getImageHeight(), imageMessage.getImageWidth(), opencv_core.CV_16UC1);
                 compressedBytesMat = new Mat(1, 1, opencv_core.CV_8UC1);
                 incomingCompressedImageBuffer = NativeMemoryTools.allocate(compressedBufferDefaultSize);
                 incomingCompressedImageBytePointer = new BytePointer(incomingCompressedImageBuffer);
                 LogTools.warn("Creating Buffer of Size: {}", compressedBufferDefaultSize);
              }

              if (latestHeightMapData == null)
              {
                 latestHeightMapData = new HeightMapData(RapidHeightMapExtractor.getHeightMapParameters().getGlobalCellSizeInMeters(),
                                                         RapidHeightMapExtractor.getHeightMapParameters().getGlobalWidthInMeters(),
                                                         imageMessage.getPosition().getX(),
                                                         imageMessage.getPosition().getY());
              }

              PerceptionMessageTools.convertToHeightMapImage(imageMessage,
                                                             heightMapImage,
                                                             incomingCompressedImageBuffer,
                                                             incomingCompressedImageBytePointer,
                                                             compressedBytesMat);

              if (!heightMapMessageGenerated)
              {
                 PerceptionMessageTools.convertToHeightMapData(heightMapImage,
                                                               latestHeightMapData,
                                                               imageMessage.getPosition(),
                                                               (float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalWidthInMeters(),
                                                               (float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalCellSizeInMeters());
                 latestHeightMapMessage = HeightMapMessageTools.toMessage(latestHeightMapData);
                 heightMapMessageGenerated = true;
              }

//              if (displayGlobalHeightMapImage.get())
//                 PerceptionDebugTools.displayDepth("Received Global Height Map", heightMapImage, 1);
//              else
//                 PerceptionDebugTools.clearAllWindows();
           });
      }
   }

   @Override
   public void setActive(boolean active)
   {
      super.setActive(active);
      if (!active)
      {
         executorService.shutdown();
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();

      ImGui.checkbox("Enable Height Map Visualizer", enableHeightMapVisualizer);
      ImGui.checkbox("Enable Height Map Renderer", enableHeightMapRenderer);
      ImGui.checkbox("Display Global Height Map Image", displayGlobalHeightMapImage);

      if (!isActive())
      {
         executorService.shutdown();
      }
      frequencyPlot.renderImGuiWidgets();
   }

   @Override
   public void update()
   {
      super.update();

      if (heightMapMessageGenerated)
      {
         heightMapMessageGenerated = false;
         updateGridMapGraphic(latestHeightMapMessage);
      }

      if (activeHeartbeat != null)
      {
         activeHeartbeat.setAlive(isActive());
      }

      boolean isActive = isActive();
      if (isActive && enableHeightMapVisualizer.get())
      {
         heightMapGraphicNew.update();
      }

      if (isActive && enableHeightMapRenderer.get() && heightMapImage != null)
      {
         if (heightMapImage.ptr(0) != null)
         {
            //PerceptionDebugTools.printMat("Height Map Image", heightMapImage, 10);
            heightMapRenderer.update(zUpToWorldTransform, heightMapImage.ptr(0), (float) RapidHeightMapExtractor.getHeightMapParameters().getHeightOffset(),
                                     zUpToWorldTransform.getTranslation().getX32(), zUpToWorldTransform.getTranslation().getY32(),
                                     heightMapImage.rows() / 2, (float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalCellSizeInMeters(),
                                     pixelScalingFactor);
         }
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (isActive() && sceneLevelCheck(sceneLevels))
      {
         if (enableHeightMapVisualizer.get())
         {
            heightMapGraphicNew.getRenderables(renderables, pool);
         }

         if (enableHeightMapRenderer.get())
         {
            heightMapRenderer.getRenderables(renderables, pool);
         }
      }
   }

   public void destroy()
   {
      executorService.shutdown();
      if (activeHeartbeat != null)
      {
         activeHeartbeat.destroy();
      }
      heightMapGraphicNew.destroy();
   }

   public HeightMapMessage getLatestHeightMapMessage()
   {
      return latestHeightMapMessage;
   }

   public HeightMapData getLatestHeightMapData()
   {
      return latestHeightMapData;
   }

   public TerrainMapData getTerrainMapData()
   {
      terrainMapData.setHeightMap(heightMapImage);
      terrainMapData.setSensorOrigin(zUpToWorldTransform.getTranslation().getX(), zUpToWorldTransform.getTranslation().getY());
      return terrainMapData;
   }

   public ImBoolean getDisplayGlobalHeightMapImage()
   {
      return displayGlobalHeightMapImage;
   }
}
