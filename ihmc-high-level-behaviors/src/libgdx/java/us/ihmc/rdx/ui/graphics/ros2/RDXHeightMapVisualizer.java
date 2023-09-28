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
import us.ihmc.perception.tools.NativeMemoryTools;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.rdx.RDXHeightMapRenderer;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.visualizers.ImGuiFrequencyPlot;
import us.ihmc.rdx.ui.visualizers.RDXVisualizer;
import us.ihmc.rdx.visualizers.RDXGridMapGraphic;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;

import java.nio.ByteBuffer;
import java.util.Set;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class RDXHeightMapVisualizer extends RDXVisualizer
{
   private final RDXHeightMapRenderer heightMapRenderer = new RDXHeightMapRenderer();
   private final RDXGridMapGraphic gridMapGraphic = new RDXGridMapGraphic();
   private final ExecutorService executorService;
   private final ImGuiFrequencyPlot frequencyPlot = new ImGuiFrequencyPlot();

   private final ImBoolean inPaintHeight = new ImBoolean(false);
   private final ImBoolean renderGroundPlane = new ImBoolean(false);
   private final ImBoolean renderGroundCells = new ImBoolean(false);
   private final ImBoolean enableHeightMapVisualizer = new ImBoolean(false);
   private final ImBoolean enableHeightMapRenderer = new ImBoolean(true);
   private final ImBoolean displayGlobalHeightMapImage = new ImBoolean(false);

   private final RigidBodyTransform zUpToWorldTransform = new RigidBodyTransform();
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
      activeHeartbeat = new ROS2Heartbeat(ros2, PerceptionAPI.PUBLISH_HEIGHT_MAP);
   }

   public void setupForImageMessage(ROS2Helper ros2)
   {
      ros2.subscribeViaCallback(PerceptionAPI.HEIGHT_MAP_GLOBAL, this::acceptImageMessage);
      activeHeartbeat = new ROS2Heartbeat(ros2, PerceptionAPI.PUBLISH_HEIGHT_MAP);
   }

   @Override
   public void create()
   {
      super.create();
      int cellsPerAxis = RapidHeightMapExtractor.CROP_WINDOW_SIZE + 1;
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
              gridMapGraphic.setInPaintHeight(inPaintHeight.get());
              gridMapGraphic.setRenderGroundPlane(renderGroundPlane.get());
              gridMapGraphic.setRenderGroundCells(renderGroundCells.get());
              gridMapGraphic.generateMeshesAsync(heightMapMessage);
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
                 latestHeightMapData = new HeightMapData(RapidHeightMapExtractor.GLOBAL_CELL_SIZE_IN_METERS,
                                                         RapidHeightMapExtractor.GLOBAL_WIDTH_IN_METERS,
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
                 PerceptionMessageTools.convertToHeightMapData(heightMapImage.ptr(0),
                                                               latestHeightMapData,
                                                               imageMessage.getPosition(),
                                                               RapidHeightMapExtractor.GLOBAL_WIDTH_IN_METERS,
                                                               RapidHeightMapExtractor.GLOBAL_CELL_SIZE_IN_METERS);
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
      ImGui.checkbox("In Paint Height", inPaintHeight);
      ImGui.checkbox("Render Ground Plane", renderGroundPlane);
      ImGui.checkbox("Render Ground Cells", renderGroundCells);
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
         gridMapGraphic.update();
      }

      if (isActive && enableHeightMapRenderer.get() && heightMapImage != null)
      {
         heightMapRenderer.update(zUpToWorldTransform,
                                  heightMapImage.ptr(0),
                                  zUpToWorldTransform.getTranslation().getX32(),
                                  zUpToWorldTransform.getTranslation().getY32(),
                                  heightMapImage.rows() / 2,
                                  RapidHeightMapExtractor.GLOBAL_CELL_SIZE_IN_METERS,
                                  pixelScalingFactor);
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (isActive() && sceneLevelCheck(sceneLevels))
      {
         if (enableHeightMapVisualizer.get())
         {
            gridMapGraphic.getRenderables(renderables, pool);
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
      gridMapGraphic.destroy();
   }

   public HeightMapMessage getLatestHeightMapMessage()
   {
      return latestHeightMapMessage;
   }

   public ImBoolean getDisplayGlobalHeightMapImage()
   {
      return displayGlobalHeightMapImage;
   }
}
