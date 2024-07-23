package us.ihmc.rdx.ui.graphics.ros2;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.GlobalMapMessage;
import perception_msgs.msg.dds.GlobalMapTileMessage;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.perception.tools.NativeMemoryTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.rdx.RDXHeightMapRenderer;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXGlobalHeightMapGraphic;
import us.ihmc.rdx.ui.graphics.RDXHeightMapGraphic;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensorProcessing.globalHeightMap.GlobalLattice;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.nio.ByteBuffer;
import java.util.List;
import java.util.Set;

public class RDXROS2HeightMapVisualizer extends RDXROS2MultiTopicVisualizer
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private final RDXHeightMapRenderer heightMapRenderer = new RDXHeightMapRenderer();
   private final RDXHeightMapGraphic heightMapGraphicNew = new RDXHeightMapGraphic();
   private final RDXGlobalHeightMapGraphic globalHeightMapGraphic = new RDXGlobalHeightMapGraphic();
   private final ResettableExceptionHandlingExecutorService executorService;

   private final ImBoolean enableHeightMapVisualizer = new ImBoolean(true);
   private final ImBoolean enableGlobalHeightMapVisualizer = new ImBoolean(false);
   private final ImBoolean enableHeightMapRenderer = new ImBoolean(false);
   private final ImBoolean displayGlobalHeightMapImage = new ImBoolean(false);

   private final RigidBodyTransform zUpToWorldTransform = new RigidBodyTransform();
   private final TerrainMapData terrainMapData = new TerrainMapData(RapidHeightMapExtractor.getHeightMapParameters().getCropWindowSize(),
                                                                    RapidHeightMapExtractor.getHeightMapParameters().getCropWindowSize());

   private ROS2PublishSubscribeAPI ros2;
   private HeightMapMessage latestHeightMapMessage = new HeightMapMessage();
   private HeightMapData latestHeightMapData;
   private Mat heightMapImage;
   private Mat compressedBytesMat;
   private ByteBuffer incomingCompressedImageBuffer;
   private BytePointer incomingCompressedImageBytePointer;

   private int compressedBufferDefaultSize = 100000;

   private float pixelScalingFactor = 10000.0f;
   private boolean heightMapMessageGenerated = false;

   public RDXROS2HeightMapVisualizer(String title)
   {
      super(title);

      executorService = MissingThreadTools.newSingleThreadExecutor("Height Map Visualizer Subscription", true, 1);
   }

   @Override
   public List<ROS2Topic<?>> getTopics()
   {
      return List.of(PerceptionAPI.HEIGHT_MAP_OUTPUT, PerceptionAPI.HEIGHT_MAP_CROPPED);
   }

   public void setupForHeightMapMessage(ROS2PublishSubscribeAPI ros2)
   {
      this.ros2 = ros2;
      ros2.subscribeViaCallback(PerceptionAPI.HEIGHT_MAP_OUTPUT, this::acceptHeightMapMessage);
   }

   public void setupForImageMessage(ROS2PublishSubscribeAPI ros2)
   {
      this.ros2 = ros2;
      ros2.subscribeViaCallback(PerceptionAPI.HEIGHT_MAP_CROPPED, this::acceptImageMessage);
   }

   public void setupForGlobalHeightMapTileMessage(ROS2PublishSubscribeAPI ros2)
   {
      this.ros2 = ros2;
      ros2.subscribeViaCallback(PerceptionAPI.GLOBAL_HEIGHT_MAP_TILE, this::acceptGlobalMapTileMessage);
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
      if (isActive())
      {
         updateGridMapGraphic(heightMapMessage);
      }
   }

   public void acceptGlobalMapMessage(GlobalMapMessage globalMapMessage)
   {
      if (enableGlobalHeightMapVisualizer.get())
      {
         for (int i = 0; i < globalMapMessage.getGlobalMap().size(); i++)
         {
            acceptGlobalMapTileMessage(globalMapMessage.getGlobalMap().get(i));
         }
      }
   }

   public void acceptGlobalMapTileMessage(GlobalMapTileMessage globalMapTileMessage)
   {
      if (enableGlobalHeightMapVisualizer.get())
      {
         int hashCode = GlobalLattice.hashCodeOfTileIndices(globalMapTileMessage.getCenterX(), globalMapTileMessage.getCenterY());
         globalHeightMapGraphic.generateMeshesAsync(globalMapTileMessage.getHeightMap(), hashCode);
      }
   }

   private void updateGridMapGraphic(HeightMapMessage heightMapMessage)
   {
      executorService.clearQueueAndExecute(() ->
                                           {
                                              if (enableHeightMapVisualizer.get())
                                              {
                                                 heightMapGraphicNew.generateMeshesAsync(heightMapMessage);
                                              }
                                           });

      getFrequency(PerceptionAPI.HEIGHT_MAP_OUTPUT).ping();
   }

   public void acceptImageMessage(ImageMessage imageMessage)
   {
      if (isActive())
      {
         executorService.clearQueueAndExecute(() ->
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
                                                    latestHeightMapData = new HeightMapData(RapidHeightMapExtractor.getHeightMapParameters()
                                                                                                                   .getGlobalCellSizeInMeters(),
                                                                                            RapidHeightMapExtractor.getHeightMapParameters()
                                                                                                                   .getGlobalWidthInMeters(),
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
                                                                                                  (float) RapidHeightMapExtractor.getHeightMapParameters()
                                                                                                                                 .getGlobalWidthInMeters(),
                                                                                                  (float) RapidHeightMapExtractor.getHeightMapParameters()
                                                                                                                                 .getGlobalCellSizeInMeters());
                                                    latestHeightMapMessage = HeightMapMessageTools.toMessage(latestHeightMapData);
                                                    heightMapMessageGenerated = true;
                                                 }

                                              });
      }

      getFrequency(PerceptionAPI.HEIGHT_MAP_CROPPED).ping();
   }

   @Override
   public void setActive(boolean active)
   {
      super.setActive(active);
      if (!active)
      {
         executorService.clearTaskQueue();
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      if (ros2 != null && ImGui.button(labels.get("Reset Ground to Feet")))
         ros2.publish(PerceptionAPI.RESET_HEIGHT_MAP);

      if (ImGui.collapsingHeader(labels.get("Visualization Options")))
      {
         ImGui.checkbox(labels.get("Enable Height Map Visualizer"), enableHeightMapVisualizer);
         ImGui.checkbox(labels.get("Enable Global Height Map Visualizer"), enableGlobalHeightMapVisualizer);
         ImGui.checkbox(labels.get("Enable Height Map Renderer"), enableHeightMapRenderer);
         ImGui.checkbox(labels.get("Display Global Height Map Image"), displayGlobalHeightMapImage);
      }
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

      boolean isActive = isActive();
      if (isActive && enableHeightMapVisualizer.get())
      {
         heightMapGraphicNew.update();
      }
      if (enableGlobalHeightMapVisualizer.get())
      {
         heightMapGraphicNew.update();
         globalHeightMapGraphic.update();
      }

      if (isActive && enableHeightMapRenderer.get() && heightMapImage != null)
      {
         if (heightMapImage.ptr(0) != null)
         {
            //PerceptionDebugTools.printMat("Height Map Image", heightMapImage, 10);
            heightMapRenderer.update(zUpToWorldTransform,
                                     heightMapImage.ptr(0),
                                     (float) RapidHeightMapExtractor.getHeightMapParameters().getHeightOffset(),
                                     zUpToWorldTransform.getTranslation().getX32(),
                                     zUpToWorldTransform.getTranslation().getY32(),
                                     heightMapImage.rows() / 2,
                                     (float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalCellSizeInMeters(),
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

         if (enableGlobalHeightMapVisualizer.get())
         {
            globalHeightMapGraphic.getRenderables(renderables, pool);
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
      executorService.destroy();
      heightMapGraphicNew.destroy();
      globalHeightMapGraphic.destroy();
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
