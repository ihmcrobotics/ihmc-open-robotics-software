package us.ihmc.rdx.ui.graphics.ros2;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import perception_msgs.msg.dds.GlobalMapTileMessage;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.TerrainMapMessage;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.communication.ContinuousHikingAPI;
import us.ihmc.perception.heightMap.HeightMapMessageTools;
import us.ihmc.perception.heightMap.HeightMapTools;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXGlobalHeightMapGraphic;
import us.ihmc.rdx.ui.graphics.RDXHeightMapRenderer;
import us.ihmc.rdx.ui.graphics.RDXOpenCVVideoVisualizer;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.perception.globalHeightMap.GlobalLattice;
import us.ihmc.perception.heightMap.HeightMapData;
import us.ihmc.perception.heightMap.HeightMapParameters;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.List;
import java.util.Set;

public class RDXROS2HeightMapVisualizer extends RDXROS2MultiTopicVisualizer
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private ROS2PublishSubscribeAPI ros2;
   private final ResettableExceptionHandlingExecutorService executorService;

   private final RDXOpenCVVideoVisualizer heightMapImageVisualizer = new RDXOpenCVVideoVisualizer("Height Map Image", "Height Map Image Panel", true);
   private final RDXHeightMapRenderer heightMapRenderer = new RDXHeightMapRenderer();
   private final RDXGlobalHeightMapGraphic globalHeightMapGraphic = new RDXGlobalHeightMapGraphic();

   private final ImBoolean enableGlobalHeightMapVisualizer = new ImBoolean(false);
   private final ImBoolean enableHeightMapRenderer = new ImBoolean(true);

   private Mat heightMap;
   private HeightMapData latestHeightMapData;
   private final HeightMapParameters heightMapParameters;
   private TerrainMapData latestTerrainMapData;

   private final Point3D heightMapCenter = new Point3D();

   private final Stopwatch stopwatch = new Stopwatch();
   private int cellsPerAxis;

   public RDXROS2HeightMapVisualizer(String title, HeightMapParameters heightMapParameters)
   {
      super(title);

      this.heightMapParameters = heightMapParameters;
      executorService = MissingThreadTools.newSingleThreadExecutor("Height Map Visualizer Subscription", true, 1);
   }

   @Override
   public List<ROS2Topic<?>> getTopics()
   {
      return List.of(PerceptionAPI.HEIGHT_MAP_MESSAGE);
   }

   public void setupForImageMessage(ROS2PublishSubscribeAPI ros2)
   {
      this.ros2 = ros2;
      ros2.subscribeViaCallback(PerceptionAPI.HEIGHT_MAP_MESSAGE, this::acceptHeightMapMessage);
      ros2.subscribeViaCallback(ContinuousHikingAPI.TERRAIN_MAP, this::acceptTerrainMapMessage);
   }

   public void setupForGlobalHeightMapTileMessage(ROS2PublishSubscribeAPI ros2)
   {
      this.ros2 = ros2;
      ros2.subscribeViaCallback(PerceptionAPI.GLOBAL_HEIGHT_MAP_TILE, this::acceptGlobalMapTileMessage);
   }

   public void acceptGlobalMapTileMessage(GlobalMapTileMessage globalMapTileMessage)
   {
      if (enableGlobalHeightMapVisualizer.get())
      {
         int hashCode = GlobalLattice.hashCodeOfTileIndices(globalMapTileMessage.getCenterX(), globalMapTileMessage.getCenterY());
         globalHeightMapGraphic.generateMeshesAsync(globalMapTileMessage.getHeightMap(), hashCode);
      }
   }

   public void acceptHeightMapMessage(HeightMapMessage heightMapMessage)
   {
      // Even if the height map is publishing, we aren't going to update anything with that data unless its active
      if (!isActive())
         return;

      executorService.clearQueueAndExecute(() ->
                                           {
                                              long sequenceId = heightMapMessage.getSequenceId();
                                              //TODO this shouldn't have to be the case, it cause's problems if you want to visualize one message
                                              if (sequenceId > 1)
                                              {
                                                 // We add +1 here because the height map is
                                                 int centerIndex = HeightMapTools.computeCenterIndex(4.0, 0.02);
                                                 cellsPerAxis = 2 * centerIndex + 1;
                                              }

                                              heightMapCenter.setX(heightMapMessage.getGridCenterX());
                                              heightMapCenter.setY(heightMapMessage.getGridCenterY());
                                              heightMap = HeightMapMessageTools.unpackMessageToMat(heightMapMessage, heightMapParameters);

                                              if (latestHeightMapData == null)
                                              {
                                                 latestHeightMapData = new HeightMapData(heightMapMessage.getXyResolution(),
                                                                                         heightMapMessage.getGridSizeXy(),
                                                                                         heightMapMessage.getGridCenterX(),
                                                                                         heightMapMessage.getGridCenterY());
                                              }
                                              HeightMapTools.convertToHeightMapData(heightMap,
                                                                                    latestHeightMapData,
                                                                                    heightMapCenter,
                                                                                    (float) heightMapParameters.getGlobalWidthInMeters(),
                                                                                    (float) heightMapParameters.getCellSizeInMeters(),
                                                                                    heightMapParameters);

                                              // This prevents the rendering from happening to early, it was throwing exceptions
                                              if (stopwatch.lapElapsed() > 3)
                                                 updateHeightMapImage();
                                           });

      getFrequency(PerceptionAPI.HEIGHT_MAP_MESSAGE).ping();
   }

   private void updateHeightMapImage()
   {
      DoublePointer minVal = new DoublePointer(1);
      DoublePointer maxVal = new DoublePointer(1);
      Point minLoc = new Point();
      Point maxLoc = new Point();

      opencv_core.minMaxLoc(heightMap, minVal, maxVal, minLoc, maxLoc, null);

      // Normalize depth to 8-bit
      Mat normalized = new Mat();
      double alpha = 255.0 / (maxVal.get() - minVal.get());
      double beta = -minVal.get() * alpha;
      heightMap.convertTo(normalized, opencv_core.CV_8U, alpha, beta);

      // Apply colormap
      Mat colorized = new Mat();
      opencv_imgproc.applyColorMap(normalized, colorized, opencv_imgproc.COLORMAP_JET);

      // Convert to RGBA for display/publishing
      Mat colorizedRGBA = new Mat();
      opencv_imgproc.cvtColor(colorized, colorizedRGBA, opencv_imgproc.COLOR_BGR2RGBA);

      heightMapImageVisualizer.setImage(colorizedRGBA);
   }

   public void acceptTerrainMapMessage(TerrainMapMessage terrainMapMessage)
   {
      // Even if the height map is publishing, we aren't going to update anything with that data unless its active
      if (!isActive())
         return;

      latestTerrainMapData = new TerrainMapData(terrainMapMessage);
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
      ImGui.indent();
      if (ros2 != null && ImGui.button(labels.get("Reset Ground to Feet")))
         ros2.publish(PerceptionAPI.RESET_HEIGHT_MAP);

      if (ros2 != null && ImGui.button(labels.get("Lower Height Map Backdrop")))
         ros2.publish(PerceptionAPI.LOWER_HEIGHT_MAP_BACKDROP);

      if (ImGui.collapsingHeader(labels.get("Visualization Options")))
      {
         ImGui.checkbox(labels.get("Enable Height Map Renderer"), enableHeightMapRenderer);
      }
      ImGui.unindent();
   }

   @Override
   public void update()
   {
      super.update();

      // From the visualizer side, if we don't want to visualize any height map, we don't need to update any graphics
      if (!isActive())
         return;

      if (cellsPerAxis > 0 && !heightMapRenderer.isHasBeenCreated())
      {
         heightMapRenderer.create(cellsPerAxis * cellsPerAxis);
         stopwatch.start();
      }
      if (enableGlobalHeightMapVisualizer.get())
      {
         globalHeightMapGraphic.update();
      }

      if (enableHeightMapRenderer.get() && heightMapRenderer.isHasBeenCreated())
      {
         // An additional check here to make sure that we have data in the image
         if (heightMap != null && heightMap.ptr(0) != null)
         {
            float pixelScalingFactor = 10000.0f;
            heightMapRenderer.update(heightMap,
                                     (float) heightMapParameters.getHeightOffset(),
                                     heightMapCenter.getX32(),
                                     heightMapCenter.getY32(),
                                     cellsPerAxis / 2,
                                     (float) heightMapParameters.getCellSizeInMeters(),
                                     pixelScalingFactor);
         }
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      // From the visualizer side, if we don't want to visualize any height map, we don't need to update any graphics
      if (!isActive())
         return;

      if (sceneLevelCheck(sceneLevels))
      {
         if (enableGlobalHeightMapVisualizer.get())
         {
            globalHeightMapGraphic.getRenderables(renderables, pool);
         }

         if (enableHeightMapRenderer.get() && heightMapRenderer.isHasBeenCreated())
         {
            heightMapRenderer.getRenderables(renderables, pool);
         }
      }
   }

   @Override
   public void destroy()
   {
      super.destroy();
      executorService.destroy();
      globalHeightMapGraphic.destroy();
      heightMapRenderer.dispose();
   }

   public HeightMapData getLatestHeightMapData()
   {
      return isActive() ? latestHeightMapData : null;
   }

   public TerrainMapData getLatestTerrainMapData()
   {
      return latestTerrainMapData;
   }

   public RDXOpenCVVideoVisualizer getHeightMapImageVisualizer()
   {
      return heightMapImageVisualizer;
   }
}