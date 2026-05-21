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
import perception_msgs.ChunkMessage;
import perception_msgs.HeightMapMessage;
import perception_msgs.TerrainMapMessage;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Heartbeat;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.perception.gpuMapping.TerrainMapMessageTools;
import us.ihmc.perception.gpuMapping.HeightMapData;
import us.ihmc.perception.gpuMapping.HeightMapMessageTools;
import us.ihmc.perception.gpuMapping.HeightMapTools;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXChunkedMapRenderer;
import us.ihmc.rdx.ui.graphics.RDXHeightMapRenderer;
import us.ihmc.rdx.ui.graphics.RDXImageVisualizer;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.nio.FloatBuffer;
import java.util.List;
import java.util.Set;

public class RDXROS2HeightMapVisualizer extends RDXROS2MultiTopicVisualizer
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ResettableExceptionHandlingExecutorService executorService;
   private final RDXImageVisualizer heightMapImageVisualizer = new RDXImageVisualizer("Height Map Image", "Height Map Image Panel", true);
   private final RDXHeightMapRenderer heightMapRenderer = new RDXHeightMapRenderer();
   private final RDXChunkedMapRenderer chunkedMapRenderer;
   private final ImBoolean requestChunkMap = new ImBoolean(false);
   private final ImBoolean requestHeightMap = new ImBoolean(true);
   private final ImBoolean requestTerrainMap = new ImBoolean(false);
   private final ImBoolean requestHeightMapController = new ImBoolean(false);
   private final ImBoolean enableChunkedMapRenderer = new ImBoolean(false);
   private final ImBoolean enableHeightMapRenderer = new ImBoolean(true);
   private final ImBoolean colorBasedOnTraversability = new ImBoolean(false);
   private final Stopwatch stopwatch = new Stopwatch();
   private ROS2Helper ros2;
   private ROS2Heartbeat chunkMapRequestHeartbeat;
   private ROS2Heartbeat heightMapRequestHeartbeat;
   private ROS2Heartbeat terrainMapRequestHeartbeat;
   private ROS2Heartbeat heightMapControllerRequestHeartbeat;
   private Mat heightMap;
   private Mat traversabilityScore;
   private HeightMapData latestHeightMapData;
   private TerrainMapData latestTerrainMapData;
   private int cellsPerAxisOfChunks;
   private ROS2Topic<HeightMapMessage> heightMapTopic = PerceptionAPI.HEIGHT_MAP_MESSAGE;
   private ROS2Topic<TerrainMapMessage> terrainMapTopic = PerceptionAPI.TERRAIN_MAP_MESSAGE;

   public RDXROS2HeightMapVisualizer(String title)
   {
      super(title);

      chunkedMapRenderer = new RDXChunkedMapRenderer();

      executorService = MissingThreadTools.newSingleThreadExecutor("Height Map Visualizer Subscription", true, 1);
   }

   public RDXROS2HeightMapVisualizer(String title,
                                     ROS2Topic<HeightMapMessage> heightMapTopic,
                                     ROS2Topic<TerrainMapMessage> terrainMapTopic)
   {
      this(title);
      this.heightMapTopic = heightMapTopic;
      this.terrainMapTopic = terrainMapTopic;
   }

   @Override
   public List<ROS2Topic<?>> getTopics()
   {
      return List.of(heightMapTopic);
   }

   public void setupForImageMessage(ROS2Helper ros2)
   {
      this.ros2 = ros2;
      ros2.subscribeViaCallback(heightMapTopic, this::acceptHeightMapMessage);
      ros2.subscribeViaCallback(terrainMapTopic, this::acceptTerrainMapMessage);
   }

   public void setupForChunkMessage(ROS2Helper ros2)
   {
      this.ros2 = ros2;
      ros2.subscribeViaCallback(PerceptionAPI.CHUNK, this::acceptChunkMessage);
   }

   public void acceptChunkMessage(ChunkMessage chunkMessage)
   {
      if (enableChunkedMapRenderer.get())
      {
         cellsPerAxisOfChunks = chunkMessage.getCellsPerAxis();
         chunkedMapRenderer.addHeightMap(chunkMessage, chunkMessage.getHashCodeOfChunk());
      }
   }

   public void acceptHeightMapMessage(HeightMapMessage heightMapMessage)
   {
      // Even if the height map is publishing, we aren't going to update anything with that data unless it's active
      if (!isActive())
         return;

      executorService.clearQueueAndExecute(() ->
                                           {
                                              HeightMapData heightMapData = HeightMapMessageTools.unpackMessageToHeightMapData(heightMapMessage);
                                              processHeightMapData(heightMapData);
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

      heightMapImageVisualizer.setImage(colorized, PixelFormat.BGR8);
   }

   public void acceptTerrainMapMessage(TerrainMapMessage terrainMapMessage)
   {
      // Even if the height map is publishing, we aren't going to update anything with that data unless it's active
      if (!isActive())
         return;

      if (colorBasedOnTraversability.get())
      {
         executorService.clearQueueAndExecute(() -> processTerrainMapData(terrainMapMessage));
      }

      getFrequency(PerceptionAPI.HEIGHT_MAP_MESSAGE).ping();
   }

   private void processHeightMapData(HeightMapData heightMapData)
   {
      if (heightMap == null)
      {
         heightMap = new Mat(heightMapData.getCellsPerAxis(), heightMapData.getCellsPerAxis(), opencv_core.CV_32FC1);
      }

      latestHeightMapData = heightMapData;
      HeightMapTools.convertHeightMapDataToMat(heightMap, latestHeightMapData);

      // This prevents the rendering from happening too early, it was throwing exceptions
      if (stopwatch.lapElapsed() > 3)
         updateHeightMapImage();
   }

   private void processTerrainMapData(TerrainMapMessage terrainMapMessage)
   {
      latestTerrainMapData = TerrainMapMessageTools.unpackMessage(terrainMapMessage);

      if (traversabilityScore == null)
      {
         traversabilityScore = new Mat(latestHeightMapData.getCellsPerAxis(), latestHeightMapData.getCellsPerAxis(), opencv_core.CV_32FC1);
      }

      float[] traversabilityScoreMap = latestTerrainMapData.getTraversabilityScoreMap();
      FloatBuffer traversabilityScoreBuffer = traversabilityScore.createBuffer();
      traversabilityScoreBuffer.put(traversabilityScoreMap);
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

      if (heightMapRequestHeartbeat != null && ImGui.checkbox(labels.get("Request Height Map"), requestHeightMap))
         heightMapRequestHeartbeat.setAlive(requestHeightMap.get());

      if (chunkMapRequestHeartbeat != null && ImGui.checkbox(labels.get("Request Chunk Map"), requestChunkMap))
         chunkMapRequestHeartbeat.setAlive(requestChunkMap.get());

      if (terrainMapRequestHeartbeat != null && ImGui.checkbox(labels.get("Request Terrain Map"), requestTerrainMap))
         terrainMapRequestHeartbeat.setAlive(requestTerrainMap.get());

      if (heightMapControllerRequestHeartbeat != null && ImGui.checkbox(labels.get("Send Height Map to Controller"), requestHeightMapController))
         heightMapControllerRequestHeartbeat.setAlive(requestHeightMapController.get());

      if (ImGui.collapsingHeader(labels.get("Visualization Options")))
      {
         ImGui.checkbox(labels.get("Enable Height Map"), enableHeightMapRenderer);
         ImGui.checkbox(labels.get("Enable Chunked Map"), enableChunkedMapRenderer);
         ImGui.checkbox(labels.get("Show Traversability"), colorBasedOnTraversability);
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

      if (latestHeightMapData != null && !heightMapRenderer.isHasBeenCreated())
      {
         int numberOfCells = latestHeightMapData.getCellsPerAxis() * latestHeightMapData.getCellsPerAxis();
         heightMapRenderer.create(numberOfCells);
         stopwatch.start();
      }

      if (cellsPerAxisOfChunks > 0)
      {
         chunkedMapRenderer.removeOldRenderers();
         chunkedMapRenderer.create();
      }

      if (enableHeightMapRenderer.get() && heightMapRenderer.isHasBeenCreated())
      {
         // An additional check here to make sure that we have data in the image
         if (heightMap != null && heightMap.ptr(0) != null)
         {
            double heightMapCenterX = latestHeightMapData.getGridCenter().getX();
            double heightMapCenterY = latestHeightMapData.getGridCenter().getY();
            double cellSize = latestHeightMapData.getCellSize();
            int centerIndex = HeightMapTools.computeCenterIndex(latestHeightMapData.getMapSize(), cellSize);
            heightMapRenderer.update(heightMap,
                                     traversabilityScore,
                                     colorBasedOnTraversability.get(),
                                     (float) heightMapCenterX,
                                     (float) heightMapCenterY,
                                     centerIndex,
                                     (float) cellSize);
         }
      }

      if (enableChunkedMapRenderer.get())
      {
         chunkedMapRenderer.update();
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
         if (enableChunkedMapRenderer.get())
         {
            chunkedMapRenderer.getRenderables(renderables, pool);
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
      chunkedMapRenderer.destroy();
      heightMapRenderer.dispose();
   }

   public void setupChunkMapRequestHeartbeat(ROS2Node ros2Node)
   {
      chunkMapRequestHeartbeat = new ROS2Heartbeat(ros2Node, PerceptionAPI.REQUEST_CHUNK_MAP);
      chunkMapRequestHeartbeat.setAlive(requestChunkMap.get());
   }

   public void setupHeightMapRequestHeartbeat(ROS2Node ros2Node)
   {
      heightMapRequestHeartbeat = new ROS2Heartbeat(ros2Node, PerceptionAPI.REQUEST_HEIGHT_MAP);
      heightMapRequestHeartbeat.setAlive(requestHeightMap.get());
   }

   public void setupTerrainMapRequestHeartbeat(ROS2Node ros2Node)
   {
      terrainMapRequestHeartbeat = new ROS2Heartbeat(ros2Node, PerceptionAPI.REQUEST_TERRAIN_MAP);
      terrainMapRequestHeartbeat.setAlive(requestTerrainMap.get());
   }

   public void setupHeightMapControllerRequestHeartbeat(ROS2Node ros2Node)
   {
      heightMapControllerRequestHeartbeat = new ROS2Heartbeat(ros2Node, PerceptionAPI.REQUEST_HEIGHT_MAP_FOR_CONTROLLER);
      heightMapControllerRequestHeartbeat.setAlive(requestHeightMapController.get());
   }

   public HeightMapData getLatestHeightMapData()
   {
      return isActive() ? latestHeightMapData : null;
   }

   public TerrainMapData getLatestTerrainMapData()
   {
      return isActive() ? latestTerrainMapData : null;
   }

   public RDXImageVisualizer getHeightMapImageVisualizer()
   {
      return heightMapImageVisualizer;
   }
}