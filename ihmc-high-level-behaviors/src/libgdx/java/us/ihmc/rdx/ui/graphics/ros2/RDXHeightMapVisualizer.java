package us.ihmc.rdx.ui.graphics.ros2;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Heartbeat;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.tools.NativeMemoryTools;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.rdx.RDXHeightMapRenderer;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.visualizers.ImGuiFrequencyPlot;
import us.ihmc.rdx.ui.visualizers.RDXVisualizer;
import us.ihmc.rdx.visualizers.RDXGridMapGraphic;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.nio.ByteBuffer;
import java.util.Set;

public class RDXHeightMapVisualizer extends RDXVisualizer
{
   private final RDXHeightMapRenderer heightMapRenderer = new RDXHeightMapRenderer();
   private final RDXGridMapGraphic gridMapGraphic = new RDXGridMapGraphic();
   private final ResettableExceptionHandlingExecutorService executorService;
   private final ImGuiFrequencyPlot frequencyPlot = new ImGuiFrequencyPlot();
   private final ImBoolean inPaintHeight = new ImBoolean(false);
   private final ImBoolean renderGroundPlane = new ImBoolean(false);
   private final ImBoolean renderGroundCells = new ImBoolean(false);
   private final ImBoolean heightMapActive = new ImBoolean(false);

   private Mat heightMapImage;
   private Mat compressedBytesMat;
   private ByteBuffer incomingCompressedImageBuffer;
   private BytePointer incomingCompressedImageBytePointer;
   
   private ROS2Heartbeat activeHeartbeat;

   public RDXHeightMapVisualizer()
   {
      super("Height Map");

      boolean daemon = true;
      int queueSize = 1;
      executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), daemon, queueSize);
   }

   public void setupForHeightMapMessage(ROS2PublishSubscribeAPI ros2)
   {
      ros2.subscribeViaCallback(PerceptionAPI.HEIGHT_MAP_OUTPUT, this::acceptHeightMapMessage);
      activeHeartbeat = new ROS2Heartbeat(ros2, PerceptionAPI.PUBLISH_HEIGHT_MAP);
   }

   public void setupForImageMessage(ROS2PublishSubscribeAPI ros2)
   {
      ros2.subscribeViaCallback(PerceptionAPI.HEIGHT_MAP_GLOBAL, this::acceptImageMessage);
      activeHeartbeat = new ROS2Heartbeat(ros2, PerceptionAPI.PUBLISH_HEIGHT_MAP);
   }

   @Override
   public void create()
   {
      super.create();
      heightMapRenderer.create(501 * 501);
   }

   public void acceptHeightMapMessage(HeightMapMessage heightMapMessage)
   {
      frequencyPlot.recordEvent();
      if (isActive())
      {
         executorService.clearQueueAndExecute(() ->
         {
            gridMapGraphic.setInPaintHeight(inPaintHeight.get());
            gridMapGraphic.setRenderGroundPlane(renderGroundPlane.get());
            gridMapGraphic.setRenderGroundCells(renderGroundCells.get());
            gridMapGraphic.generateMeshesAsync(heightMapMessage);
         });
      }
   }

   public void acceptImageMessage(ImageMessage imageMessage)
   {
      frequencyPlot.recordEvent();
      if (isActive())
      {
         executorService.clearQueueAndExecute(() ->
         {
            int numberOfBytes = imageMessage.getData().size();

            if (heightMapImage == null)
            {
               heightMapImage = new Mat(imageMessage.getImageHeight(), imageMessage.getImageWidth(), opencv_core.CV_16UC1);
               compressedBytesMat = new Mat(1, 1, opencv_core.CV_8UC1);
               incomingCompressedImageBuffer = NativeMemoryTools.allocate(numberOfBytes * 2);
               incomingCompressedImageBytePointer = new BytePointer(incomingCompressedImageBuffer);
            }

            incomingCompressedImageBuffer.rewind();
            incomingCompressedImageBuffer.limit(numberOfBytes);
            for (int i = 0; i < numberOfBytes; i++)
            {
               incomingCompressedImageBuffer.put(imageMessage.getData().get(i));
            }
            incomingCompressedImageBuffer.flip();

            compressedBytesMat.cols(numberOfBytes);
            compressedBytesMat.data(incomingCompressedImageBytePointer);

            // Decompress the height map image
            opencv_imgcodecs.imdecode(compressedBytesMat, opencv_imgcodecs.IMREAD_UNCHANGED, heightMapImage);

            RigidBodyTransform transform = new RigidBodyTransform(imageMessage.getOrientation(), imageMessage.getPosition());

            // Update the height map renderer with the new image
            heightMapRenderer.update(transform, heightMapImage.ptr(0), heightMapImage.rows() / 2, 0.02f);

            PerceptionDebugTools.displayDepth("Received Global Height Map", heightMapImage, 1);
         });
      }
   }

   @Override
   public void setActive(boolean active)
   {
      super.setActive(active);
      if (!active)
      {
         executorService.interruptAndReset();
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();

      ImGui.checkbox("Height map active", heightMapActive);

      ImGui.checkbox("In Paint Height", inPaintHeight);
      ImGui.checkbox("Render Ground Plane", renderGroundPlane);
      ImGui.checkbox("Render Ground Cells", renderGroundCells);

      if (!isActive())
      {
         executorService.interruptAndReset();
      }
      frequencyPlot.renderImGuiWidgets();
   }

   @Override
   public void update()
   {
      super.update();

      if (activeHeartbeat != null)
      {
         activeHeartbeat.setAlive(heightMapActive.get());
      }

      boolean isActive = isActive();
      if (isActive)
      {
         gridMapGraphic.update();
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (isActive() && sceneLevelCheck(sceneLevels))
      {
         gridMapGraphic.getRenderables(renderables, pool);
         heightMapRenderer.getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      if (activeHeartbeat != null)
         activeHeartbeat.destroy();
      gridMapGraphic.destroy();
   }
}
