package us.ihmc.gdx.ui.graphics.live;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import imgui.internal.ImGui;
import net.jpountz.lz4.LZ4Factory;
import net.jpountz.lz4.LZ4FastDecompressor;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.gdx.GDXPointCloudRenderer;
import us.ihmc.gdx.ui.visualizers.ImGuiFrequencyPlot;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXVisualizer;
import us.ihmc.robotEnvironmentAwareness.communication.converters.StereoPointCloudCompression;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.nio.ByteBuffer;
import java.util.concurrent.atomic.AtomicReference;

public class GDXROS2PointCloudVisualizer extends ImGuiGDXVisualizer implements RenderableProvider
{
   private final ROS2Node ros2Node;
   private final ROS2Topic<?> topic;
   private final ImGuiFrequencyPlot frequencyPlot = new ImGuiFrequencyPlot();
   private final GDXPointCloudRenderer pointCloudRenderer = new GDXPointCloudRenderer();
   private final int pointsPerSegment;
   private final int numberOfSegments;
   private final ResettableExceptionHandlingExecutorService threadQueue;
   private final LZ4FastDecompressor lz4Decompressor = LZ4Factory.nativeInstance().fastDecompressor();
   private final ByteBuffer decompressionInputDirectBuffer;
   private final ByteBuffer decompressionOutputDirectBuffer;
   private final int inputBytesPerPoint = 3 * Integer.BYTES;
   private final AtomicReference<LidarScanMessage> latestLidarScanMessageReference = new AtomicReference<>(null);

   private Point3D32[] points;
   private final RecyclingArrayList<Point3D32> pointsToRender = new RecyclingArrayList<>(Point3D32::new);

   public GDXROS2PointCloudVisualizer(String title, ROS2Node ros2Node, ROS2Topic<?> topic)
   {
      this(title, ros2Node, topic, 500000, 1);
   }

   public GDXROS2PointCloudVisualizer(String title, ROS2Node ros2Node, ROS2Topic<?> topic, int pointsPerSegment, int numberOfSegments)
   {
      super(title);
      this.ros2Node = ros2Node;
      this.topic = topic;
      this.pointsPerSegment = pointsPerSegment;
      this.numberOfSegments = numberOfSegments;
      threadQueue = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
      decompressionInputDirectBuffer = ByteBuffer.allocateDirect(pointsPerSegment * inputBytesPerPoint);
      decompressionOutputDirectBuffer = ByteBuffer.allocateDirect(pointsPerSegment * inputBytesPerPoint);

      if (topic.getType().equals(LidarScanMessage.class))
      {
         new IHMCROS2Callback<>(ros2Node, topic.withType(LidarScanMessage.class), this::queueRenderLidarScan);
      }
      else if (topic.getType().equals(StereoVisionPointCloudMessage.class))
      {
         new IHMCROS2Callback<>(ros2Node, topic.withType(StereoVisionPointCloudMessage.class), this::queueRenderStereoVisionPointCloud);
      }
   }

   private void queueRenderStereoVisionPointCloud(StereoVisionPointCloudMessage message)
   {
      frequencyPlot.onRecievedMessage();
      if (isActive())
      {
         threadQueue.clearQueueAndExecute(() ->
         {
            points = StereoPointCloudCompression.decompressPointCloudToArray32(message);
            //         int[] colors = PointCloudCompression.decompressColorsToIntArray(message);
         });
      }
   }

   private void queueRenderLidarScan(LidarScanMessage message)
   {
      frequencyPlot.onRecievedMessage();
      latestLidarScanMessageReference.set(message);
   }

   @Override
   public void create()
   {
      super.create();
      pointCloudRenderer.create(pointsPerSegment, numberOfSegments);
   }

   @Override
   public void update()
   {
      super.update();
      if (isActive())
      {
         LidarScanMessage latestLidarScanMessage = latestLidarScanMessageReference.getAndSet(null);

         if (latestLidarScanMessage != null)
         {
            decompressionInputDirectBuffer.rewind();
            int numberOfBytes = latestLidarScanMessage.getScan().size();
            decompressionInputDirectBuffer.limit(numberOfBytes);
            for (int i = 0; i < numberOfBytes; i++)
            {
               decompressionInputDirectBuffer.put(latestLidarScanMessage.getScan().get(i));
            }
            decompressionInputDirectBuffer.flip();
            decompressionOutputDirectBuffer.clear();
            lz4Decompressor.decompress(decompressionInputDirectBuffer, decompressionOutputDirectBuffer);
            decompressionOutputDirectBuffer.rewind();

//            decompressionInputDirectBuffer.rewind(); // TEMP REMOVE
            pointCloudRenderer.updateMeshFastest(xyzRGBASizeFloatBuffer ->
            {
               for (int i = 0; i < pointsPerSegment; i++)
               {
                  float x = decompressionOutputDirectBuffer.getInt() * 0.003f;
                  float y = decompressionOutputDirectBuffer.getInt() * 0.003f;
                  float z = decompressionOutputDirectBuffer.getInt() * 0.003f;
//                  float x = decompressionInputDirectBuffer.getFloat();
//                  float y = decompressionInputDirectBuffer.getFloat();
//                  float z = decompressionInputDirectBuffer.getFloat();
                  xyzRGBASizeFloatBuffer.put(x);
                  xyzRGBASizeFloatBuffer.put(y);
                  xyzRGBASizeFloatBuffer.put(z);
                  xyzRGBASizeFloatBuffer.put(1.0f);
                  xyzRGBASizeFloatBuffer.put(1.0f);
                  xyzRGBASizeFloatBuffer.put(1.0f);
                  xyzRGBASizeFloatBuffer.put(1.0f);
                  xyzRGBASizeFloatBuffer.put(0.01f);
               }
               return pointsPerSegment;
            }, (int) latestLidarScanMessage.getSequenceId());
         }
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
      ImGui.text(topic.getName());
      frequencyPlot.renderImGuiWidgets();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (isActive())
         pointCloudRenderer.getRenderables(renderables, pool);
   }
}
