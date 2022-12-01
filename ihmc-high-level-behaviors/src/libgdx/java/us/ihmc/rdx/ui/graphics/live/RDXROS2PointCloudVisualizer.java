package us.ihmc.rdx.ui.graphics.live;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.type.ImBoolean;
import perception_msgs.msg.dds.FusedSensorHeadPointCloudMessage;
import perception_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import imgui.internal.ImGui;
import imgui.type.ImFloat;
import net.jpountz.lz4.LZ4Factory;
import net.jpountz.lz4.LZ4FastDecompressor;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import us.ihmc.commons.FormattingTools;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.packets.LidarPointCloudCompression;
import us.ihmc.communication.packets.StereoPointCloudCompression;
import us.ihmc.log.LogTools;
import us.ihmc.perception.elements.DiscretizedColoredPointCloud;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.tools.ImPlotIntegerPlot;
import us.ihmc.rdx.ui.visualizers.ImGuiFrequencyPlot;
import us.ihmc.rdx.ui.visualizers.RDXVisualizer;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLIntBuffer;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.nio.BufferOverflowException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.atomic.AtomicReference;

public class RDXROS2PointCloudVisualizer extends RDXVisualizer implements RenderableProvider
{
   private final ROS2Node ros2Node;
   private final ROS2Topic<?> topic;
   private IHMCROS2Callback<?> ros2Callback = null;
   private final ImGuiFrequencyPlot frequencyPlot = new ImGuiFrequencyPlot();
   private final ImPlotIntegerPlot segmentIndexPlot = new ImPlotIntegerPlot("Segment", 30);
   private final ImFloat pointSize = new ImFloat(0.01f);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean subscribed = new ImBoolean(true);
   private final RDXPointCloudRenderer pointCloudRenderer = new RDXPointCloudRenderer();
   private int pointsPerSegment;
   private int numberOfSegments;
   private int totalNumberOfPoints;
   private final ResettableExceptionHandlingExecutorService threadQueue;
   private final LZ4FastDecompressor lz4Decompressor = LZ4Factory.nativeInstance().fastDecompressor();
   private ByteBuffer decompressionInputDirectBuffer;
   private final AtomicReference<FusedSensorHeadPointCloudMessage> latestFusedSensorHeadPointCloudMessageReference = new AtomicReference<>(null);
   private final AtomicReference<LidarScanMessage> latestLidarScanMessageReference = new AtomicReference<>(null);
   private final AtomicReference<StereoVisionPointCloudMessage> latestStereoVisionMessageReference = new AtomicReference<>(null);
   private final Color color = new Color();
   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private _cl_kernel unpackPointCloudKernel;
   private OpenCLFloatBuffer pointCloudVertexBuffer;
   private OpenCLIntBuffer decompressedOpenCLIntBuffer;
   private OpenCLFloatBuffer parametersOpenCLFloatBuffer;
   private String messageSizeString;

   public RDXROS2PointCloudVisualizer(String title, ROS2Node ros2Node, ROS2Topic<?> topic)
   {
      super(title + " (ROS 2)");
      this.ros2Node = ros2Node;
      this.topic = topic;
      threadQueue = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

      setSubscribed(subscribed.get());
   }

   private void subscribe()
   {
      subscribed.set(true);
      if (topic.getType().equals(LidarScanMessage.class))
      {
         ros2Callback = new IHMCROS2Callback<>(ros2Node, topic.withType(LidarScanMessage.class), this::queueRenderLidarScan);
      }
      else if (topic.getType().equals(StereoVisionPointCloudMessage.class))
      {
         ros2Callback = new IHMCROS2Callback<>(ros2Node, topic.withType(StereoVisionPointCloudMessage.class), this::queueRenderStereoVisionPointCloud);
      }
      else if (topic.getType().equals(FusedSensorHeadPointCloudMessage.class))
      {
         ros2Callback = new IHMCROS2Callback<>(ros2Node,
                                               topic.withType(FusedSensorHeadPointCloudMessage.class),
                                               ROS2QosProfile.BEST_EFFORT(),
                                               this::queueRenderFusedSensorHeadPointCloud);
      }
   }

   private void queueRenderStereoVisionPointCloud(StereoVisionPointCloudMessage message)
   {
      frequencyPlot.recordEvent();
      // TODO: Possibly decompress on a thread here
      // TODO: threadQueue.clearQueueAndExecute(() ->
      latestStereoVisionMessageReference.set(message);
   }

   private void queueRenderLidarScan(LidarScanMessage message)
   {
      frequencyPlot.recordEvent();
      latestLidarScanMessageReference.set(message);
   }

   private void queueRenderFusedSensorHeadPointCloud(FusedSensorHeadPointCloudMessage message)
   {
      frequencyPlot.recordEvent();
      latestFusedSensorHeadPointCloudMessageReference.set(message);
   }

   @Override
   public void create()
   {
      super.create();

      openCLManager = new OpenCLManager();
      openCLManager.create();
      openCLProgram = openCLManager.loadProgram("FusedSensorPointCloudSubscriberVisualizer");
      unpackPointCloudKernel = openCLManager.createKernel(openCLProgram, "unpackPointCloud");

      parametersOpenCLFloatBuffer = new OpenCLFloatBuffer(4);
      parametersOpenCLFloatBuffer.createOpenCLBufferObject(openCLManager);
   }

   @Override
   public void update()
   {
      super.update();

      if (subscribed.get() && isActive())
      {
         FusedSensorHeadPointCloudMessage fusedMessage = latestFusedSensorHeadPointCloudMessageReference.getAndSet(null);
         if (fusedMessage != null)
         {
            segmentIndexPlot.addValue((int) fusedMessage.getSegmentIndex());

            if (pointsPerSegment != fusedMessage.getPointsPerSegment())
            {
               pointsPerSegment = fusedMessage.getPointsPerSegment();
               numberOfSegments = (int) fusedMessage.getNumberOfSegments();
               totalNumberOfPoints = pointsPerSegment * numberOfSegments;
               int bytesPerSegment = pointsPerSegment * DiscretizedColoredPointCloud.DISCRETE_BYTES_PER_POINT;
               String kilobytes = FormattingTools.getFormattedDecimal1D((double) bytesPerSegment / 1000.0);
               messageSizeString = String.format("Message size: %s KB", kilobytes);
               pointCloudRenderer.create(pointsPerSegment, numberOfSegments);
               decompressionInputDirectBuffer = ByteBuffer.allocateDirect(bytesPerSegment);
               decompressionInputDirectBuffer.order(ByteOrder.nativeOrder());
               if (decompressedOpenCLIntBuffer != null)
                  decompressedOpenCLIntBuffer.destroy(openCLManager);
               decompressedOpenCLIntBuffer = new OpenCLIntBuffer(pointsPerSegment * DiscretizedColoredPointCloud.DISCRETE_INTS_PER_POINT);
               decompressedOpenCLIntBuffer.createOpenCLBufferObject(openCLManager);
               if (pointCloudVertexBuffer != null)
                  pointCloudVertexBuffer.destroy(openCLManager);
               pointCloudVertexBuffer = new OpenCLFloatBuffer(totalNumberOfPoints * RDXPointCloudRenderer.FLOATS_PER_VERTEX,
                                                              pointCloudRenderer.getVertexBuffer());
               pointCloudVertexBuffer.createOpenCLBufferObject(openCLManager);
               LogTools.info("Allocated new buffers. {} points per segment. {} segments.", pointsPerSegment, numberOfSegments);
            }

            if (fusedMessage.getSegmentIndex() == pointCloudRenderer.getCurrentSegmentIndex())
            {
               decompressionInputDirectBuffer.rewind();
               int numberOfBytes = fusedMessage.getScan().size();
               decompressionInputDirectBuffer.limit(numberOfBytes);
               for (int i = 0; i < numberOfBytes; i++)
               {
                  decompressionInputDirectBuffer.put(fusedMessage.getScan().get(i));
               }
               decompressionInputDirectBuffer.flip();
               decompressedOpenCLIntBuffer.getBackingDirectByteBuffer().rewind();
               // TODO: Look at using bytedeco LZ4 1.9.X, which is supposed to be 12% faster than 1.8.X
               lz4Decompressor.decompress(decompressionInputDirectBuffer, decompressedOpenCLIntBuffer.getBackingDirectByteBuffer());
               decompressedOpenCLIntBuffer.getBackingDirectByteBuffer().rewind();

               parametersOpenCLFloatBuffer.getBytedecoFloatBufferPointer().put(0, pointCloudRenderer.getCurrentSegmentIndex());
               parametersOpenCLFloatBuffer.getBytedecoFloatBufferPointer().put(1, pointSize.get());
               parametersOpenCLFloatBuffer.getBytedecoFloatBufferPointer().put(2, pointsPerSegment);
               parametersOpenCLFloatBuffer.getBytedecoFloatBufferPointer().put(3, (float) DiscretizedColoredPointCloud.DISCRETE_RESOLUTION);

               parametersOpenCLFloatBuffer.writeOpenCLBufferObject(openCLManager);
               decompressedOpenCLIntBuffer.writeOpenCLBufferObject(openCLManager);
               pointCloudRenderer.updateMeshFastestBeforeKernel();
               pointCloudVertexBuffer.syncWithBackingBuffer();

               openCLManager.setKernelArgument(unpackPointCloudKernel, 0, parametersOpenCLFloatBuffer.getOpenCLBufferObject());
               openCLManager.setKernelArgument(unpackPointCloudKernel, 1, decompressedOpenCLIntBuffer.getOpenCLBufferObject());
               openCLManager.setKernelArgument(unpackPointCloudKernel, 2, pointCloudVertexBuffer.getOpenCLBufferObject());
               openCLManager.execute1D(unpackPointCloudKernel, pointsPerSegment);
               pointCloudVertexBuffer.readOpenCLBufferObject(openCLManager);

               pointCloudRenderer.updateMeshFastestAfterKernel();
            }
         }

         LidarScanMessage latestLidarScanMessage = latestLidarScanMessageReference.getAndSet(null);
         if (latestLidarScanMessage != null)
         {
            int numberOfScanPoints = latestLidarScanMessage.getNumberOfPoints();
            pointCloudRenderer.updateMeshFastest(xyzRGBASizeFloatBuffer ->
            {
               float size = pointSize.get();
               LidarPointCloudCompression.decompressPointCloud(latestLidarScanMessage.getScan(), numberOfScanPoints, (i, x, y, z) ->
               {
                  xyzRGBASizeFloatBuffer.put((float) x);
                  xyzRGBASizeFloatBuffer.put((float) y);
                  xyzRGBASizeFloatBuffer.put((float) z);
                  xyzRGBASizeFloatBuffer.put(color.r);
                  xyzRGBASizeFloatBuffer.put(color.g);
                  xyzRGBASizeFloatBuffer.put(color.b);
                  xyzRGBASizeFloatBuffer.put(color.a);
                  xyzRGBASizeFloatBuffer.put(size);
               });

               return numberOfScanPoints;
            });
         }

         StereoVisionPointCloudMessage latestStereoVisionMessage = latestStereoVisionMessageReference.getAndSet(null);
         if (latestStereoVisionMessage != null)
         {
            float size = pointSize.get();
            pointCloudRenderer.updateMeshFastest(xyzRGBASizeFloatBuffer ->
            {
               StereoPointCloudCompression.decompressPointCloud(latestStereoVisionMessage, (x, y, z) ->
               {
                  try
                  {
                     xyzRGBASizeFloatBuffer.put((float) x);
                     xyzRGBASizeFloatBuffer.put((float) y);
                     xyzRGBASizeFloatBuffer.put((float) z);
                     xyzRGBASizeFloatBuffer.put(color.r);
                     xyzRGBASizeFloatBuffer.put(color.g);
                     xyzRGBASizeFloatBuffer.put(color.b);
                     xyzRGBASizeFloatBuffer.put(color.a);
                     xyzRGBASizeFloatBuffer.put(size);
                  }
                  catch (BufferOverflowException e)
                  {
                     e.printStackTrace();
                  }
               });

               return latestStereoVisionMessage.getNumberOfPoints();
            });
         }
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      if (ImGui.checkbox(labels.getHidden(getTitle() + "Subscribed"), subscribed))
      {
         setSubscribed(subscribed.get());
      }
      ImGuiTools.previousWidgetTooltip("Subscribed");
      ImGui.sameLine();
      super.renderImGuiWidgets();
      ImGui.text(topic.getName());
      ImGui.sameLine();
      ImGui.pushItemWidth(30.0f);
      ImGui.dragFloat(labels.get("Size"), pointSize.getData(), 0.001f, 0.0005f, 0.1f);
      ImGui.popItemWidth();
      if (messageSizeString != null)
      {
         ImGui.sameLine();
         ImGui.text(messageSizeString);
      }
      frequencyPlot.renderImGuiWidgets();
      segmentIndexPlot.renderImGuiWidgets();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (isActive())
         pointCloudRenderer.getRenderables(renderables, pool);
   }

   public void setSubscribed(boolean subscribed)
   {
      if (subscribed && ros2Callback == null)
      {
         subscribe();
      }
      else if (!subscribed && ros2Callback != null)
      {
         unsubscribe();
      }
   }

   private void unsubscribe()
   {
      subscribed.set(false);
      ros2Callback.destroy();
      ros2Callback = null;
   }

   public boolean isSubscribed()
   {
      return subscribed.get();
   }
}
