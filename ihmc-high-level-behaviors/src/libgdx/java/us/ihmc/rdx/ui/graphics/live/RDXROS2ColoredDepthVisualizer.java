package us.ihmc.rdx.ui.graphics.live;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.VideoPacket;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.tools.ImPlotIntegerPlot;
import us.ihmc.rdx.ui.visualizers.ImGuiFrequencyPlot;
import us.ihmc.rdx.ui.visualizers.RDXVisualizer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

import java.nio.ByteBuffer;
import java.util.concurrent.atomic.AtomicReference;

import static org.bytedeco.opencv.global.opencv_highgui.imshow;
import static org.bytedeco.opencv.global.opencv_highgui.waitKeyEx;

public class RDXROS2ColoredDepthVisualizer extends RDXVisualizer implements RenderableProvider
{
   private float depthToMetersScalar = 2.500000118743628E-4f;

   private byte[] heapArrayDepth = new byte[25000000];
   private final ROS2Node ros2Node;
   private final ROS2Topic<?> depthTopic;
   private final ROS2Topic<?> colorTopic;
   private IHMCROS2Callback<?> ros2Callback = null;
   private final ImGuiFrequencyPlot frequencyPlot = new ImGuiFrequencyPlot();
   private final ImPlotIntegerPlot segmentIndexPlot = new ImPlotIntegerPlot("Segment", 30);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean subscribed = new ImBoolean(true);
   private final RDXPointCloudRenderer pointCloudRenderer = new RDXPointCloudRenderer();
   private ByteBuffer decompressionInputDirectBuffer;
   private final Color color = new Color();
   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private _cl_kernel unpackPointCloudKernel;
   private String messageSizeString;

   private AtomicReference<VideoPacket> colorPacketReference = new AtomicReference<>(null);;
   private AtomicReference<VideoPacket> depthPacketReference = new AtomicReference<>(null);

   public RDXROS2ColoredDepthVisualizer(String title, ROS2Node ros2Node, ROS2Topic<?> depthTopic, ROS2Topic<?> colorTopic)
   {
      super(title + " (ROS 2)");
      this.ros2Node = ros2Node;
      this.depthTopic = depthTopic;
      this.colorTopic = colorTopic;

      setSubscribed(subscribed.get());
   }

   private void subscribe()
   {
      subscribed.set(true);
      ros2Callback = new IHMCROS2Callback<>(ros2Node, ROS2Tools.L515_DEPTH, this::depthCallback);
      ros2Callback = new IHMCROS2Callback<>(ros2Node, ROS2Tools.L515_VIDEO, this::colorCallback);
   }

   private void colorCallback(VideoPacket packet)
   {
      colorPacketReference.set(packet);
   }

   private void depthCallback(VideoPacket packet)
   {
      LogTools.info("Time Now: {}", System.currentTimeMillis());
      depthPacketReference.set(packet);
   }

   @Override
   public void create()
   {
      super.create();

      openCLManager = new OpenCLManager();
      openCLManager.create();
      openCLProgram = openCLManager.loadProgram("FusedSensorPointCloudSubscriberVisualizer");
      unpackPointCloudKernel = openCLManager.createKernel(openCLProgram, "unpackPointCloud");
   }

   @Override
   public void update()
   {
      super.update();

      if (subscribed.get() && isActive())
      {
         if (depthPacketReference.get() != null)
         {
            VideoPacket depthPacket = this.depthPacketReference.getAndSet(null);

            Mat depthImage = new Mat(depthPacket.getImageHeight(), depthPacket.getImageWidth(), opencv_core.CV_16UC1);
            depthPacket.getData().toArray(heapArrayDepth, 0, depthPacket.getData().size());
            BytedecoOpenCVTools.decompressDepthPNG(heapArrayDepth, depthImage);

            Mat depthInMeters = new Mat(depthPacket.getImageHeight(), depthPacket.getImageWidth(), opencv_core.CV_32FC1);
            depthImage.convertTo(depthInMeters, opencv_core.CV_32FC1, depthToMetersScalar, 0.0);




            /* For Display Only */
            Mat displayDepth = new Mat(depthPacket.getImageHeight(), depthPacket.getImageWidth(), opencv_core.CV_8UC1);
            Mat finalDisplayDepth = new Mat(depthPacket.getImageHeight(), depthPacket.getImageWidth(), opencv_core.CV_8UC3);
            BytedecoOpenCVTools.clampTo8BitUnsignedChar(depthImage, displayDepth, 0.0, 255.0);
            BytedecoOpenCVTools.convert8BitGrayTo8BitRGBA(displayDepth, finalDisplayDepth);

            imshow("/l515/depth", finalDisplayDepth);
            int code = waitKeyEx(1);
            if (code == 113)
            {
               System.exit(0);
            }
            /* Display Ends */

            //long end_decompress = System.nanoTime();
            //
            //LogTools.info("Loading Time: {} ms", (end_load - begin_load) / 1e6);
            //LogTools.info("Decompression Time: {} ms",(end_decompress - begin_decompress) / 1e6f);
            //
            //if (pointsPerSegment != fusedMessage.getPointsPerSegment())
            //{
            //   pointsPerSegment = fusedMessage.getPointsPerSegment();
            //   numberOfSegments = (int) fusedMessage.getNumberOfSegments();
            //   totalNumberOfPoints = pointsPerSegment * numberOfSegments;
            //   int bytesPerSegment = pointsPerSegment * DiscretizedColoredPointCloud.DISCRETE_BYTES_PER_POINT;
            //   String kilobytes = FormattingTools.getFormattedDecimal1D((double) bytesPerSegment / 1000.0);
            //   messageSizeString = String.format("Message size: %s KB", kilobytes);
            //   pointCloudRenderer.create(pointsPerSegment, numberOfSegments);
            //   decompressionInputDirectBuffer = ByteBuffer.allocateDirect(bytesPerSegment);
            //   decompressionInputDirectBuffer.order(ByteOrder.nativeOrder());
            //   if (decompressedOpenCLIntBuffer != null)
            //      decompressedOpenCLIntBuffer.destroy(openCLManager);
            //   decompressedOpenCLIntBuffer = new OpenCLIntBuffer(pointsPerSegment * DiscretizedColoredPointCloud.DISCRETE_INTS_PER_POINT);
            //   decompressedOpenCLIntBuffer.createOpenCLBufferObject(openCLManager);
            //   if (pointCloudVertexBuffer != null)
            //      pointCloudVertexBuffer.destroy(openCLManager);
            //   pointCloudVertexBuffer = new OpenCLFloatBuffer(totalNumberOfPoints * RDXPointCloudRenderer.FLOATS_PER_VERTEX,
            //                                                  pointCloudRenderer.getVertexBuffer());
            //   pointCloudVertexBuffer.createOpenCLBufferObject(openCLManager);
            //   LogTools.info("Allocated new buffers. {} points per segment. {} segments.", pointsPerSegment, numberOfSegments);
            //}
            //
            //if (fusedMessage.getSegmentIndex() == pointCloudRenderer.getCurrentSegmentIndex())
            //{
            //   decompressionInputDirectBuffer.rewind();
            //   int numberOfBytes = fusedMessage.getScan().size();
            //   decompressionInputDirectBuffer.limit(numberOfBytes);
            //   for (int i = 0; i < numberOfBytes; i++)
            //   {
            //      decompressionInputDirectBuffer.put(fusedMessage.getScan().get(i));
            //   }
            //   decompressionInputDirectBuffer.flip();
            //   decompressedOpenCLIntBuffer.getBackingDirectByteBuffer().rewind();
            //   // TODO: Look at using bytedeco LZ4 1.9.X, which is supposed to be 12% faster than 1.8.X
            //   lz4Decompressor.decompress(decompressionInputDirectBuffer, decompressedOpenCLIntBuffer.getBackingDirectByteBuffer());
            //   decompressedOpenCLIntBuffer.getBackingDirectByteBuffer().rewind();
            //
            //   parametersOpenCLFloatBuffer.getBytedecoFloatBufferPointer().put(0, pointCloudRenderer.getCurrentSegmentIndex());
            //   parametersOpenCLFloatBuffer.getBytedecoFloatBufferPointer().put(1, pointSize.get());
            //   parametersOpenCLFloatBuffer.getBytedecoFloatBufferPointer().put(2, pointsPerSegment);
            //   parametersOpenCLFloatBuffer.getBytedecoFloatBufferPointer().put(3, (float) DiscretizedColoredPointCloud.DISCRETE_RESOLUTION);
            //
            //   parametersOpenCLFloatBuffer.writeOpenCLBufferObject(openCLManager);
            //   decompressedOpenCLIntBuffer.writeOpenCLBufferObject(openCLManager);
            //   pointCloudRenderer.updateMeshFastestBeforeKernel();
            //   pointCloudVertexBuffer.syncWithBackingBuffer();
            //
            //   openCLManager.setKernelArgument(unpackPointCloudKernel, 0, parametersOpenCLFloatBuffer.getOpenCLBufferObject());
            //   openCLManager.setKernelArgument(unpackPointCloudKernel, 1, decompressedOpenCLIntBuffer.getOpenCLBufferObject());
            //   openCLManager.setKernelArgument(unpackPointCloudKernel, 2, pointCloudVertexBuffer.getOpenCLBufferObject());
            //   openCLManager.execute1D(unpackPointCloudKernel, pointsPerSegment);
            //   pointCloudVertexBuffer.readOpenCLBufferObject(openCLManager);
            //
            //   pointCloudRenderer.updateMeshFastestAfterKernel();
            //}
         }

         //LidarScanMessage latestLidarScanMessage = latestLidarScanMessageReference.getAndSet(null);
         //if (latestLidarScanMessage != null)
         //{
         //   int numberOfScanPoints = latestLidarScanMessage.getNumberOfPoints();
         //   pointCloudRenderer.updateMeshFastest(xyzRGBASizeFloatBuffer ->
         //                                        {
         //                                           float size = pointSize.get();
         //                                           LidarPointCloudCompression.decompressPointCloud(latestLidarScanMessage.getScan(),
         //                                                                                           numberOfScanPoints,
         //                                                                                           (i, x, y, z) ->
         //                                                                                           {
         //                                                                                              xyzRGBASizeFloatBuffer.put((float) x);
         //                                                                                              xyzRGBASizeFloatBuffer.put((float) y);
         //                                                                                              xyzRGBASizeFloatBuffer.put((float) z);
         //                                                                                              xyzRGBASizeFloatBuffer.put(color.r);
         //                                                                                              xyzRGBASizeFloatBuffer.put(color.g);
         //                                                                                              xyzRGBASizeFloatBuffer.put(color.b);
         //                                                                                              xyzRGBASizeFloatBuffer.put(color.a);
         //                                                                                              xyzRGBASizeFloatBuffer.put(size);
         //                                                                                           });
         //
         //                                           return numberOfScanPoints;
         //                                        });
         //}
         //
         //StereoVisionPointCloudMessage latestStereoVisionMessage = latestStereoVisionMessageReference.getAndSet(null);
         //if (latestStereoVisionMessage != null)
         //{
         //   float size = pointSize.get();
         //   pointCloudRenderer.updateMeshFastest(xyzRGBASizeFloatBuffer ->
         //                                        {
         //                                           StereoPointCloudCompression.decompressPointCloud(latestStereoVisionMessage, (x, y, z) ->
         //                                           {
         //                                              try
         //                                              {
         //                                                 xyzRGBASizeFloatBuffer.put((float) x);
         //                                                 xyzRGBASizeFloatBuffer.put((float) y);
         //                                                 xyzRGBASizeFloatBuffer.put((float) z);
         //                                                 xyzRGBASizeFloatBuffer.put(color.r);
         //                                                 xyzRGBASizeFloatBuffer.put(color.g);
         //                                                 xyzRGBASizeFloatBuffer.put(color.b);
         //                                                 xyzRGBASizeFloatBuffer.put(color.a);
         //                                                 xyzRGBASizeFloatBuffer.put(size);
         //                                              }
         //                                              catch (BufferOverflowException e)
         //                                              {
         //                                                 e.printStackTrace();
         //                                              }
         //                                           });
         //
         //                                           return latestStereoVisionMessage.getNumberOfPoints();
         //                                        });
         //}
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
      ImGui.text(depthTopic.getName());
      ImGui.text(colorTopic.getName());
      ImGui.sameLine();
      ImGui.pushItemWidth(30.0f);
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
