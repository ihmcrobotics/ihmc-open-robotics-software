package us.ihmc.rdx.ui.graphics.live;

import boofcv.struct.calib.CameraPinholeBrown;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.*;
import us.ihmc.perception.tools.NativeMemoryTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.tools.ImPlotDoublePlot;
import us.ihmc.rdx.ui.tools.ImPlotFrequencyPlot;
import us.ihmc.rdx.ui.visualizers.RDXVisualizer;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.string.StringTools;

import java.nio.ByteBuffer;

public class RDXROS2ColoredDepthVisualizer extends RDXVisualizer implements RenderableProvider
{
   private final static int TOTAL_NUMBER_OF_PARAMETERS = 24;

   private final static int FLOATS_PER_POINT = 8;
   private final static int BYTES_PER_POINT = FLOATS_PER_POINT * 4;

   private final float FOCAL_LENGTH_COLOR = 0.00254f;
   private final float CMOS_WIDTH_COLOR = 0.0036894f;
   private final float CMOS_HEIGHT_COLOR = 0.0020753f;

   private final CameraPinholeBrown depthCameraInstrinsics = new CameraPinholeBrown();
   private final CameraPinholeBrown colorCameraInstrinsics = new CameraPinholeBrown();
   private final RDXPointCloudRenderer pointCloudRenderer = new RDXPointCloudRenderer();

   private final Object imageMessagesSyncObject = new Object();
   private final ImageMessage colorImageMessage = new ImageMessage();
   private final ImageMessage depthImageMessage = new ImageMessage();
   private final SampleInfo colorSampleInfo = new SampleInfo();
   private final SampleInfo depthSampleInfo = new SampleInfo();

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean subscribed = new ImBoolean(true);
   private final String titleBeforeAdditions;
   private final PubSubImplementation pubSubImplementation;
   private RealtimeROS2Node realtimeROS2Node;
   private final ROS2Topic<ImageMessage> depthTopic;
   private final ROS2Topic<ImageMessage> colorTopic;
   private final ImPlotFrequencyPlot colorFrequencyPlot = new ImPlotFrequencyPlot("Color Hz", 30);
   private final ImPlotFrequencyPlot depthFrequencyPlot = new ImPlotFrequencyPlot("Depth Hz", 30);
   private final ImPlotDoublePlot colorDelayPlot = new ImPlotDoublePlot("Color Delay", 30);
   private final ImPlotDoublePlot depthDelayPlot = new ImPlotDoublePlot("Depth Delay", 30);
   private final RDXMessageSizeReadout colorMessageSizeReadout = new RDXMessageSizeReadout();
   private final RDXMessageSizeReadout depthMessageSizeReadout = new RDXMessageSizeReadout();
   private final RDXSequenceDiscontinuityPlot colorSequenceDiscontinuityPlot = new RDXSequenceDiscontinuityPlot();
   private final RDXSequenceDiscontinuityPlot depthSequenceDiscontinuityPlot = new RDXSequenceDiscontinuityPlot();

   private ByteBuffer depthDecompressionInputBuffer;
   private BytePointer depthDecompressionInputBytePointer;
   private Mat depthDecompressionInputMat;

   private ByteBuffer colorDecompressionInputBuffer;
   private BytePointer colorDecompressionInputBytePointer;
   private Mat colorDecompressionInputMat;

   private boolean depthInitialized = false;
   private boolean colorInitialized = false;
   private boolean sinusoidalPatternEnabled = false;

   private float depthToMetersScalar = 2.500000118743628E-4f;
   private int totalNumberOfPoints = 0;

   private final RigidBodyTransform sensorTransformToWorld = new RigidBodyTransform();
   private OpenCLManager openCLManager;
   private OpenCLFloatBuffer finalColoredDepthBuffer;
   private _cl_program openCLProgram;
   private _cl_kernel createPointCloudKernel;

   private OpenCLFloatBuffer parametersBuffer;
   private Mat yuv1420Image;
   private BytedecoImage color8UC3Image;
   private BytedecoImage color8UC4Image;
   private BytedecoImage depth16UC1Image;
   private BytedecoImage depth32FC1Image;

   private volatile boolean depthAvailable = false;
   private volatile boolean colorAvailable = false;
   private volatile boolean depthReceivedOne = false;
   private volatile boolean colorReceivedOne = false;
   private int depthWidth;
   private int depthHeight;
   private int colorWidth;
   private int colorHeight;
   private int totalColorPixels;

   public RDXROS2ColoredDepthVisualizer(String title, PubSubImplementation pubSubImplementation, ROS2Topic<ImageMessage> depthTopic, ROS2Topic<ImageMessage> colorTopic)
   {
      super(title + " (ROS 2)");
      titleBeforeAdditions = title;
      this.pubSubImplementation = pubSubImplementation;
      this.depthTopic = depthTopic;
      this.colorTopic = colorTopic;

      setSubscribed(subscribed.get());
   }

   private void subscribe()
   {
      subscribed.set(true);

      this.realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(pubSubImplementation, StringTools.titleToSnakeCase(titleBeforeAdditions));
      ROS2Tools.createCallbackSubscription(realtimeROS2Node, depthTopic, ROS2QosProfile.BEST_EFFORT(), subscriber ->
      {
         synchronized (imageMessagesSyncObject)
         {
            depthFrequencyPlot.ping();
            depthImageMessage.getData().resetQuick();
            subscriber.takeNextData(depthImageMessage, depthSampleInfo);
            depthAvailable = true;
            depthReceivedOne = true;
         }
      });
      ROS2Tools.createCallbackSubscription(realtimeROS2Node, colorTopic, ROS2QosProfile.BEST_EFFORT(), subscriber ->
      {
         synchronized (imageMessagesSyncObject)
         {
            colorFrequencyPlot.ping();
            colorImageMessage.getData().resetQuick();
            subscriber.takeNextData(colorImageMessage, colorSampleInfo);
            colorAvailable = true;
            colorReceivedOne = true;
         }
      });
      realtimeROS2Node.spin();
   }

   @Override
   public void create()
   {
      super.create();

      openCLManager = new OpenCLManager();
      openCLManager.create();
      openCLProgram = openCLManager.loadProgram("ColoredPointCloudCreator", "PerceptionCommon.cl");
      createPointCloudKernel = openCLManager.createKernel(openCLProgram, "createPointCloud");

      parametersBuffer = new OpenCLFloatBuffer(TOTAL_NUMBER_OF_PARAMETERS);
      parametersBuffer.createOpenCLBufferObject(openCLManager);
   }

   @Override
   public void update()
   {
      super.update();

      if (subscribed.get() && isActive() && colorAvailable && depthAvailable)
      {
         synchronized (imageMessagesSyncObject)
         {
            if (!depthInitialized)
            {
               depthWidth = depthImageMessage.getImageWidth();
               depthHeight = depthImageMessage.getImageHeight();
               totalNumberOfPoints = depthHeight * depthWidth;

               depthDecompressionInputBuffer = NativeMemoryTools.allocate(depthWidth * depthHeight * 2);
               depthDecompressionInputBytePointer = new BytePointer(depthDecompressionInputBuffer);
               depthDecompressionInputMat = new Mat(1, 1, opencv_core.CV_8UC1);

               // Set the depth camera intrinsics
               depthCameraInstrinsics.setCx(depthImageMessage.getIntrinsicParameters().getCx());
               depthCameraInstrinsics.setCy(depthImageMessage.getIntrinsicParameters().getCy());
               depthCameraInstrinsics.setFx(depthImageMessage.getIntrinsicParameters().getFx());
               depthCameraInstrinsics.setFy(depthImageMessage.getIntrinsicParameters().getFy());

               // Create OpenCV mat for depth image
               depth16UC1Image = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);

               // Allocates memory for vertex buffer for the point cloud
               LogTools.info("Allocated new buffers. {} total points", totalNumberOfPoints);
               pointCloudRenderer.create(totalNumberOfPoints);

               if (finalColoredDepthBuffer != null)
               {
                  finalColoredDepthBuffer.destroy(openCLManager);
               }
               finalColoredDepthBuffer = new OpenCLFloatBuffer(totalNumberOfPoints * FLOATS_PER_POINT, pointCloudRenderer.getVertexBuffer());
               finalColoredDepthBuffer.createOpenCLBufferObject(openCLManager);

               // Create OpenCV mat for depth image in meters. This is an OpenCL-mapped buffer which is uploaded to the OpenCL kernel.
               depth32FC1Image = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_32FC1);
               depth32FC1Image.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);

               depthInitialized = true;
            }

            // Get the depth image from the packet
            int depthNumberOfBytes = depthImageMessage.getData().size();
            depthDecompressionInputBuffer.rewind();
            depthDecompressionInputBuffer.limit(depthWidth * depthHeight * 2);
            for (int i = 0; i < depthNumberOfBytes; i++)
            {
               depthDecompressionInputBuffer.put(depthImageMessage.getData().get(i));
            }
            depthDecompressionInputBuffer.flip();

            depthDecompressionInputBytePointer.position(0);
            depthDecompressionInputBytePointer.limit(depthNumberOfBytes);

            depthDecompressionInputMat.cols(depthNumberOfBytes);
            depthDecompressionInputMat.data(depthDecompressionInputBytePointer);

            depth16UC1Image.getBackingDirectByteBuffer().rewind();
            opencv_imgcodecs.imdecode(depthDecompressionInputMat,
                                      opencv_imgcodecs.IMREAD_UNCHANGED,
                                      depth16UC1Image.getBytedecoOpenCVMat());
            depth16UC1Image.getBackingDirectByteBuffer().rewind();

            // Put the color image into OpenCL buffer
            depth16UC1Image.getBytedecoOpenCVMat().convertTo(depth32FC1Image.getBytedecoOpenCVMat(), opencv_core.CV_32FC1, depthToMetersScalar, 0.0);

            // Get the color image from the packet
            if (!colorInitialized)
            {
               colorWidth = colorImageMessage.getImageWidth();
               colorHeight = colorImageMessage.getImageHeight();
               totalColorPixels = colorHeight * colorWidth;

               colorDecompressionInputBuffer = NativeMemoryTools.allocate(colorWidth * colorHeight * 3);
               colorDecompressionInputBytePointer = new BytePointer(colorDecompressionInputBuffer);
               colorDecompressionInputMat = new Mat(1, 1, opencv_core.CV_8UC1);

               yuv1420Image = new Mat(1, 1, opencv_core.CV_8UC1);
               color8UC3Image = new BytedecoImage(colorWidth, colorHeight, opencv_core.CV_8UC3);
               color8UC4Image = new BytedecoImage(colorWidth, colorHeight, opencv_core.CV_8UC4);
               color8UC4Image.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);

               colorCameraInstrinsics.setCx(colorImageMessage.getIntrinsicParameters().getCx());
               colorCameraInstrinsics.setCy(colorImageMessage.getIntrinsicParameters().getCy());
               colorCameraInstrinsics.setFx(colorImageMessage.getIntrinsicParameters().getFx());
               colorCameraInstrinsics.setFy(colorImageMessage.getIntrinsicParameters().getFy());

               colorInitialized = true;
            }

            // Get the color image from the packet
            int colorNumberOfBytes = colorImageMessage.getData().size();
            colorDecompressionInputBuffer.rewind();
            colorDecompressionInputBuffer.limit(colorWidth * colorHeight * 3); // 8UC3 - 3 bytes each
            for (int i = 0; i < colorNumberOfBytes; i++)
            {
               colorDecompressionInputBuffer.put(colorImageMessage.getData().get(i));
            }
            colorDecompressionInputBuffer.flip();

            colorDecompressionInputBytePointer.position(0);
            colorDecompressionInputBytePointer.limit(colorNumberOfBytes);

            colorDecompressionInputMat.cols(colorNumberOfBytes);
            colorDecompressionInputMat.data(colorDecompressionInputBytePointer);

            opencv_imgcodecs.imdecode(colorDecompressionInputMat, opencv_imgcodecs.IMREAD_UNCHANGED, yuv1420Image);

            opencv_imgproc.cvtColor(yuv1420Image, color8UC3Image.getBytedecoOpenCVMat(), opencv_imgproc.COLOR_YUV2RGBA_I420);
            // Put the depth image into OpenCL buffer
            opencv_imgproc.cvtColor(color8UC3Image.getBytedecoOpenCVMat(), color8UC4Image.getBytedecoOpenCVMat(), opencv_imgproc.COLOR_RGB2RGBA);

            sensorTransformToWorld.getTranslation().set(colorImageMessage.getPosition());
            sensorTransformToWorld.getRotation().set(colorImageMessage.getOrientation());

            depthMessageSizeReadout.update(depthImageMessage.getData().size());
            colorMessageSizeReadout.update(colorImageMessage.getData().size());
            depthSequenceDiscontinuityPlot.update(depthImageMessage.getSequenceNumber());
            colorSequenceDiscontinuityPlot.update(colorImageMessage.getSequenceNumber());
            depthDelayPlot.addValue(TimeTools.calculateDelay(depthImageMessage.getAcquisitionTime().getSecondsSinceEpoch(),
                                                             depthImageMessage.getAcquisitionTime().getAdditionalNanos()));
            colorDelayPlot.addValue(TimeTools.calculateDelay(colorImageMessage.getAcquisitionTime().getSecondsSinceEpoch(),
                                                             colorImageMessage.getAcquisitionTime().getAdditionalNanos()));

            depthAvailable = false;
            colorAvailable = false;
         }

         // If both depth and color images are available, configure the OpenCL kernel and run it, to generate the point cloud float buffer.
         parametersBuffer.getBytedecoFloatBufferPointer().put(0, FOCAL_LENGTH_COLOR);
         parametersBuffer.getBytedecoFloatBufferPointer().put(1, CMOS_WIDTH_COLOR);
         parametersBuffer.getBytedecoFloatBufferPointer().put(2, CMOS_HEIGHT_COLOR);
         parametersBuffer.getBytedecoFloatBufferPointer().put(3, sensorTransformToWorld.getTranslation().getX32());
         parametersBuffer.getBytedecoFloatBufferPointer().put(4, sensorTransformToWorld.getTranslation().getY32());
         parametersBuffer.getBytedecoFloatBufferPointer().put(5, sensorTransformToWorld.getTranslation().getZ32());
         parametersBuffer.getBytedecoFloatBufferPointer().put(6, (float) sensorTransformToWorld.getRotation().getM00());
         parametersBuffer.getBytedecoFloatBufferPointer().put(7, (float) sensorTransformToWorld.getRotation().getM01());
         parametersBuffer.getBytedecoFloatBufferPointer().put(8, (float) sensorTransformToWorld.getRotation().getM02());
         parametersBuffer.getBytedecoFloatBufferPointer().put(9, (float) sensorTransformToWorld.getRotation().getM10());
         parametersBuffer.getBytedecoFloatBufferPointer().put(10, (float) sensorTransformToWorld.getRotation().getM11());
         parametersBuffer.getBytedecoFloatBufferPointer().put(11, (float) sensorTransformToWorld.getRotation().getM12());
         parametersBuffer.getBytedecoFloatBufferPointer().put(12, (float) sensorTransformToWorld.getRotation().getM20());
         parametersBuffer.getBytedecoFloatBufferPointer().put(13, (float) sensorTransformToWorld.getRotation().getM21());
         parametersBuffer.getBytedecoFloatBufferPointer().put(14, (float) sensorTransformToWorld.getRotation().getM22());
         parametersBuffer.getBytedecoFloatBufferPointer().put(15, (float) depthCameraInstrinsics.getCx());
         parametersBuffer.getBytedecoFloatBufferPointer().put(16, (float) depthCameraInstrinsics.getCy());
         parametersBuffer.getBytedecoFloatBufferPointer().put(17, (float) depthCameraInstrinsics.getFx());
         parametersBuffer.getBytedecoFloatBufferPointer().put(18, (float) depthCameraInstrinsics.getFy());
         parametersBuffer.getBytedecoFloatBufferPointer().put(19, (float) depth32FC1Image.getImageWidth());
         parametersBuffer.getBytedecoFloatBufferPointer().put(20, (float) depth32FC1Image.getImageHeight());
         parametersBuffer.getBytedecoFloatBufferPointer().put(21, (float) color8UC4Image.getImageWidth());
         parametersBuffer.getBytedecoFloatBufferPointer().put(22, (float) color8UC4Image.getImageHeight());
         parametersBuffer.getBytedecoFloatBufferPointer().put(23, (float) (sinusoidalPatternEnabled ? 1.0f : 0.0f));
         // Update TOTAL_NUMBER_OF_POINTS in the static constants if adding more parameters here.

         // Upload the buffers to the OpenCL device (GPU)
         depth32FC1Image.writeOpenCLImage(openCLManager);
         color8UC4Image.writeOpenCLImage(openCLManager);
         parametersBuffer.writeOpenCLBufferObject(openCLManager);

         // Set the OpenCL kernel arguments
         openCLManager.setKernelArgument(createPointCloudKernel, 0, depth32FC1Image.getOpenCLImageObject());
         openCLManager.setKernelArgument(createPointCloudKernel, 1, color8UC4Image.getOpenCLImageObject());
         openCLManager.setKernelArgument(createPointCloudKernel, 2, finalColoredDepthBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(createPointCloudKernel, 3, parametersBuffer.getOpenCLBufferObject());

         // Run the OpenCL kernel
         openCLManager.execute2D(createPointCloudKernel, depthWidth, depthHeight);

         // Read the OpenCL buffer back to the CPU
         finalColoredDepthBuffer.readOpenCLBufferObject(openCLManager);
         openCLManager.finish();

         // Request the PointCloudRenderer to render the point cloud from OpenCL-mapped buffers
         pointCloudRenderer.updateMeshFastest(totalNumberOfPoints);
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
      ImGui.text(colorTopic.getName());
      if (colorReceivedOne)
      {
         colorMessageSizeReadout.renderImGuiWidgets();
      }
      ImGui.text(depthTopic.getName());
      if (depthReceivedOne)
      {
         depthMessageSizeReadout.renderImGuiWidgets();
      }
      if (colorReceivedOne)
         colorFrequencyPlot.renderImGuiWidgets();
      if (depthReceivedOne)
         depthFrequencyPlot.renderImGuiWidgets();

      if (colorReceivedOne)
         colorDelayPlot.renderImGuiWidgets();
      if (depthReceivedOne)
         depthDelayPlot.renderImGuiWidgets();

      if (colorReceivedOne)
         colorSequenceDiscontinuityPlot.renderImGuiWidgets();
      if (depthReceivedOne)
         depthSequenceDiscontinuityPlot.renderImGuiWidgets();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (isActive())
         pointCloudRenderer.getRenderables(renderables, pool);
   }

   @Override
   public void destroy()
   {
      unsubscribe();
      super.destroy();
   }

   public void setSubscribed(boolean subscribed)
   {
      if (subscribed && realtimeROS2Node == null)
      {
         subscribe();
      }
      else if (!subscribed && realtimeROS2Node == null)
      {
         unsubscribe();
      }
   }

   private void unsubscribe()
   {
      subscribed.set(false);
      if (realtimeROS2Node != null)
      {
         realtimeROS2Node.destroy();
         realtimeROS2Node = null;
      }
   }

   public boolean isSubscribed()
   {
      return subscribed.get();
   }
}