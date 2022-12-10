package us.ihmc.rdx.ui.graphics.live;

import boofcv.struct.calib.CameraPinholeBrown;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.VideoPacket;
import us.ihmc.commons.FormattingTools;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.tools.ImPlotIntegerPlot;
import us.ihmc.rdx.ui.visualizers.ImGuiFrequencyPlot;
import us.ihmc.rdx.ui.visualizers.RDXVisualizer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

import java.util.concurrent.atomic.AtomicReference;

import static org.bytedeco.opencv.global.opencv_highgui.imshow;
import static org.bytedeco.opencv.global.opencv_highgui.waitKeyEx;

public class RDXROS2ColoredDepthVisualizer extends RDXVisualizer implements RenderableProvider
{
   private final static int FLOATS_PER_POINT = 6;
   private final static int BYTES_PER_POINT = FLOATS_PER_POINT * 4;

   private final RigidBodyTransform transformToWorldFrame = new RigidBodyTransform();

   private boolean depthInitialized = false;
   private boolean colorInitialized = false;

   private float depthToMetersScalar = 2.500000118743628E-4f;

   private BytedecoImage depth32FC1Image;

   private Mat depthImage;
   private Mat colorImage;

   private final CameraPinholeBrown depthCameraInstrinsics = new CameraPinholeBrown();
   private final CameraPinholeBrown colorCameraInstrinsics = new CameraPinholeBrown();

   private byte[] heapArrayDepth = new byte[25000000];
   private byte[] heapArrayColor = new byte[25000000];
   private final ROS2Node ros2Node;
   private final ROS2Topic<?> depthTopic;
   private final ROS2Topic<?> colorTopic;
   private IHMCROS2Callback<?> ros2Callback = null;
   private final ImGuiFrequencyPlot frequencyPlot = new ImGuiFrequencyPlot();
   private final ImPlotIntegerPlot segmentIndexPlot = new ImPlotIntegerPlot("Segment", 30);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean subscribed = new ImBoolean(true);
   private final RDXPointCloudRenderer pointCloudRenderer = new RDXPointCloudRenderer();
   private OpenCLManager openCLManager;
   private OpenCLFloatBuffer finalColoredDepthBuffer;
   private _cl_program openCLProgram;
   private _cl_kernel createPointCloudKernel;
   private String messageSizeString;

   private AtomicReference<VideoPacket> colorPacketReference = new AtomicReference<>(null);
   ;
   private AtomicReference<VideoPacket> depthPacketReference = new AtomicReference<>(null);

   private OpenCLFloatBuffer parametersBuffer;
   private BytedecoImage color8UC4Image;

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
      LogTools.info("Received color packet");
      colorPacketReference.set(packet);
   }

   private void depthCallback(VideoPacket packet)
   {
      LogTools.info("Received depth packet");
      depthPacketReference.set(packet);
   }

   @Override
   public void create()
   {
      super.create();

      openCLManager = new OpenCLManager();
      openCLManager.create();
      openCLProgram = openCLManager.loadProgram("RealsenseColoredPointCloudCreator");
      createPointCloudKernel = openCLManager.createKernel(openCLProgram, "createPointCloud");

      parametersBuffer = new OpenCLFloatBuffer(23);
      parametersBuffer.createOpenCLBufferObject(openCLManager);
   }

   @Override
   public void update()
   {
      super.update();

      if (subscribed.get() && isActive())
      {
         VideoPacket depthPacket = this.depthPacketReference.getAndSet(null);
         if (depthPacket != null)
         {
            if (!depthInitialized)
            {
               depthCameraInstrinsics.setCx(depthPacket.getIntrinsicParameters().getCx());
               depthCameraInstrinsics.setCy(depthPacket.getIntrinsicParameters().getCy());
               depthCameraInstrinsics.setFx(depthPacket.getIntrinsicParameters().getFx());
               depthCameraInstrinsics.setFy(depthPacket.getIntrinsicParameters().getFy());




               depthImage = new Mat(depthPacket.getImageHeight(), depthPacket.getImageWidth(), opencv_core.CV_16UC1);

               int totalNumberOfPoints = depthPacket.getImageHeight() * depthPacket.getImageWidth();
               int bytesPerSegment = totalNumberOfPoints * BYTES_PER_POINT;
               String kilobytes = FormattingTools.getFormattedDecimal1D((double) bytesPerSegment / 1000.0);
               messageSizeString = String.format("Message size: %s KB", kilobytes);

               //               pointCloudRenderer.create(totalNumberOfPoints);

               if (finalColoredDepthBuffer != null)
                  finalColoredDepthBuffer.destroy(openCLManager);
               finalColoredDepthBuffer = new OpenCLFloatBuffer(totalNumberOfPoints * FLOATS_PER_POINT);
               finalColoredDepthBuffer.createOpenCLBufferObject(openCLManager);

               depth32FC1Image = new BytedecoImage(depthPacket.getImageWidth(), depthPacket.getImageHeight(), opencv_core.CV_32FC1);
               depth32FC1Image.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);

               LogTools.info("Allocated new buffers. {} total points", totalNumberOfPoints);

               depthInitialized = true;
            }

            LogTools.info("Initialized. Rendering Now");

            // Get the depth image from the packet
            depthPacket.getData().toArray(heapArrayDepth, 0, depthPacket.getData().size());
            BytedecoOpenCVTools.decompressDepthPNG(heapArrayDepth, depthImage);
            depthImage.convertTo(depth32FC1Image.getBytedecoOpenCVMat(), opencv_core.CV_32FC1, depthToMetersScalar, 0.0);

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
         }

         // Get the color image from the packet
         VideoPacket colorPacket = this.colorPacketReference.getAndSet(null);
         if (colorPacket != null)
         {
            if (!colorInitialized)
            {
               colorImage = new Mat(colorPacket.getImageHeight(), colorPacket.getImageWidth(), opencv_core.CV_8UC3);
               color8UC4Image = new BytedecoImage(colorPacket.getImageWidth(), colorPacket.getImageHeight(), opencv_core.CV_8UC4);
               color8UC4Image.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);

               colorPacket.getImageWidth();
               colorPacket.getImageHeight();

               colorInitialized = true;
            }

            colorPacket.getData().toArray(heapArrayColor, 0, colorPacket.getData().size());
            colorImage = BytedecoOpenCVTools.decompressImageJPGUsingYUV(heapArrayColor);

            imshow("/l515/color", colorImage);
            int code = waitKeyEx(1);
            if (code == 113)
            {
               System.exit(0);
            }
         }

         if (depthInitialized && colorInitialized)
         {
            LogTools.info("Creating Fused Point Cloud");

            float l515ColorFocalLength = 0.00254f;
            float l515ColorCMOSWidth = 0.0036894f;
            float l515ColorCMOSHeight = 0.0020753f;

            parametersBuffer.getBytedecoFloatBufferPointer().put(0, l515ColorFocalLength);
            parametersBuffer.getBytedecoFloatBufferPointer().put(1, l515ColorCMOSWidth);
            parametersBuffer.getBytedecoFloatBufferPointer().put(2, l515ColorCMOSHeight);
            parametersBuffer.getBytedecoFloatBufferPointer().put(3, transformToWorldFrame.getTranslation().getX32());
            parametersBuffer.getBytedecoFloatBufferPointer().put(4, transformToWorldFrame.getTranslation().getY32());
            parametersBuffer.getBytedecoFloatBufferPointer().put(5, transformToWorldFrame.getTranslation().getZ32());
            parametersBuffer.getBytedecoFloatBufferPointer().put(6, (float) transformToWorldFrame.getRotation().getM00());
            parametersBuffer.getBytedecoFloatBufferPointer().put(7, (float) transformToWorldFrame.getRotation().getM01());
            parametersBuffer.getBytedecoFloatBufferPointer().put(8, (float) transformToWorldFrame.getRotation().getM02());
            parametersBuffer.getBytedecoFloatBufferPointer().put(9, (float) transformToWorldFrame.getRotation().getM10());
            parametersBuffer.getBytedecoFloatBufferPointer().put(10, (float) transformToWorldFrame.getRotation().getM11());
            parametersBuffer.getBytedecoFloatBufferPointer().put(11, (float) transformToWorldFrame.getRotation().getM12());
            parametersBuffer.getBytedecoFloatBufferPointer().put(12, (float) transformToWorldFrame.getRotation().getM20());
            parametersBuffer.getBytedecoFloatBufferPointer().put(13, (float) transformToWorldFrame.getRotation().getM21());
            parametersBuffer.getBytedecoFloatBufferPointer().put(14, (float) transformToWorldFrame.getRotation().getM22());
            parametersBuffer.getBytedecoFloatBufferPointer().put(15, (float) depthCameraInstrinsics.getCx());
            parametersBuffer.getBytedecoFloatBufferPointer().put(16, (float) depthCameraInstrinsics.getCy());
            parametersBuffer.getBytedecoFloatBufferPointer().put(17, (float) depthCameraInstrinsics.getFx());
            parametersBuffer.getBytedecoFloatBufferPointer().put(18, (float) depthCameraInstrinsics.getFy());
            parametersBuffer.getBytedecoFloatBufferPointer().put(19, (float) depthImage.cols());
            parametersBuffer.getBytedecoFloatBufferPointer().put(20, (float) depthImage.rows());
            parametersBuffer.getBytedecoFloatBufferPointer().put(21, (float) colorImage.cols());
            parametersBuffer.getBytedecoFloatBufferPointer().put(22, (float) colorImage.rows());

            depth32FC1Image.writeOpenCLImage(openCLManager);
            color8UC4Image.writeOpenCLImage(openCLManager);
            parametersBuffer.writeOpenCLBufferObject(openCLManager);

            openCLManager.setKernelArgument(createPointCloudKernel, 0, depth32FC1Image.getOpenCLImageObject());
            openCLManager.setKernelArgument(createPointCloudKernel, 1, color8UC4Image.getOpenCLImageObject());
            openCLManager.setKernelArgument(createPointCloudKernel, 2, finalColoredDepthBuffer.getOpenCLBufferObject());
            openCLManager.setKernelArgument(createPointCloudKernel, 3, parametersBuffer.getOpenCLBufferObject());
            openCLManager.execute2D(createPointCloudKernel, depthImage.cols(), depthImage.rows());
            finalColoredDepthBuffer.readOpenCLBufferObject(openCLManager);
            openCLManager.finish();
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
