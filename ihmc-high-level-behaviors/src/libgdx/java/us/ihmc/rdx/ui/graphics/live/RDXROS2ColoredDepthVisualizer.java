package us.ihmc.rdx.ui.graphics.live;

import boofcv.struct.calib.CameraPinholeBrown;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.RigidBodyTransformMessage;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.VideoPacket;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commons.FormattingTools;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.nadia.parameters.NadiaRobotModel;
import us.ihmc.nadia.parameters.robotVersions.NadiaVersion;
import us.ihmc.perception.*;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.tools.ImPlotIntegerPlot;
import us.ihmc.rdx.ui.visualizers.ImGuiFrequencyPlot;
import us.ihmc.rdx.ui.visualizers.RDXVisualizer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

import java.util.concurrent.atomic.AtomicReference;

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
   private final RigidBodyTransform transformToWorldFrame = new RigidBodyTransform();

   private final AtomicReference<VideoPacket> colorPacketReference = new AtomicReference<>(null);
   private final AtomicReference<VideoPacket> depthPacketReference = new AtomicReference<>(null);

   private final ImPlotIntegerPlot segmentIndexPlot = new ImPlotIntegerPlot("Segment", 30);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiFrequencyPlot frequencyPlot = new ImGuiFrequencyPlot();
   private final ImBoolean subscribed = new ImBoolean(true);
   private final ROS2Topic<VideoPacket> depthTopic;
   private final ROS2Topic<VideoPacket> colorTopic;
   private final ROS2Node ros2Node;

   private final byte[] heapArrayDepth = new byte[25000000];
   private final byte[] heapArrayColor = new byte[25000000];

   private boolean depthInitialized = false;
   private boolean colorInitialized = false;
   private boolean sinusoidalPatternEnabled = false;

   private float depthToMetersScalar = 2.500000118743628E-4f;
   private int totalNumberOfPoints = 0;
   private int bytesPerSegment = 0;
   private String kilobytes = "";

   private final FramePose3D cameraPose = new FramePose3D();
   private final RigidBodyTransform steppingCameraTransform;
   private final IHMCROS2Input<RigidBodyTransformMessage> frameUpdateSubscription;
   private IHMCROS2Callback<?> ros2Callback = null;
   private OpenCLManager openCLManager;
   private OpenCLFloatBuffer finalColoredDepthBuffer;
   private _cl_program openCLProgram;
   private _cl_kernel createPointCloudKernel;
   private String messageSizeString;

   private final ROS2Helper ros2Helper;
   private final ROS2SyncedRobotModel syncedRobot;
   private OpenCLFloatBuffer parametersBuffer;
   private BytedecoImage color8UC4Image;
   private BytedecoImage depth32FC1Image;
   private Mat depthImage;
   private Mat colorImage;

   private boolean depthAvailable = false;
   private boolean colorAvailable = false;

   public RDXROS2ColoredDepthVisualizer(String title, ROS2Node ros2Node, ROS2Topic<VideoPacket> depthTopic, ROS2Topic<VideoPacket> colorTopic)
   {
      super(title + " (ROS 2)");
      this.ros2Node = ros2Node;
      this.depthTopic = depthTopic;
      this.colorTopic = colorTopic;

      NadiaRobotModel robotModel = new NadiaRobotModel(NadiaVersion.V16_NEW_TORSO_FULL_ROBOT, RobotTarget.REAL_ROBOT);


      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "l515_point_cloud_node");
      ros2Helper = new ROS2Helper(ros2Node);

      steppingCameraTransform = robotModel.getSensorInformation().getSteppingCameraTransform();
      frameUpdateSubscription = ros2Helper.subscribe(ROS2Tools.STEPPING_FRAME_UPDATE);

      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);
      syncedRobot.initializeToDefaultRobotInitialSetup(0.0, 0.0, 0.0, 0.0);

      setSubscribed(subscribed.get());
   }

   private void subscribe()
   {
      subscribed.set(true);

      ros2Callback = new IHMCROS2Callback<>(ros2Node, depthTopic, (packet) -> {
         depthPacketReference.set(packet);
         depthAvailable = true;
      });
      ros2Callback = new IHMCROS2Callback<>(ros2Node, colorTopic, (packet) -> {
         colorPacketReference.set(packet);
         colorAvailable = true;
      });
   }

   @Override
   public void create()
   {
      super.create();

      openCLManager = new OpenCLManager();
      openCLManager.create();
      openCLProgram = openCLManager.loadProgram("ColoredPointCloudCreator");
      createPointCloudKernel = openCLManager.createKernel(openCLProgram, "createPointCloud");

      parametersBuffer = new OpenCLFloatBuffer(TOTAL_NUMBER_OF_PARAMETERS);
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
               // Set the depth camera intrinsics
               depthCameraInstrinsics.setCx(depthPacket.getIntrinsicParameters().getCx());
               depthCameraInstrinsics.setCy(depthPacket.getIntrinsicParameters().getCy());
               depthCameraInstrinsics.setFx(depthPacket.getIntrinsicParameters().getFx());
               depthCameraInstrinsics.setFy(depthPacket.getIntrinsicParameters().getFy());

               // Create OpenCV mat for depth image
               depthImage = new Mat(depthPacket.getImageHeight(), depthPacket.getImageWidth(), opencv_core.CV_16UC1);

               totalNumberOfPoints = depthPacket.getImageHeight() * depthPacket.getImageWidth();
               bytesPerSegment = totalNumberOfPoints * BYTES_PER_POINT;
               kilobytes = FormattingTools.getFormattedDecimal1D((double) bytesPerSegment / 1000.0);
               messageSizeString = String.format("Message size: %s KB", kilobytes);

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
               depth32FC1Image = new BytedecoImage(depthPacket.getImageWidth(), depthPacket.getImageHeight(), opencv_core.CV_32FC1);
               depth32FC1Image.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);

               depthInitialized = true;
            }

            // Get the depth image from the packet
            depthPacket.getData().toArray(heapArrayDepth, 0, depthPacket.getData().size());
            BytedecoOpenCVTools.decompressDepthPNG(heapArrayDepth, depthImage);

            // Put the color image into OpenCL buffer
            depthImage.convertTo(depth32FC1Image.getBytedecoOpenCVMat(), opencv_core.CV_32FC1, depthToMetersScalar, 0.0);
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

               colorCameraInstrinsics.setCx(colorPacket.getIntrinsicParameters().getCx());
               colorCameraInstrinsics.setCy(colorPacket.getIntrinsicParameters().getCy());
               colorCameraInstrinsics.setFx(colorPacket.getIntrinsicParameters().getFx());
               colorCameraInstrinsics.setFy(colorPacket.getIntrinsicParameters().getFy());

               colorInitialized = true;
            }

            // Get the color image from the packet
            colorPacket.getData().toArray(heapArrayColor, 0, colorPacket.getData().size());
            colorImage = BytedecoOpenCVTools.decompressImageJPGUsingYUV(heapArrayColor);

            // Put the depth image into OpenCL buffer
            opencv_imgproc.cvtColor(colorImage, color8UC4Image.getBytedecoOpenCVMat(), opencv_imgproc.COLOR_RGB2RGBA);
         }

         if (depthInitialized && colorInitialized)
         {
            if (frameUpdateSubscription.getMessageNotification().poll())
            {
               MessageTools.toEuclid(frameUpdateSubscription.getMessageNotification().read(), steppingCameraTransform);
            }
            syncedRobot.update();
            ReferenceFrame cameraFrame = syncedRobot.getReferenceFrames().getSteppingCameraFrame();
            cameraPose.setToZero(cameraFrame);
            cameraPose.changeFrame(ReferenceFrame.getWorldFrame());
            RigidBodyTransform transformToWorldFrame = cameraFrame.getTransformToWorldFrame();

            // If both depth and color images are available, configure the OpenCL kernel and run it, to generate the point cloud float buffer.
            parametersBuffer.getBytedecoFloatBufferPointer().put(0, FOCAL_LENGTH_COLOR);
            parametersBuffer.getBytedecoFloatBufferPointer().put(1, CMOS_WIDTH_COLOR);
            parametersBuffer.getBytedecoFloatBufferPointer().put(2, CMOS_HEIGHT_COLOR);
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
            openCLManager.execute2D(createPointCloudKernel, depthImage.cols(), depthImage.rows());

            // Read the OpenCL buffer back to the CPU
            finalColoredDepthBuffer.readOpenCLBufferObject(openCLManager);
            openCLManager.finish();

            // Request the PointCloudRenderer to render the point cloud from OpenCL-mapped buffers
            pointCloudRenderer.updateMeshFastest(totalNumberOfPoints);

            depthAvailable = false;
            colorAvailable = false;

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
