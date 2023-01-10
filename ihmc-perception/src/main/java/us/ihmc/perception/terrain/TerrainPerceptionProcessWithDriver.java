package us.ihmc.perception.terrain;

import boofcv.struct.calib.CameraPinholeBrown;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl._cl_mem;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.IntrinsicParametersMessage;
import perception_msgs.msg.dds.PlanarRegionsListWithPoseMessage;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.ihmcPerception.depthData.CollisionShapeTester;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.perception.*;
import us.ihmc.perception.filters.CollidingScanRegionFilter;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsCustomizer;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.perception.realsense.BytedecoRealsense;
import us.ihmc.perception.realsense.RealSenseHardwareManager;
import us.ihmc.perception.realsense.RealsenseConfiguration;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegionsListWithPose;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.Throttler;

import java.time.Instant;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import static org.bytedeco.opencv.global.opencv_highgui.*;

/**
 * Publishes color and depth from Realsense D435
 * ----+ L515 Device Configuration: Serial Number: F0245563, Depth Height 768, Depth Width: 1024, Depth FPS: 15, Color Height 720, Color Width: 1280, Color FPS:
 * 15
 * ----+ D435: Serial Number: 752112070330, Depth Width: 848, Depth Height: 480, Depth FPS: 30, Color Width: 848, Color Height: 480, Color FPS: 30
 */
public class TerrainPerceptionProcessWithDriver
{
   private final Activator nativesLoadedActivator;
   private final ROS2Helper ros2Helper;
   private final Supplier<ReferenceFrame> sensorFrameUpdater;
   private final FramePose3D cameraPose = new FramePose3D();
   private final ROS2Topic<ImageMessage> colorTopic;
   private final ROS2Topic<ImageMessage> depthTopic;
   private final ImageMessage colorImageMessage = new ImageMessage();
   private final ImageMessage depthImageMessage = new ImageMessage();
   private final Mat yuvColorImage = new Mat();
   private final Throttler throttler = new Throttler();
   private final RapidPlanarRegionsExtractor rapidPlanarRegionsExtractor;
   private final RapidPlanarRegionsCustomizer rapidPlanarRegionsCustomizer;
   private final ROS2Topic<PlanarRegionsListWithPoseMessage> regionsWithPoseTopic;

   private RealSenseHardwareManager realSenseHardwareManager;
   private CollidingScanRegionFilter collisionFilter;
   private BytedecoRealsense sensor;
   private Mat color8UC3Image;

   private volatile boolean running = true;
   private final double outputPeriod = UnitConversions.hertzToSeconds(30.0);

   private BytePointer compressedColorPointer;
   private BytePointer compressedDepthPointer;

   private final String serialNumber;
   private long depthSequenceNumber = 0;
   private long colorSequenceNumber = 0;

   private final RealtimeROS2Node realtimeROS2Node;
   private final ROS2StoredPropertySetGroup ros2PropertySetGroup;
   private final OpenCLManager openCLManager;
   private final _cl_program openCLProgram;

   private final IHMCRealtimeROS2Publisher<ImageMessage> depthPublisher;
   private final IHMCRealtimeROS2Publisher<ImageMessage> debugImagePublisher;
   private CameraPinholeBrown depthCameraIntrinsics;
   private final ImageMessage depthImagePacket = new ImageMessage();
   private final ImageMessage debugExtractionImagePacket = new ImageMessage();

   private Mat depth16UC1Mat;
   private BytedecoImage depth16UC1Image;

   private RealsenseConfiguration realsenseConfiguration;

   private final ROS2Node ros2Node;

   private int depthWidth;
   private int depthHeight;
   private int colorWidth;
   private int colorHeight;

   public TerrainPerceptionProcessWithDriver(RealsenseConfiguration realsenseConfiguration,
                                             ROS2Topic<ImageMessage> depthTopic,
                                             ROS2Topic<ImageMessage> colorTopic,
                                             ROS2Topic<PlanarRegionsListWithPoseMessage> regionsWithPoseTopic,
                                             Supplier<ReferenceFrame> sensorFrameUpdater)
   {
      this.serialNumber = realsenseConfiguration.getSerialNumber();
      this.colorTopic = colorTopic;
      this.depthTopic = depthTopic;
      this.regionsWithPoseTopic = regionsWithPoseTopic;
      this.sensorFrameUpdater = sensorFrameUpdater;
      this.realsenseConfiguration = realsenseConfiguration;

      depthCameraIntrinsics = new CameraPinholeBrown();
      nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();

      this.openCLManager = new OpenCLManager();
      this.openCLManager.create();
      openCLProgram = openCLManager.loadProgram("RapidRegionsExtractor");

      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "realsense_publisher");
      depthPublisher = ROS2Tools.createPublisher(realtimeROS2Node, depthTopic, ROS2QosProfile.BEST_EFFORT());
      debugImagePublisher = ROS2Tools.createPublisher(realtimeROS2Node, ROS2Tools.TERRAIN_DEBUG_IMAGE, ROS2QosProfile.BEST_EFFORT());

      realtimeROS2Node.spin();

      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "terrain_perception_node");
      ros2Helper = new ROS2Helper(ros2Node);

      rapidPlanarRegionsExtractor = new RapidPlanarRegionsExtractor();
      rapidPlanarRegionsExtractor.create(openCLManager,
                                         openCLProgram,
                                         realsenseConfiguration.getDepthWidth(),
                                         realsenseConfiguration.getDepthHeight(),
                                         depthCameraIntrinsics.fx,
                                         depthCameraIntrinsics.fy,
                                         depthCameraIntrinsics.cx,
                                         depthCameraIntrinsics.cy);

      rapidPlanarRegionsCustomizer = new RapidPlanarRegionsCustomizer();

      ros2PropertySetGroup = new ROS2StoredPropertySetGroup(ros2Helper);
      ros2PropertySetGroup.registerStoredPropertySet(GPUPlanarRegionExtractionComms.PARAMETERS, rapidPlanarRegionsExtractor.getParameters());
      ros2PropertySetGroup.registerStoredPropertySet(GPUPlanarRegionExtractionComms.POLYGONIZER_PARAMETERS,
                                                     rapidPlanarRegionsCustomizer.getPolygonizerParameters());
      ros2PropertySetGroup.registerStoredPropertySet(GPUPlanarRegionExtractionComms.CONVEX_HULL_FACTORY_PARAMETERS,
                                                     rapidPlanarRegionsCustomizer.getConcaveHullFactoryParameters());

      while (running)
      {
         update();
         throttler.waitAndRun(outputPeriod); // do the waiting after we send to remove unecessary latency
      }

      destroy();
      System.exit(0);
   }

   private void update()
   {
      if (nativesLoadedActivator.poll())
      {
         if (nativesLoadedActivator.isNewlyActivated())
         {
            realSenseHardwareManager = new RealSenseHardwareManager();
            sensor = realSenseHardwareManager.createBytedecoRealsenseDevice(realsenseConfiguration.getSerialNumber(), realsenseConfiguration.getDepthWidth(),
                                                                            realsenseConfiguration.getDepthHeight(), realsenseConfiguration.getDepthFPS());

            if (sensor.getDevice() == null)
            {
               running = false;
               throw new RuntimeException("Device not found. Set -Dd435.serial.number=00000000000");
            }
            sensor.enableColor(realsenseConfiguration.getColorWidth(), realsenseConfiguration.getColorHeight(), realsenseConfiguration.getColorFPS());
            sensor.initialize();

            depthWidth = sensor.getDepthWidth();
            depthHeight = sensor.getDepthHeight();
            colorWidth = sensor.getColorWidth();
            colorHeight = sensor.getColorHeight();

            LogTools.info("Depth width: " + depthWidth + ", height: " + depthHeight);
            LogTools.info("Color width: " + colorWidth + ", height: " + colorHeight);
         }

         if (sensor.readFrameData())
         {
            sensor.updateDataBytePointers();

            Instant now = Instant.now();


            //            _cl_mem image = openCLManager.createImage(OpenCL.CL_MEM_READ_ONLY, depthWidth, depthHeight, OpenCL.CL_R, OpenCL.CL_UNSIGNED_INT16, depthFrameData);


            MutableBytePointer depthFrameData = sensor.getDepthFrameData();
            depth16UC1Mat = new Mat(depthHeight, depthWidth, opencv_core.CV_16UC1, depthFrameData);
            depth16UC1Image = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);

            BytedecoImage depth32FC1Image = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_32FC1);
            depth16UC1Mat.convertTo(depth32FC1Image.getBytedecoOpenCVMat(), opencv_core.CV_32FC1, 1.0f, 0.0);

            int resultType = opencv_core.CV_16UC1;
            depth32FC1Image.getBytedecoOpenCVMat().convertTo(depth16UC1Image.getBytedecoOpenCVMat(), resultType, 1, 0);

            setDepthExtrinsics(sensor, depthImageMessage.getIntrinsicParameters());

            MutableBytePointer colorFrameData = sensor.getColorFrameData();
            color8UC3Image = new Mat(colorHeight, colorWidth, opencv_core.CV_8UC3, colorFrameData);
            setColorExtrinsics(sensor, colorImageMessage.getIntrinsicParameters());

            // Important not to store as a field, as update() needs to be called each frame
            ReferenceFrame cameraFrame = sensorFrameUpdater.get();
            cameraPose.setToZero(cameraFrame);
            cameraPose.changeFrame(ReferenceFrame.getWorldFrame());
            depthImageMessage.getPosition().set(cameraPose.getPosition());
            depthImageMessage.getOrientation().set(cameraPose.getOrientation());
            colorImageMessage.getPosition().set(cameraPose.getPosition());
            colorImageMessage.getOrientation().set(cameraPose.getOrientation());
            colorImageMessage.setSequenceNumber(colorSequenceNumber++);
            depthImageMessage.setSequenceNumber(depthSequenceNumber++);

//            display(depth16UC1Mat, color8UC3Image, 0);

            LogTools.info("Depth Image: {}", depth16UC1Image);

            publishPlanarRegionsWithPose(depth16UC1Image, cameraFrame, regionsWithPoseTopic);
            publishCompressedDepth(depth16UC1Image, now, sensor.getDepthHeight(), sensor.getDepthWidth(), depthTopic);
            publishCompressedColor(color8UC3Image, now, sensor.getColorHeight(), sensor.getColorWidth(), colorTopic);
         }
      }
   }

   private void publishCompressedDepth(BytedecoImage depth16UC1Image, Instant now, int height, int width, ROS2Topic<ImageMessage> topic)
   {
      compressedDepthPointer = new BytePointer();
      BytedecoOpenCVTools.compressImagePNG(depth16UC1Image.getBytedecoOpenCVMat(), compressedDepthPointer);
      BytedecoOpenCVTools.fillImageMessage(compressedDepthPointer, depthImageMessage, height, width);
      MessageTools.toMessage(now, depthImageMessage.getAcquisitionTime());
      ros2Helper.publish(topic, depthImageMessage);
   }

   private void publishCompressedColor(Mat color8UC3Image, Instant now, int height, int width, ROS2Topic<ImageMessage> topic)
   {
      compressedColorPointer = new BytePointer();
      BytedecoOpenCVTools.compressRGBImageJPG(color8UC3Image, yuvColorImage, compressedColorPointer);
      BytedecoOpenCVTools.fillImageMessage(compressedColorPointer, colorImageMessage, height, width);
      MessageTools.toMessage(now, colorImageMessage.getAcquisitionTime());
      ros2Helper.publish(topic, colorImageMessage);
   }

   private void publishPlanarRegionsWithPose(BytedecoImage depthImage, ReferenceFrame cameraFrame, ROS2Topic<PlanarRegionsListWithPoseMessage> topic)
   {
      PlanarRegionsListWithPose planarRegionsListWithPose = new PlanarRegionsListWithPose();
      rapidPlanarRegionsExtractor.update(depthImage, true);
      rapidPlanarRegionsCustomizer.createCustomPlanarRegionsList(rapidPlanarRegionsExtractor.getGPUPlanarRegions(), cameraFrame, planarRegionsListWithPose);
      ros2Helper.publish(topic, PlanarRegionMessageConverter.convertToPlanarRegionsListWithPoseMessage(planarRegionsListWithPose));
   }

   private void setDepthExtrinsics(BytedecoRealsense sensor, IntrinsicParametersMessage intrinsicParametersMessage)
   {
      intrinsicParametersMessage.setFx(sensor.getDepthFocalLengthPixelsX());
      intrinsicParametersMessage.setFy(sensor.getDepthFocalLengthPixelsY());
      intrinsicParametersMessage.setSkew(0.0);
      intrinsicParametersMessage.setCx(sensor.getDepthPrincipalOffsetXPixels());
      intrinsicParametersMessage.setCy(sensor.getDepthPrincipalOffsetYPixels());
   }

   private void setColorExtrinsics(BytedecoRealsense sensor, IntrinsicParametersMessage intrinsicParametersMessage)
   {
      intrinsicParametersMessage.setFx(sensor.getColorFocalLengthPixelsX());
      intrinsicParametersMessage.setFy(sensor.getColorFocalLengthPixelsY());
      intrinsicParametersMessage.setSkew(0.0);
      intrinsicParametersMessage.setCx(sensor.getColorPrincipalOffsetXPixels());
      intrinsicParametersMessage.setCy(sensor.getColorPrincipalOffsetYPixels());
   }

   private void display(Mat depth, Mat color, int delay)
   {
      Mat depthDisplay = new Mat();
      BytedecoOpenCVTools.clampTo8BitUnsignedChar(depth, depthDisplay, 0.0, 255.0);
      BytedecoOpenCVTools.convert8BitGrayTo8BitRGBA(depthDisplay, depthDisplay);

      imshow("Depth", depthDisplay);
      imshow("Color", color);
      int code = waitKeyEx(delay);
      if (code == 113)
      {
         running = false;
      }
   }

   private void destroy()
   {
      running = false;
      sensor.deleteDevice();
      depth16UC1Image.destroy(openCLManager);
      realSenseHardwareManager.deleteContext();
   }

   public void setCollisionFilter(CollisionBoxProvider collisionBoxProvider, FullHumanoidRobotModel fullRobotModel)
   {
      CollisionShapeTester shapeTester = new CollisionShapeTester();
      for (RobotSide robotSide : RobotSide.values)
      {
         List<JointBasics> joints = new ArrayList<>();
         RigidBodyBasics shin = fullRobotModel.getFoot(robotSide).getParentJoint().getPredecessor().getParentJoint().getPredecessor();
         MultiBodySystemTools.collectJointPath(fullRobotModel.getPelvis(), shin, joints);
         joints.forEach(joint -> shapeTester.addJoint(collisionBoxProvider, joint));
      }
      collisionFilter = new CollidingScanRegionFilter(shapeTester);
   }

   public static void main(String[] args)
   {
      /*
         Color: [fx:901.3026, fy:901.8400, cx:635.2337, cy:350.9427, h:720, w:1280]
         Depth: [fx:730.7891, fy:731.0859, cx:528.6094, cy:408.1602, h:768, w:1024]
      */

      String l515SerialNumber = System.getProperty("l515.serial.number", "F1121365"); // Benchtop L515: F1120592, Tripod: F1121365
      new TerrainPerceptionProcessWithDriver(new RealsenseConfiguration(l515SerialNumber, 768, 1024, 30, true, 720, 1280, 30),
                                             ROS2Tools.L515_DEPTH_IMAGE,
                                             ROS2Tools.L515_COLOR_IMAGE,
                                             ROS2Tools.MAPSENSE_REGIONS_WITH_POSE,
                                             ReferenceFrame::getWorldFrame);
   }
}