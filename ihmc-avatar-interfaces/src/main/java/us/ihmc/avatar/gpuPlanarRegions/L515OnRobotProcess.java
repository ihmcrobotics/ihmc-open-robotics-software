package us.ihmc.avatar.gpuPlanarRegions;

import boofcv.struct.calib.CameraPinholeBrown;
import controller_msgs.msg.dds.RobotFrameData;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.StereoPointCloudCompression;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.MutableBytePointer;
import us.ihmc.perception.realsense.BytedecoRealsenseL515;
import us.ihmc.perception.realsense.RealSenseHardwareManager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.communication.producers.RobotFrameDataPublisher;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.time.Instant;

public class L515OnRobotProcess
{
   private static final String SERIAL_NUMBER = System.getProperty("l515.serial.number", "F0000000");
   private static final String ROBOT_NAME = System.getProperty("robot.name");

   private final PausablePeriodicThread thread;
   private final Activator nativesLoadedActivator;
   private final RealtimeROS2Node realtimeROS2Node;
   private final IHMCRealtimeROS2Publisher<StereoVisionPointCloudMessage> ros2PointCloudPublisher;
   private final IHMCROS2Input<RobotFrameData> sensorFrameData;
   private final RigidBodyTransform cameraTransformToWorld = new RigidBodyTransform();
   private final ReferenceFrame cameraFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("steppingCamera",
                                                                                                              ReferenceFrame.getWorldFrame(),
                                                                                                              cameraTransformToWorld);
   private RealSenseHardwareManager realSenseHardwareManager;
   private BytedecoRealsenseL515 l515;
   private Mat depthU16C1Image;
   private BytedecoImage depth32FC1Image;
   private int depthWidth;
   private int depthHeight;
   private CameraPinholeBrown depthCameraIntrinsics;
   private Point3D[] points;
   private int[] colors;

   public L515OnRobotProcess()
   {
      if (ROBOT_NAME == null)
      {
         LogTools.error("-Drobot.name=<name> must be set!");
         System.exit(1);
      }

      nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();

      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "l515_videopub");

      ROS2Topic<StereoVisionPointCloudMessage> pointCloudTopic = ROS2Tools.IHMC_ROOT.withTypeName(StereoVisionPointCloudMessage.class);
      LogTools.info("Publishing ROS 2 stereo vision: {}", pointCloudTopic);
      ros2PointCloudPublisher = ROS2Tools.createPublisher(realtimeROS2Node, pointCloudTopic, ROS2QosProfile.BEST_EFFORT());
      realtimeROS2Node.spin();

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "l515_node");
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

      ROS2Topic<RobotFrameData> sensorFrameTopic = RobotFrameDataPublisher.getTopic(ROBOT_NAME, "steppingCamera");
      sensorFrameData = ros2Helper.subscribe(sensorFrameTopic);

      thread = new PausablePeriodicThread("L515Node", UnitConversions.hertzToSeconds(31.0), 1, false, this::update);
      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "L515Shutdown"));
      thread.start();
   }

   private void update()
   {
      if (nativesLoadedActivator.poll())
      {
         if (nativesLoadedActivator.isNewlyActivated())
         {
            realSenseHardwareManager = new RealSenseHardwareManager();
            l515 = realSenseHardwareManager.createFullFeaturedL515(SERIAL_NUMBER);
            if (l515.getDevice() == null)
            {
               thread.stop();
               throw new RuntimeException("Device not found. Set -Dl515.serial.number=F0000000");
            }
            l515.initialize();

            depthWidth = l515.getDepthWidth();
            depthHeight = l515.getDepthHeight();
            points = new Point3D[depthWidth * depthHeight];
            colors = new int[points.length];
            for (int i = 0; i < points.length; i++)
            {
               points[i] = new Point3D();
            }

            depthCameraIntrinsics = new CameraPinholeBrown();
         }

         if (l515.readFrameData())
         {
            Instant now = Instant.now();
            long nanoTime = System.nanoTime();
            double dataAquisitionTime = Conversions.nanosecondsToSeconds(nanoTime);

            l515.updateDataBytePointers();

            if (depthU16C1Image == null)
            {
               MutableBytePointer depthFrameData = l515.getDepthFrameData();
               depthU16C1Image = new Mat(depthHeight, depthWidth, opencv_core.CV_16UC1, depthFrameData);
               depth32FC1Image = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_32FC1);

               depthCameraIntrinsics.setFx(l515.getFocalLengthPixelsX());
               depthCameraIntrinsics.setFy(l515.getFocalLengthPixelsY());
               depthCameraIntrinsics.setSkew(0.0);
               depthCameraIntrinsics.setCx(l515.getPrincipalOffsetXPixels());
               depthCameraIntrinsics.setCy(l515.getPrincipalOffsetYPixels());
            }

            depthU16C1Image.convertTo(depth32FC1Image.getBytedecoOpenCVMat(), opencv_core.CV_32FC1, l515.getDepthToMeterConversion(), 0.0);

            Pose3D sensorPose = sensorFrameData.getLatest().getFramePoseInWorld();
            cameraTransformToWorld.set(sensorPose);
            cameraFrame.update();
            // TODO: Wait for frame at time of data aquisition?

            int depthFrameDataSize = l515.getDepthFrameDataSize();

            // TODO: Put in OpenCL
            for (int x = 0; x < depthWidth; x++)
            {
               for (int y = 0; y < depthHeight; y++)
               {
                  float eyeDepth = depth32FC1Image.getFloat(x, y);
                  Point3D point = points[y * x + x];
                  point.setX(eyeDepth);
                  point.setY(-(x - depthCameraIntrinsics.getCx()) / l515.getFocalLengthPixelsX() * eyeDepth);
                  point.setZ(-(y - depthCameraIntrinsics.getCy()) / l515.getFocalLengthPixelsY() * eyeDepth);
               }
            }

            double minimumResolution = 0.005;
            StereoVisionPointCloudMessage message = StereoPointCloudCompression.compressPointCloud(nanoTime,
                                                                                                   points,
                                                                                                   colors,
                                                                                                   points.length,
                                                                                                   minimumResolution,
                                                                                                   null);
            message.getSensorPosition().set(sensorPose.getPosition());
            message.getSensorOrientation().set(sensorPose.getOrientation());
            ros2PointCloudPublisher.publish(message);
         }
      }
   }

   private void destroy()
   {
      realtimeROS2Node.destroy();
      l515.deleteDevice();
      realSenseHardwareManager.deleteContext();
   }
}
