package us.ihmc.avatar.gpuPlanarRegions;

import boofcv.struct.calib.CameraPinholeBrown;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.StereoPointCloudCompression;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.MutableBytePointer;
import us.ihmc.perception.realsense.BytedecoRealsense;
import us.ihmc.perception.realsense.RealSenseHardwareManager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.time.Instant;

/**
 * This class is designed to just publish Realsense L515 data without doing any extra perception processing,
 * such as planar regions or SLAM.
 */
public class L515OnRobotProcess
{
   private static final String SERIAL_NUMBER = System.getProperty("l515.serial.number", "F0000000");

   private final PausablePeriodicThread thread;
   private final Activator nativesLoadedActivator;
   private final RealtimeROS2Node realtimeROS2Node;
   private final IHMCRealtimeROS2Publisher<StereoVisionPointCloudMessage> ros2PointCloudPublisher;
//   private final IHMCROS2Input<RobotFrameData> sensorFrameData;
   private final ROS2Helper ros2Helper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final FramePose3D cameraPose = new FramePose3D();
   private final FramePoint3D framePoint = new FramePoint3D();
   private final ROS2Topic<StereoVisionPointCloudMessage> pointCloudTopic;
   private RealSenseHardwareManager realSenseHardwareManager;
   private BytedecoRealsense l515;
   private Mat depthU16C1Image;
   private BytedecoImage depth32FC1Image;
   private int depthWidth;
   private int depthHeight;
   private CameraPinholeBrown depthCameraIntrinsics;
   private Point3D[] points;
   private int[] colors;

   public L515OnRobotProcess(DRCRobotModel robotModel)
   {
      nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();

      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "l515_point_cloud_realtime_node");

      pointCloudTopic = ROS2Tools.IHMC_ROOT.withTypeName(StereoVisionPointCloudMessage.class);
      LogTools.info("Publishing ROS 2 stereo vision: {}", pointCloudTopic);
      ros2PointCloudPublisher = ROS2Tools.createPublisher(realtimeROS2Node, pointCloudTopic, ROS2QosProfile.DEFAULT());
      realtimeROS2Node.spin();

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "l515_point_cloud_node");
      ros2Helper = new ROS2Helper(ros2Node);
      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);

      // TODO: Controller should publish sensor frames
//      ROS2Topic<RobotFrameData> sensorFrameTopic = RobotFrameDataPublisher.getTopic("", "steppingCamera");
//      sensorFrameData = ros2Helper.subscribe(sensorFrameTopic);

      thread = new PausablePeriodicThread("L515Node", UnitConversions.hertzToSeconds(2.0), 1, false, this::update);
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

               depthCameraIntrinsics.setFx(l515.getDepthFocalLengthPixelsX());
               depthCameraIntrinsics.setFy(l515.getDepthFocalLengthPixelsY());
               depthCameraIntrinsics.setSkew(0.0);
               depthCameraIntrinsics.setCx(l515.getDepthPrincipalOffsetXPixels());
               depthCameraIntrinsics.setCy(l515.getDepthPrincipalOffsetYPixels());
            }

            depthU16C1Image.convertTo(depth32FC1Image.getBytedecoOpenCVMat(), opencv_core.CV_32FC1, l515.getDepthToMeterConversion(), 0.0);

//            Pose3D sensorPose = sensorFrameData.getLatest().getFramePoseInWorld();
//            cameraTransformToWorld.set(sensorPose);
//            cameraFrame.update();
            // TODO: Wait for frame at time of data aquisition?

            syncedRobot.update();
            ReferenceFrame cameraFrame =
                  syncedRobot.hasReceivedFirstMessage() ? syncedRobot.getReferenceFrames().getSteppingCameraFrame() : ReferenceFrame.getWorldFrame();
            cameraPose.setToZero(cameraFrame);
            cameraPose.changeFrame(ReferenceFrame.getWorldFrame());

            // TODO: Put in OpenCL
            for (int x = 0; x < depthWidth; x++)
            {
               for (int y = 0; y < depthHeight; y++)
               {
                  float eyeDepth = depth32FC1Image.getFloat(x, y);
                  framePoint.setToZero(cameraFrame);
                  framePoint.setX(eyeDepth);
                  framePoint.setY(-(x - depthCameraIntrinsics.getCx()) / l515.getDepthFocalLengthPixelsX() * eyeDepth);
                  framePoint.setZ(-(y - depthCameraIntrinsics.getCy()) / l515.getDepthFocalLengthPixelsY() * eyeDepth);
                  framePoint.changeFrame(ReferenceFrame.getWorldFrame());
                  Point3D point = points[y * depthWidth + x];
                  point.set(framePoint);
               }
            }

            double minimumResolution = 0.005;
            StereoVisionPointCloudMessage message = StereoPointCloudCompression.compressPointCloud(nanoTime,
                                                                                                   points,
                                                                                                   colors,
                                                                                                   points.length,
                                                                                                   minimumResolution,
                                                                                                   null);
            message.getSensorPosition().set(cameraPose.getPosition());
            message.getSensorOrientation().set(cameraPose.getOrientation());
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
