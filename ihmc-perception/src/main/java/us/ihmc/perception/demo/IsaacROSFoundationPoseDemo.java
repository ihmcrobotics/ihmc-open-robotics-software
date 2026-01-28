package us.ihmc.perception.demo;

import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.RawImagePublisher;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseCommunicatorMap;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.sensors.zed.ROS2ZEDSVOPlaybackSensor;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.zed.global.zed;

/**
 * Demo for IHMC's usage of Isaac ROS FoundationPose. The code for launching Isaac ROS FoundationPose can be found
 * <a href="https://github.com/ihmcrobotics/ihmc-isaac-ros/tree/main/src/ihmc_isaac_ros_foundationpose">here</a>
 * <p>
 * This demo is intended to run on a Jetson device. Run {@code gradle installDist} to generate the entrypoint
 * of this class and use {@code rsync -r build/install/ihmc-perception user@jetson.address:~} to copy the
 * required libraries and binaries to the Jetson device.
 * <p>
 * Once the files are on the Jetson, SSH into the Jetson and start the following processes:
 * <ul>
 *    <li>This demo, by running {@code ~/ihmc-perception/bin/IsaacROSFoundationPoseDemo}</li>
 *    <li>Isaac ROS FoundationPose, by following instructions found
 *    <a href="https://github.com/ihmcrobotics/ihmc-isaac-ros/blob/main/src/ihmc_isaac_ros_foundationpose/README.md">here</a></li>
 * </ul>
 * And run the RDXIsaacROSFoundationPoseDemoUI on any machine that can communicate with the Jetson.
 */
public class IsaacROSFoundationPoseDemo
{
   private static final String SVO_FILE = "/opt/ihmc/LogData/UserFolders/TomaszFolder/20251020_ZEDXMini_DoorChargeBarrierBottle.svo2";

   private final ROS2Node ros2Node = new ROS2NodeBuilder().build(getClass().getSimpleName().toLowerCase());
   private final ROS2PeerClockOffsetEstimator peerClockOffsetEstimator = new ROS2PeerClockOffsetEstimator(ros2Node);

   private final RawImagePublisher imagePublisher = new RawImagePublisher(ros2Node, 0.5);

   private final ZEDImageSensor zedImageSensor;

   private final IsaacROSFoundationPoseCommunicatorMap foundationPoseCommunicators;
   private final YOLOv8DetectionExecutor yoloExecutor;

   private final RepeatingTaskThread taskThread;

   private IsaacROSFoundationPoseDemo()
   {
      zedImageSensor = new ROS2ZEDSVOPlaybackSensor(new ROS2Helper(ros2Node), 0, ZEDModelData.ZED_2I, zed.SL_DEPTH_MODE_NEURAL_LIGHT, SVO_FILE);
//      zedImageSensor = new ZEDImageSensor(0, ZEDModelData.ZED_X_MINI, zed.SL_INPUT_TYPE_GMSL, zed.SL_DEPTH_MODE_NEURAL, zed.SL_RESOLUTION_SVGA, 15);
//      zedImageSensor = new ZEDImageSensor(0, ZEDModelData.ZED_2, zed.SL_INPUT_TYPE_USB, zed.SL_DEPTH_MODE_NEURAL, zed.SL_RESOLUTION_HD720, 15);
      zedImageSensor.enablePositionalTracking(true);
      zedImageSensor.setSensorFrame(zedImageSensor.getTrackedSensorFrame());
      zedImageSensor.run(true);

      foundationPoseCommunicators = new IsaacROSFoundationPoseCommunicatorMap(peerClockOffsetEstimator);

      yoloExecutor = new YOLOv8DetectionExecutor(peerClockOffsetEstimator, () -> true);
      yoloExecutor.addDetectionConsumerCallback(foundationPoseCommunicators::updatePoseEstimations);
      yoloExecutor.disableAllModels();

      taskThread = new RepeatingTaskThread(getClass().getSimpleName() + "Thread", this::task);
      taskThread.startRepeating();

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, getClass().getSimpleName() + "Shutdown"));
   }

   private void task()
   {
      try
      {
         // Wait for the ZED to produce an image
         zedImageSensor.waitForGrab();

         // Get the images
         RawImage color = zedImageSensor.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
         RawImage depth = zedImageSensor.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);

         // Publish for the UI
         imagePublisher.publishImage(PerceptionAPI.EXPERIMENTAL_ZED_DEPTH, depth);
         imagePublisher.publishImage(PerceptionAPI.EXPERIMENTAL_ZED_COLOR.get(RobotSide.LEFT), color);

         // Run YOLO using the color image
         yoloExecutor.runNextEnabledModel(color, depth);

         // Update FoundationPose manager
         foundationPoseCommunicators.updateCommunicators();

         color.release();
         depth.release();
      }
      catch (InterruptedException ignored)
      {
      }
   }

   private void destroy()
   {
      taskThread.blockingKill();
      imagePublisher.close();
      zedImageSensor.close();
      yoloExecutor.destroy();
      foundationPoseCommunicators.closeCommunicators();
      peerClockOffsetEstimator.destroy();
      ros2Node.destroy();
   }

   public static void main(String[] args)
   {
      new IsaacROSFoundationPoseDemo();
   }
}
