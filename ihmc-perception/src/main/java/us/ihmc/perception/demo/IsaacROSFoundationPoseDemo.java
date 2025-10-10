package us.ihmc.perception.demo;

import ihmc_common_msgs.msg.dds.Box3DMessage;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.RawImagePublisher;
import us.ihmc.perception.detections.DetectionManager;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseCommunicator;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseObject;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.zed.global.zed;

public class IsaacROSFoundationPoseDemo
{
   public static final ROS2Topic<Box3DMessage> RESULT_TOPIC = new ROS2Topic<>().withModule("ihmc/isaac_ros_foundationpose_demo/output")
                                                                               .withType(Box3DMessage.class);

   private final ROS2Node ros2Node = new ROS2NodeBuilder().build(getClass().getSimpleName().toLowerCase());
   private final ROS2PeerClockOffsetEstimator peerClockOffsetEstimator = new ROS2PeerClockOffsetEstimator(ros2Node);
   private final ROS2Publisher<Box3DMessage> resultPublisher = ros2Node.createPublisher(RESULT_TOPIC);

   private final RawImagePublisher imagePublisher = new RawImagePublisher(ros2Node, 0.5);

   private final ZEDImageSensor zedImageSensor;

   private final DetectionManager detectionManager;
   private final IsaacROSFoundationPoseCommunicator foundationPoseCommunicator;
   private final YOLOv8DetectionExecutor yoloExecutor;

   private final RepeatingTaskThread taskThread;

   private IsaacROSFoundationPoseDemo()
   {
//      zedImageSensor = new ZEDImageSensor(0, ZEDModelData.ZED_X_MINI, zed.SL_INPUT_TYPE_GMSL, zed.SL_DEPTH_MODE_NEURAL, zed.SL_RESOLUTION_SVGA, 15);
      zedImageSensor = new ZEDImageSensor(0, ZEDModelData.ZED_2, zed.SL_INPUT_TYPE_USB, zed.SL_DEPTH_MODE_NEURAL, zed.SL_RESOLUTION_HD720, 15);
      zedImageSensor.enablePositionalTracking(true);
      zedImageSensor.setSensorFrame(zedImageSensor.getTrackedSensorFrame());
      zedImageSensor.run(true);

      detectionManager = new DetectionManager(ros2Node);
      foundationPoseCommunicator = new IsaacROSFoundationPoseCommunicator(IsaacROSFoundationPoseObject.MUSTARD);
      foundationPoseCommunicator.setResetDistance(0.1);
      foundationPoseCommunicator.addResultCallback(box ->
      {
         Box3DMessage message = new Box3DMessage();
         message.getPose().set(box.getPose());
         message.getSize().set(box.getSize());
         resultPublisher.publish(message);
      });

      yoloExecutor = new YOLOv8DetectionExecutor(new CRDTInfo(ROS2ActorDesignation.ROBOT, peerClockOffsetEstimator), () -> true);
      yoloExecutor.addDetectionConsumerCallback(detectionManager::addDetections);
      yoloExecutor.addDetectionConsumerCallback(foundationPoseCommunicator::updateDetections);
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
         imagePublisher.publishImage(PerceptionAPI.ZED_DEPTH, depth);
         imagePublisher.publishImage(PerceptionAPI.ZED_COLOR_IMAGES.get(RobotSide.LEFT), color);

         // Run YOLO using the color image
         yoloExecutor.runNextEnabledModel(color, depth);

         // Update the detection manager
         detectionManager.updateDetections();

         color.release();
         depth.release();
      }
      catch (InterruptedException ignored) {}
   }

   private void destroy()
   {
      taskThread.blockingKill();
      imagePublisher.close();
      zedImageSensor.close();
      yoloExecutor.destroy();
      foundationPoseCommunicator.close();
      peerClockOffsetEstimator.destroy();
      ros2Node.destroy();
   }

   public static void main(String[] args)
   {
      new IsaacROSFoundationPoseDemo();
   }
}
