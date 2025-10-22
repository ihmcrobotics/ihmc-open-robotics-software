package us.ihmc.perception.demo;

import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.RawImagePublisher;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseManager;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.sensors.zed.ZEDSVOPlaybackSensor;
import us.ihmc.zed.global.zed;

public class IsaacROSFoundationPoseDemo
{
   private static final String SVO_FILE = "/home/robotlab/Downloads/20251020_ZEDXMini_DoorChargeBarrierBottle.svo2";

   private final ROS2Node ros2Node = new ROS2NodeBuilder().build(getClass().getSimpleName().toLowerCase());
   private final ROS2PeerClockOffsetEstimator peerClockOffsetEstimator = new ROS2PeerClockOffsetEstimator(ros2Node);

   private final RawImagePublisher imagePublisher = new RawImagePublisher(ros2Node, 0.5);

   private final ZEDImageSensor zedImageSensor;

   private final IsaacROSFoundationPoseManager foundationPoseManager;
   private final YOLOv8DetectionExecutor yoloExecutor;

   private final RepeatingTaskThread taskThread;

   private IsaacROSFoundationPoseDemo()
   {
      zedImageSensor = new ZEDSVOPlaybackSensor(0, ZEDModelData.ZED_2I, zed.SL_DEPTH_MODE_NEURAL_LIGHT, SVO_FILE);
//      zedImageSensor = new ZEDImageSensor(0, ZEDModelData.ZED_X_MINI, zed.SL_INPUT_TYPE_GMSL, zed.SL_DEPTH_MODE_NEURAL, zed.SL_RESOLUTION_SVGA, 15);
//      zedImageSensor = new ZEDImageSensor(0, ZEDModelData.ZED_2, zed.SL_INPUT_TYPE_USB, zed.SL_DEPTH_MODE_NEURAL, zed.SL_RESOLUTION_HD720, 15);
      zedImageSensor.enablePositionalTracking(true);
      zedImageSensor.setSensorFrame(zedImageSensor.getTrackedSensorFrame());
      zedImageSensor.run(true);

      CRDTInfo crdtInfo = new CRDTInfo(ROS2ActorDesignation.ROBOT, peerClockOffsetEstimator);
      foundationPoseManager = new IsaacROSFoundationPoseManager(crdtInfo);

      yoloExecutor = new YOLOv8DetectionExecutor(new CRDTInfo(ROS2ActorDesignation.ROBOT, peerClockOffsetEstimator), () -> true);
      yoloExecutor.addDetectionConsumerCallback(foundationPoseManager::updatePoseEstimations);
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

         // Update FoundationPose manager
         foundationPoseManager.update();

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
      foundationPoseManager.close();
      peerClockOffsetEstimator.destroy();
      ros2Node.destroy();
   }

   public static void main(String[] args)
   {
      new IsaacROSFoundationPoseDemo();
   }
}
