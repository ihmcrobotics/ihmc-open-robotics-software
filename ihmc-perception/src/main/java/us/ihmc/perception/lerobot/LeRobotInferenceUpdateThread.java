package us.ihmc.perception.lerobot;

import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.perception.RawImage;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.sensors.zed.ZEDImageSensor;

public class LeRobotInferenceUpdateThread extends RepeatingTaskThread
{
   private static final ROS2Topic<?> LEROBOT_UI = new ROS2Topic<>().withPrefix("lerobot_ui");
   private static final ROS2Topic<std_msgs.msg.dds.Bool> RUNNING = LEROBOT_UI.withSuffix("running").withType(std_msgs.msg.dds.Bool.class);

//   private final LatestTimestampModifiable latestTimestampModifiable;
//   private final CRDTBidirectionalBoolean running;

   private final LeRobotInferenceManager leRobotInferenceManager;
   private final FullHumanoidRobotModel fullRobotModel;
   private final Object fullRobotModelSync;
   private final ImageSensor zedSensor;
   private final Pose3D leftPose = new Pose3D();
   private final Pose3D rightPose = new Pose3D();

   public LeRobotInferenceUpdateThread(String policyName,
                                       ROS2Node ros2Node,
                                       ROS2PeerClockOffsetEstimator clockOffsetEstimator,
                                       String robotName,
                                       FullHumanoidRobotModel fullRobotModel,
                                       Object fullRobotModelSync,
                                       ImageSensor zedSensor)
   {
      super(LeRobotInferenceUpdateThread.class.getSimpleName());

      this.fullRobotModel = fullRobotModel;
      this.fullRobotModelSync = fullRobotModelSync;
      this.zedSensor = zedSensor;

      setFrequencyLimit(20); // TODO: Pick an appropriate frequency

//      CRDTInfo crdtInfo = new CRDTInfo(ROS2ActorDesignation.ROBOT, clockOffsetEstimator);
//      latestTimestampModifiable = new LatestTimestampModifiable(crdtInfo);
//      running = new CRDTBidirectionalBoolean(latestTimestampModifiable, false);

      leRobotInferenceManager = new LeRobotInferenceManager(policyName, robotName, fullRobotModel);
      leRobotInferenceManager.setRunning(true); // TODO: Set from ROS 2 callback on new topic that talks to UI
      leRobotInferenceManager.startPythonServer();

      ros2Node.createSubscription2(RUNNING, message -> leRobotInferenceManager.setRunning(message.getData()));
   }

   @Override
   protected void runTask() throws Throwable
   {
//      latestTimestampModifiable.checkModified();
//      running.

      try
      {
         zedSensor.waitForGrab();

         synchronized (fullRobotModelSync)
         {
            leftPose.set(fullRobotModel.getHand(RobotSide.LEFT).getParentJoint().getFrameAfterJoint().getTransformToWorldFrame());
            rightPose.set(fullRobotModel.getHand(RobotSide.RIGHT).getParentJoint().getFrameAfterJoint().getTransformToWorldFrame());
         }
         leRobotInferenceManager.publishHandPoses(leftPose, rightPose);

         RawImage leftBGRAImage = zedSensor.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
         RawImage rightBGRAImage = zedSensor.getImage(ZEDImageSensor.RIGHT_COLOR_IMAGE_KEY);

         leRobotInferenceManager.publishImage(RobotSide.LEFT, leftBGRAImage.getCpuImageMat());
         leRobotInferenceManager.publishImage(RobotSide.RIGHT, rightBGRAImage.getCpuImageMat());

         leftBGRAImage.release();
         rightBGRAImage.release();

         leRobotInferenceManager.update();
      }
      catch (InterruptedException ex) { } // Ignore
   }

   public void destroy()
   {
      blockingKill();
      leRobotInferenceManager.destroy();
   }
}
