package us.ihmc.perception.lerobot;

import behavior_msgs.msg.dds.LerobotInferenceOperationMessage;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTStatusLong;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.ROS2IOTopicPair;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.perception.RawImage;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.sensors.zed.ZEDImageSensor;

public class LeRobotInferenceUpdateThread extends RepeatingTaskThread
{
   public static final ROS2IOTopicPair<LerobotInferenceOperationMessage> LEROBOT_UI
         = new ROS2IOTopicPair<>(new ROS2Topic<>().withPrefix("lerobot_ui").withTypeName(LerobotInferenceOperationMessage.class));
   public static final double HZ = 20.0; // TODO: Pick an appropriate frequency

   private final LeRobotInferenceManager leRobotInferenceManager;
   private final FullHumanoidRobotModel fullRobotModel;
   private final Object fullRobotModelSync;
   private final ImageSensor zedSensor;
   private final Pose3D leftPose = new Pose3D();
   private final Pose3D rightPose = new Pose3D();

   private final ROS2Node ros2Node = new ROS2NodeBuilder().build("lerobot_update_thread");
   private final LatestTimestampModifiable latestTimestampModifiable;
   private final CRDTStatusLong sequenceID;
   private final CRDTBidirectionalBoolean running;
   private final CRDTBidirectionalBoolean controlRobot;
   private final TypedNotification<LerobotInferenceOperationMessage> commandSubscription;
   private final ROS2Publisher<LerobotInferenceOperationMessage> statusPublisher;

   public LeRobotInferenceUpdateThread(String policyName,
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

      latestTimestampModifiable = new LatestTimestampModifiable(new CRDTInfo(ROS2ActorDesignation.ROBOT, clockOffsetEstimator));
      latestTimestampModifiable.modify(); // On startup, we want the initial state to propagate
      sequenceID = new CRDTStatusLong(ROS2ActorDesignation.ROBOT, latestTimestampModifiable.getCRDTInfo(), 0L);
      running = new CRDTBidirectionalBoolean(latestTimestampModifiable, false);
      controlRobot = new CRDTBidirectionalBoolean(latestTimestampModifiable, false);
      // TODO finish up these data sync comms

      leRobotInferenceManager = new LeRobotInferenceManager(policyName, robotName, fullRobotModel, ros2Node);
      leRobotInferenceManager.startPythonServer();

      commandSubscription = ROS2Tools.createNotificationSubscription(ros2Node, LEROBOT_UI.getTopic(ROS2ActorDesignation.ROBOT.getIncomingQualifier()));
      statusPublisher = ros2Node.createPublisher(LEROBOT_UI.getTopic(ROS2ActorDesignation.ROBOT.getOutgoingQualifier()));
   }

   @Override
   protected void runTask() throws Throwable
   {
      if (commandSubscription.poll())
      {
         LerobotInferenceOperationMessage command = commandSubscription.read();
         latestTimestampModifiable.fromMessage(command.getLatestTimestampModifiable());
         running.fromMessage(command.getRunning());

         leRobotInferenceManager.setRunning(running.getValue());
      }

      try
      {
         //TODO: Look at this and all zedSensor for LeRobot
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

         leRobotInferenceManager.setRunning(running.getValue());
         leRobotInferenceManager.update();
      }
      catch (InterruptedException ex) { } // Ignore

      LerobotInferenceOperationMessage status = new LerobotInferenceOperationMessage();
      latestTimestampModifiable.toMessage(status.getLatestTimestampModifiable());
      status.setRunning(running.toMessage());
      statusPublisher.publish(status);
   }
   public void destroy()
   {
      blockingKill();
      leRobotInferenceManager.destroy();
      ros2Node.destroy();
   }
}
