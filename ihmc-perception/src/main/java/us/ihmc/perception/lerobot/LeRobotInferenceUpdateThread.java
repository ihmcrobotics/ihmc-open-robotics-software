package us.ihmc.perception.lerobot;

import behavior_msgs.msg.dds.LerobotInferenceOperationMessage;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.ROS2IOTopicPair;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensors.ImageSensor;

import java.util.List;

/**
 * Autonomy process thread for managing a {@link LeRobotInferenceManager} and supporting remote UI.
 */
public class LeRobotInferenceUpdateThread extends RepeatingTaskThread
{
   public static final boolean USE_HAND_POSES = false;

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
   private final List<String> armJointNames;
   private long sequenceID = 0L;
   private final CRDTBidirectionalBoolean running;
   private final CRDTBidirectionalBoolean controlRobot;
   private final TypedNotification<LerobotInferenceOperationMessage> commandSubscription;
   private final ROS2Publisher<LerobotInferenceOperationMessage> statusPublisher;

   public LeRobotInferenceUpdateThread(String policyName,
                                       ROS2PeerClockOffsetEstimator clockOffsetEstimator,
                                       String robotName,
                                       FullHumanoidRobotModel fullRobotModel,
                                       Object fullRobotModelSync,
                                       FullHumanoidRobotModel forwardKinematicsModel,
                                       HumanoidJointNameMap jointMap,
                                       ImageSensor zedSensor)
   {
      super(LeRobotInferenceUpdateThread.class.getSimpleName());

      this.fullRobotModel = fullRobotModel;
      this.fullRobotModelSync = fullRobotModelSync;
      this.zedSensor = zedSensor;

      armJointNames = jointMap.getArmJointNamesAsStrings();

      latestTimestampModifiable = new LatestTimestampModifiable(new CRDTInfo(ROS2ActorDesignation.ROBOT, clockOffsetEstimator));
      latestTimestampModifiable.modify(); // On startup, we want the initial state to propagate
      running = new CRDTBidirectionalBoolean(latestTimestampModifiable, false);
      controlRobot = new CRDTBidirectionalBoolean(latestTimestampModifiable, false);

      leRobotInferenceManager = new LeRobotInferenceManager(policyName,
                                                            robotName,
                                                            fullRobotModel,
                                                            fullRobotModelSync,
                                                            forwardKinematicsModel,
                                                            ros2Node,
                                                            USE_HAND_POSES,
                                                            armJointNames);
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
         controlRobot.fromMessage(command.getControlRobot());
      }

      try
      {
         //TODO: Look at this and all zedSensor for LeRobot
         zedSensor.waitForGrab();

         leRobotInferenceManager.publishState(messageData ->
         {
            synchronized (fullRobotModelSync)
            {
               if (USE_HAND_POSES)
               {
                  leftPose.set(fullRobotModel.getHand(RobotSide.LEFT).getParentJoint().getFrameAfterJoint().getTransformToWorldFrame());
                  rightPose.set(fullRobotModel.getHand(RobotSide.RIGHT).getParentJoint().getFrameAfterJoint().getTransformToWorldFrame());

                  for (RobotSide side : RobotSide.values)
                  {
                     messageData.add((side == RobotSide.LEFT ? leftPose : rightPose).getPosition().getX32());
                     messageData.add((side == RobotSide.LEFT ? leftPose : rightPose).getPosition().getY32());
                     messageData.add((side == RobotSide.LEFT ? leftPose : rightPose).getPosition().getZ32());
                     messageData.add((side == RobotSide.LEFT ? leftPose : rightPose).getOrientation().getX32());
                     messageData.add((side == RobotSide.LEFT ? leftPose : rightPose).getOrientation().getY32());
                     messageData.add((side == RobotSide.LEFT ? leftPose : rightPose).getOrientation().getZ32());
                     messageData.add((side == RobotSide.LEFT ? leftPose : rightPose).getOrientation().getS32());
                  }
               }
               else
               {
                  for (String armJointName : armJointNames)
                  {
                     messageData.add((float) fullRobotModel.getOneDoFJointByName(armJointName).getQ());
                  }
               }
            }
         });
         leRobotInferenceManager.setRunning(running.getValue());
         leRobotInferenceManager.getIKStreaming().setControlRobot(controlRobot.getValue());
         leRobotInferenceManager.update();
      }
      catch (InterruptedException ex) { } // Ignore

      LerobotInferenceOperationMessage status = new LerobotInferenceOperationMessage();
      latestTimestampModifiable.toMessage(status.getLatestTimestampModifiable());
      status.setSequenceId(sequenceID++);
      status.setRunning(running.toMessage());
      status.setControlRobot(controlRobot.toMessage());
      status.setPythonStatusFrequency(leRobotInferenceManager.getStatusFrequency());
      status.setPythonStatusMessage(leRobotInferenceManager.getStatusMessage());
      status.setReceivedActions(leRobotInferenceManager.getNumberOfActionsReceived());
      statusPublisher.publish(status);
   }
   public void destroy()
   {
      blockingKill();
      leRobotInferenceManager.destroy();
      ros2Node.destroy();
   }
}
