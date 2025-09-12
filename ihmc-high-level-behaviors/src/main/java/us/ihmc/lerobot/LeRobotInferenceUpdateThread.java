package us.ihmc.lerobot;

import behavior_msgs.msg.dds.LerobotInferenceOperationMessage;
import std_msgs.msg.dds.Float32MultiArray;
import std_msgs.msg.dds.Int32;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.commons.time.FrequencyCalculator;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.ROS2IOTopicPair;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.idl.IDLSequence;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Topic;

/**
 * Autonomy process thread for managing a LeRobot inference and supporting remote UI.
 * Manages communication with the Python side, which is running the LeRobot code
 * with pytorch inference of the visuomotor policy. We use a ROS 2 API to interface with it.
 */
public class LeRobotInferenceUpdateThread extends RepeatingTaskThread
{
   private static final ROS2Topic<?> LEROBOT = new ROS2Topic<>().withPrefix("lerobot");
   /** 0: stop, 1: pause, 2: run */
   private static final ROS2Topic<std_msgs.msg.dds.Int32> COMMAND = LEROBOT.withSuffix("command").withType(std_msgs.msg.dds.Int32.class);
   private static final ROS2Topic<std_msgs.msg.dds.String> STATUS = LEROBOT.withSuffix("status").withType(std_msgs.msg.dds.String.class);
   private static final ROS2Topic<Float32MultiArray> STATE = LEROBOT.withSuffix("state").withType(Float32MultiArray.class);
   private static final ROS2Topic<Float32MultiArray> ACTION = LEROBOT.withSuffix("action").withType(Float32MultiArray.class);

   public static final ROS2IOTopicPair<LerobotInferenceOperationMessage> OPERATOR_UI
         = new ROS2IOTopicPair<>(new ROS2Topic<>().withPrefix("lerobot_ui").withTypeName(LerobotInferenceOperationMessage.class));

   private final ROS2SyncedRobotModel syncedRobot;
   private final LeRobotIKStreaming ikStreaming;
   private final std_msgs.msg.dds.Int32 command = new std_msgs.msg.dds.Int32();
   private String status = "Python not started";
   private final Float32MultiArray stateMessage = new Float32MultiArray();
   private final FrequencyCalculator statusFrequency = new FrequencyCalculator();
   private final FramePose3D framePose = new FramePose3D();
   private final SideDependentList<Pose3D> stateHandPoses = new SideDependentList<>(new Pose3D(), new Pose3D());
   private final SideDependentList<Pose3D> stateForearmPoses = new SideDependentList<>(new Pose3D(), new Pose3D());
   private final SideDependentList<Pose3D> actionHandPoses = new SideDependentList<>(new Pose3D(), new Pose3D());
   private final SideDependentList<Pose3D> actionForearmPoses = new SideDependentList<>(new Pose3D(), new Pose3D());
   private long actionTimestampNanos = 0L;
   private long numberOfActionsReceived = 0L;
   private final ROS2Publisher<Int32> commandPublisher;
   private final ROS2Publisher<Float32MultiArray> statePublisher;
   private final TypedNotification<Float32MultiArray> actionSubscription;

   private final ROS2Node ros2Node = new ROS2NodeBuilder().build("lerobot_update_thread");
   private final LatestTimestampModifiable latestTimestampModifiable;
   private long sequenceID = 0L;
   private final CRDTBidirectionalBoolean running;
   private final CRDTBidirectionalBoolean controlRobot;
   private final TypedNotification<LerobotInferenceOperationMessage> uiCommandSubscription;
   private final ROS2Publisher<LerobotInferenceOperationMessage> uiStatusPublisher;

   public LeRobotInferenceUpdateThread(ROS2PeerClockOffsetEstimator clockOffsetEstimator, DRCRobotModel robotModel, ROS2SyncedRobotModel syncedRobot)
   {
      super(LeRobotInferenceUpdateThread.class.getSimpleName());

      this.syncedRobot = syncedRobot;

      setFrequencyLimit(120.0);

      actionHandPoses.forEach(Pose3D::setToNaN);
      actionForearmPoses.forEach(Pose3D::setToNaN);
      ikStreaming = new LeRobotIKStreaming(ros2Node, robotModel, syncedRobot);

      commandPublisher = ros2Node.createPublisher(COMMAND);
      statePublisher = ros2Node.createPublisher(STATE);
      ros2Node.createSubscription2(STATUS, message ->
      {
         status = message.getDataAsString();
         statusFrequency.ping();
      });
      actionSubscription = ROS2Tools.createNotificationSubscription(ros2Node, ACTION);

      latestTimestampModifiable = new LatestTimestampModifiable(new CRDTInfo(ROS2ActorDesignation.ROBOT, clockOffsetEstimator));
      latestTimestampModifiable.modify(); // On startup, we want the initial state to propagate
      running = new CRDTBidirectionalBoolean(latestTimestampModifiable, false);
      controlRobot = new CRDTBidirectionalBoolean(latestTimestampModifiable, false);

      uiCommandSubscription = ROS2Tools.createNotificationSubscription(ros2Node, OPERATOR_UI.getTopic(ROS2ActorDesignation.ROBOT.getIncomingQualifier()));
      uiStatusPublisher = ros2Node.createPublisher(OPERATOR_UI.getTopic(ROS2ActorDesignation.ROBOT.getOutgoingQualifier()));
   }

   @Override
   protected void runTask()
   {
      if (uiCommandSubscription.poll())
      {
         LerobotInferenceOperationMessage uiCommand = uiCommandSubscription.read();
         latestTimestampModifiable.fromMessage(uiCommand.getLatestTimestampModifiable());
         boolean wasRunning = running.getValue();
         running.fromMessage(uiCommand.getRunning());
         if (!wasRunning && running.getValue())
            ikStreaming.wakeUp();
         controlRobot.fromMessage(uiCommand.getControlRobot());
      }

      IDLSequence.Float messageData = stateMessage.getData();
      messageData.resetQuick();
      synchronized (syncedRobot)
      {
         for (RobotSide side : RobotSide.values)
         {
            Pose3D stateHandPose = stateHandPoses.get(side);
            framePose.setToZero(syncedRobot.getFullRobotModel().getHand(side).getParentJoint().getFrameAfterJoint());
            framePose.changeFrame(syncedRobot.getReferenceFrames().getPelvisFrame());
            stateHandPose.set(framePose);
            messageData.add(stateHandPose.getPosition().getX32());
            messageData.add(stateHandPose.getPosition().getY32());
            messageData.add(stateHandPose.getPosition().getZ32());
            messageData.add(stateHandPose.getOrientation().getX32());
            messageData.add(stateHandPose.getOrientation().getY32());
            messageData.add(stateHandPose.getOrientation().getZ32());
            messageData.add(stateHandPose.getOrientation().getS32());
            Pose3D stateForearmPose = stateForearmPoses.get(side);
            framePose.setToZero(syncedRobot.getFullRobotModel().getForearm(side).getParentJoint().getFrameAfterJoint());
            framePose.changeFrame(syncedRobot.getReferenceFrames().getPelvisFrame());
            stateForearmPose.set(framePose);
            messageData.add(stateForearmPose.getPosition().getX32());
            messageData.add(stateForearmPose.getPosition().getY32());
            messageData.add(stateForearmPose.getPosition().getZ32());
            messageData.add(stateForearmPose.getOrientation().getX32());
            messageData.add(stateForearmPose.getOrientation().getY32());
            messageData.add(stateForearmPose.getOrientation().getZ32());
            messageData.add(stateForearmPose.getOrientation().getS32());
         }
      }
      statePublisher.publish(stateMessage);

      ikStreaming.setControlRobot(controlRobot.getValue());

      command.setData(running.getValue() ? 2 : 1);
      commandPublisher.publish(command);

      if (actionSubscription.poll())
      {
         actionTimestampNanos = System.nanoTime(); // TODO: Get this from the diffusion policy on the python side?
         ++numberOfActionsReceived;

         synchronized (syncedRobot)
         {
            IDLSequence.Float data = actionSubscription.read().getData();
            int i = 0;
            for (RobotSide side : RobotSide.values)
            {
               framePose.setToZero(syncedRobot.getReferenceFrames().getPelvisFrame());
               framePose.getPosition().set(data.get(i++), data.get(i++), data.get(i++));
               framePose.getOrientation().set(data.get(i++), data.get(i++), data.get(i++), data.get(i++));
               framePose.changeFrame(ReferenceFrame.getWorldFrame());
               actionHandPoses.get(side).set(framePose);
               framePose.setToZero(syncedRobot.getReferenceFrames().getPelvisFrame());
               framePose.getPosition().set(data.get(i++), data.get(i++), data.get(i++));
               framePose.getOrientation().set(data.get(i++), data.get(i++), data.get(i++), data.get(i++));
               framePose.changeFrame(ReferenceFrame.getWorldFrame());
               actionForearmPoses.get(side).set(framePose);
            }
         }

         ikStreaming.update(actionTimestampNanos, actionHandPoses, actionForearmPoses);
      }

      LerobotInferenceOperationMessage uiStatus = new LerobotInferenceOperationMessage();
      latestTimestampModifiable.toMessage(uiStatus.getLatestTimestampModifiable());
      uiStatus.setSequenceId(sequenceID++);
      uiStatus.setRunning(running.toMessage());
      uiStatus.setControlRobot(controlRobot.toMessage());
      for (RobotSide side : RobotSide.values)
      {
         uiStatus.getActionHandPoses()[side.ordinal()].set(actionHandPoses.get(side));
         uiStatus.getActionForearmPoses()[side.ordinal()].set(actionForearmPoses.get(side));
      }
      uiStatus.setPythonStatusFrequency(statusFrequency.getFrequencyDecaying());
      uiStatus.setPythonStatusMessage(status);
      uiStatus.setReceivedActions(numberOfActionsReceived);
      uiStatusPublisher.publish(uiStatus);
   }

   public void destroy()
   {
      blockingKill();
      ros2Node.destroy();
   }
}
