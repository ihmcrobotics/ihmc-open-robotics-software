package us.ihmc.lerobot;

import behavior_msgs.msg.dds.VisuomotorOperationMessage;
import std_msgs.msg.dds.Float32MultiArray;
import std_msgs.msg.dds.Int32;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.commons.time.FrequencyCalculator;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ToolboxAPIs;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.communication.packets.ToolboxState;
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
 * Autonomy process thread for managing visuomotor inference and supporting remote UI.
 * Manages communication with the Python side, which is running the LeRobot code
 * with pytorch inference of the visuomotor policy. We use a ROS 2 API to interface with it.
 */
public class VisuomotorPolicyUpdateThread extends RepeatingTaskThread
{
   private static final ROS2Topic<?> VISUOMOTOR = new ROS2Topic<>().withPrefix("lerobot");
   /** 0: stop, 1: pause, 2: run */
   private static final ROS2Topic<std_msgs.msg.dds.Int32> COMMAND = VISUOMOTOR.withSuffix("command").withType(std_msgs.msg.dds.Int32.class);
   private static final ROS2Topic<std_msgs.msg.dds.String> STATUS = VISUOMOTOR.withSuffix("status").withType(std_msgs.msg.dds.String.class);
   private static final ROS2Topic<Float32MultiArray> STATE = VISUOMOTOR.withSuffix("state").withType(Float32MultiArray.class);
   private static final ROS2Topic<Float32MultiArray> ACTION = VISUOMOTOR.withSuffix("action").withType(Float32MultiArray.class);

   public static final ROS2IOTopicPair<VisuomotorOperationMessage> OPERATOR_UI
         = new ROS2IOTopicPair<>(new ROS2Topic<>().withPrefix("lerobot_ui").withTypeName(VisuomotorOperationMessage.class));

   private final ROS2SyncedRobotModel syncedRobot;
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

   private final ROS2Node ros2Node = new ROS2NodeBuilder().build("visuomotor_update_thread");
   private final LatestTimestampModifiable latestTimestampModifiable;
   private long sequenceID = 0L;
   private final CRDTBidirectionalBoolean running;
   private final CRDTBidirectionalBoolean controlRobot;
   private final TypedNotification<VisuomotorOperationMessage> uiCommandSubscription;
   private final ROS2Publisher<VisuomotorOperationMessage> uiStatusPublisher;

   private final ROS2Publisher<KinematicsStreamingToolboxInputMessage> kstInputPublisher;
   private final ROS2Publisher<ToolboxStateMessage> kstStatePublisher;

   public VisuomotorPolicyUpdateThread(ROS2PeerClockOffsetEstimator clockOffsetEstimator, DRCRobotModel robotModel, ROS2SyncedRobotModel syncedRobot)
   {
      super(VisuomotorPolicyUpdateThread.class.getSimpleName());

      this.syncedRobot = syncedRobot;

      setFrequencyLimit(120.0);

      actionHandPoses.forEach(Pose3D::setToNaN);
      actionForearmPoses.forEach(Pose3D::setToNaN);

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

      kstInputPublisher = ros2Node.createPublisher(ToolboxAPIs.getIKStreamingInputTopic(robotModel.getSimpleRobotName()));
      kstStatePublisher = ros2Node.createPublisher(ToolboxAPIs.getIKStreamingStateTopic(robotModel.getSimpleRobotName()));
   }

   @Override
   protected void runTask()
   {
      if (uiCommandSubscription.poll())
      {
         VisuomotorOperationMessage uiCommand = uiCommandSubscription.read();
         latestTimestampModifiable.fromMessage(uiCommand.getLatestTimestampModifiable());
         boolean wasRunning = running.getValue();
         running.fromMessage(uiCommand.getRunning());
         if (!wasRunning && running.getValue())
         {
            ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
            toolboxStateMessage.setRequestedToolboxState(ToolboxState.WAKE_UP.toByte());
            kstStatePublisher.publish(toolboxStateMessage);
         }
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

         KinematicsStreamingToolboxInputMessage ikInputMessage = new KinematicsStreamingToolboxInputMessage();
         ikInputMessage.setStreamToController(controlRobot.getValue());
         ikInputMessage.setTimestamp(actionTimestampNanos);
         for (RobotSide side : RobotSide.values)
         {
            KinematicsToolboxRigidBodyMessage rigidBodyMessage = new KinematicsToolboxRigidBodyMessage();
            rigidBodyMessage.setEndEffectorHashCode(syncedRobot.getFullRobotModel().getHand(side).hashCode());
            rigidBodyMessage.getDesiredPositionInWorld().set(actionHandPoses.get(side).getTranslation());
            rigidBodyMessage.getDesiredOrientationInWorld().set(actionHandPoses.get(side).getRotation());
            rigidBodyMessage.getAngularWeightMatrix().setXWeight(0.02);
            rigidBodyMessage.getAngularWeightMatrix().setYWeight(0.02);
            rigidBodyMessage.getAngularWeightMatrix().setZWeight(0.02);
            ikInputMessage.getInputs().add().set(rigidBodyMessage);

            rigidBodyMessage = new KinematicsToolboxRigidBodyMessage();
            rigidBodyMessage.setEndEffectorHashCode(syncedRobot.getFullRobotModel().getForearm(side).hashCode());
            rigidBodyMessage.getDesiredPositionInWorld().set(actionForearmPoses.get(side).getTranslation());
            rigidBodyMessage.getLinearSelectionMatrix().setXSelected(false); // Disable position tracking for forearm
            rigidBodyMessage.getLinearSelectionMatrix().setYSelected(false);
            rigidBodyMessage.getLinearSelectionMatrix().setZSelected(false);
            rigidBodyMessage.getDesiredOrientationInWorld().set(actionForearmPoses.get(side).getRotation());
            rigidBodyMessage.getAngularWeightMatrix().setXWeight(0.01);
            rigidBodyMessage.getAngularWeightMatrix().setYWeight(0.01);
            rigidBodyMessage.getAngularWeightMatrix().setZWeight(0.001);
            ikInputMessage.getInputs().add().set(rigidBodyMessage);
         }
         kstInputPublisher.publish(ikInputMessage);
      }

      VisuomotorOperationMessage uiStatus = new VisuomotorOperationMessage();
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
