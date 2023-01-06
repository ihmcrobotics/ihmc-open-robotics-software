package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.*;
import std_msgs.msg.dds.Bool;
import std_msgs.msg.dds.Empty;
import std_msgs.msg.dds.Int32;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.actions.*;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2Topic;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.UUID;

public class BehaviorActionSequence
{
   public static final ROS2Topic<?> ROOT_TOPIC = ROS2Tools.IHMC_ROOT.withRobot("behavior_action_sequence");
   public static final ROS2Topic<?> COMMAND_TOPIC = ROOT_TOPIC.withInput();
   public static final ROS2Topic<?> STATUS_TOPIC = ROOT_TOPIC.withOutput();
   public static final ROS2Topic<ActionSequenceUpdateMessage> SEQUENCE_COMMAND_TOPIC
         = COMMAND_TOPIC.withType(ActionSequenceUpdateMessage.class).withSuffix("sequence_update");
   public static final ROS2Topic<ArmJointAnglesActionMessage> ARM_JOINT_ANGLES_UPDATE_TOPIC
         = COMMAND_TOPIC.withType(ArmJointAnglesActionMessage.class).withSuffix("arm_joint_angles_update");
   public static final ROS2Topic<ChestOrientationActionMessage> CHEST_ORIENTATION_UPDATE_TOPIC
         = COMMAND_TOPIC.withType(ChestOrientationActionMessage.class).withSuffix("chest_orientation_update");
   public static final ROS2Topic<FootstepActionMessage> FOOTSTEP_UPDATE_TOPIC
         = COMMAND_TOPIC.withType(FootstepActionMessage.class).withSuffix("footstep_update");
   public static final ROS2Topic<HandConfigurationActionMessage> HAND_CONFIGURATION_UPDATE_TOPIC
         = COMMAND_TOPIC.withType(HandConfigurationActionMessage.class).withSuffix("hand_configuration_update");
   public static final ROS2Topic<HandPoseActionMessage> HAND_POSE_UPDATE_TOPIC
         = COMMAND_TOPIC.withType(HandPoseActionMessage.class).withSuffix("hand_pose_update");
   public static final ROS2Topic<HandWrenchActionMessage> HAND_WRENCH_UPDATE_TOPIC
         = COMMAND_TOPIC.withType(HandWrenchActionMessage.class).withSuffix("hand_wrench_update");
   public static final ROS2Topic<PelvisHeightActionMessage> PELVIS_HEIGHT_UPDATE_TOPIC
         = COMMAND_TOPIC.withType(PelvisHeightActionMessage.class).withSuffix("pelvis_height_update");
   public static final ROS2Topic<WaitDurationActionMessage> WAIT_DURATION_UPDATE_TOPIC
         = COMMAND_TOPIC.withType(WaitDurationActionMessage.class).withSuffix("wait_duration_update");
   public static final ROS2Topic<WalkActionMessage> WALK_UPDATE_TOPIC
         = COMMAND_TOPIC.withType(WalkActionMessage.class).withSuffix("walk_update");
   public static final ROS2Topic<ActionSequenceUpdateMessage> SEQUENCE_STATUS_TOPIC
         = STATUS_TOPIC.withType(ActionSequenceUpdateMessage.class).withSuffix("sequence_status");
   public static final ROS2Topic<Empty> MANUALLY_EXECUTE_NEXT_ACTION_TOPIC = COMMAND_TOPIC.withType(Empty.class).withSuffix("manually_execute_next_action");
   public static final ROS2Topic<Bool> AUTOMATIC_EXECUTION_COMMAND_TOPIC = COMMAND_TOPIC.withType(Bool.class).withSuffix("automatic_execution");
   public static final ROS2Topic<Bool> AUTOMATIC_EXECUTION_STATUS_TOPIC = STATUS_TOPIC.withType(Bool.class).withSuffix("automatic_execution");
   public static final ROS2Topic<Int32> EXECUTION_NEXT_INDEX_COMMAND_TOPIC = COMMAND_TOPIC.withType(Int32.class).withSuffix("execution_next_index");
   public static final ROS2Topic<Int32> EXECUTION_NEXT_INDEX_STATUS_TOPIC = STATUS_TOPIC.withType(Int32.class).withSuffix("execution_next_index");

   private final DRCRobotModel robotModel;
   private final ROS2ControllerHelper ros2;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final ROS2SyncedRobotModel syncedRobot;
   private final FootstepPlanningModule footstepPlanner;

   private final LinkedList<BehaviorAction> actionSequence = new LinkedList<>();
   private final IHMCROS2Input<Empty> manuallyExecuteSubscription;
   private final IHMCROS2Input<Bool> automaticExecutionSubscription;
   private final IHMCROS2Input<Int32> executionNextIndexSubscription;
   private boolean automaticExecution = false;
   private int excecutionNextIndex = 0;
   private BehaviorAction currentlyExecutingAction = null;

   private final IHMCROS2Input<ActionSequenceUpdateMessage> updateSubscription;
   private final HashMap<Long, Integer> receivedMessagesForID = new HashMap<>();
   private final BehaviorActionReceiver<ArmJointAnglesActionMessage> armJointAnglesMessageReceiver = new BehaviorActionReceiver<>();
   private final BehaviorActionReceiver<ChestOrientationActionMessage> chestOrientationMessageReceiver = new BehaviorActionReceiver<>();
   private final BehaviorActionReceiver<FootstepActionMessage> footstepMessageReceiver = new BehaviorActionReceiver<>();
   private final BehaviorActionReceiver<HandConfigurationActionMessage> handConfigurationMessageReceiver = new BehaviorActionReceiver<>();
   private final BehaviorActionReceiver<HandPoseActionMessage> handPoseMessageReceiver = new BehaviorActionReceiver<>();
   private final BehaviorActionReceiver<HandWrenchActionMessage> handWrenchMessageReceiver = new BehaviorActionReceiver<>();
   private final BehaviorActionReceiver<PelvisHeightActionMessage> pelvisHeightMessageReceiver = new BehaviorActionReceiver<>();
   private final BehaviorActionReceiver<WaitDurationActionMessage> waitDurationMessageReceiver = new BehaviorActionReceiver<>();
   private final BehaviorActionReceiver<WalkActionMessage> walkMessageReceiver = new BehaviorActionReceiver<>();
   public final Int32 executionNextIndexStatusMessage = new Int32();
   public final Bool automaticExecutionStatusMessage = new Bool();

   public BehaviorActionSequence(DRCRobotModel robotModel, ROS2ControllerHelper ros2, ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.robotModel = robotModel;
      this.ros2 = ros2;
      this.referenceFrameLibrary = referenceFrameLibrary;

      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2.getROS2NodeInterface());
      footstepPlanner = FootstepPlanningModuleLauncher.createModule(robotModel);

      addCommonFrames(referenceFrameLibrary, syncedRobot);
      referenceFrameLibrary.build();

      updateSubscription = ros2.subscribe(SEQUENCE_COMMAND_TOPIC);
      ros2.subscribeViaCallback(ARM_JOINT_ANGLES_UPDATE_TOPIC,
                                message -> armJointAnglesMessageReceiver.receive(message, message.getActionInformation(), receivedMessagesForID));
      ros2.subscribeViaCallback(CHEST_ORIENTATION_UPDATE_TOPIC,
                                message -> chestOrientationMessageReceiver.receive(message, message.getActionInformation(), receivedMessagesForID));
      ros2.subscribeViaCallback(FOOTSTEP_UPDATE_TOPIC,
                                message -> footstepMessageReceiver.receive(message, message.getActionInformation(), receivedMessagesForID));
      ros2.subscribeViaCallback(HAND_CONFIGURATION_UPDATE_TOPIC,
                                message -> handConfigurationMessageReceiver.receive(message, message.getActionInformation(), receivedMessagesForID));
      ros2.subscribeViaCallback(HAND_POSE_UPDATE_TOPIC,
                                message -> handPoseMessageReceiver.receive(message, message.getActionInformation(), receivedMessagesForID));
      ros2.subscribeViaCallback(HAND_WRENCH_UPDATE_TOPIC,
                                message -> handWrenchMessageReceiver.receive(message, message.getActionInformation(), receivedMessagesForID));
      ros2.subscribeViaCallback(PELVIS_HEIGHT_UPDATE_TOPIC,
                                message -> pelvisHeightMessageReceiver.receive(message, message.getActionInformation(), receivedMessagesForID));
      ros2.subscribeViaCallback(WAIT_DURATION_UPDATE_TOPIC,
                                message -> waitDurationMessageReceiver.receive(message, message.getActionInformation(), receivedMessagesForID));
      ros2.subscribeViaCallback(WALK_UPDATE_TOPIC,
                                message -> walkMessageReceiver.receive(message, message.getActionInformation(), receivedMessagesForID));
      manuallyExecuteSubscription = ros2.subscribe(MANUALLY_EXECUTE_NEXT_ACTION_TOPIC);
      automaticExecutionSubscription = ros2.subscribe(AUTOMATIC_EXECUTION_COMMAND_TOPIC);
      executionNextIndexSubscription = ros2.subscribe(EXECUTION_NEXT_INDEX_COMMAND_TOPIC);
   }

   public static void addCommonFrames(ReferenceFrameLibrary referenceFrameLibrary, ROS2SyncedRobotModel syncedRobot)
   {
      referenceFrameLibrary.add(ReferenceFrame.getWorldFrame());
      referenceFrameLibrary.add(syncedRobot.getReferenceFrames().getChestFrame());
      referenceFrameLibrary.add(syncedRobot.getReferenceFrames().getMidFeetUnderPelvisFrame());
      referenceFrameLibrary.add(syncedRobot.getReferenceFrames().getPelvisFrame());
      referenceFrameLibrary.add(syncedRobot.getReferenceFrames().getPelvisZUpFrame());
      referenceFrameLibrary.add(syncedRobot.getReferenceFrames().getMidFeetZUpFrame());
      referenceFrameLibrary.add(syncedRobot.getReferenceFrames().getMidFootZUpGroundFrame());
   }

   public void update()
   {
      // Check latest received update; if we've got enough action updates for that UUID
      ActionSequenceUpdateMessage latestUpdateInformation = updateSubscription.getLatest();
      long sequenceUpdateUUID = latestUpdateInformation.getSequenceUpdateUuid();
      Integer numberOfActionsReceived = receivedMessagesForID.get(sequenceUpdateUUID);
      if (numberOfActionsReceived != null && numberOfActionsReceived == latestUpdateInformation.getSequenceSize()) // Do the update
      {
         automaticExecution = false;

         BehaviorAction[] actionArray = new BehaviorAction[numberOfActionsReceived];

         for (ArmJointAnglesActionMessage message : armJointAnglesMessageReceiver.removeActionList(sequenceUpdateUUID))
         {
            ArmJointAnglesAction action = new ArmJointAnglesAction(ros2);
            action.fromMessage(message);
            actionArray[(int) message.getActionInformation().getActionIndex()] = action;
         }
         for (ChestOrientationActionMessage message : chestOrientationMessageReceiver.removeActionList(sequenceUpdateUUID))
         {
            ChestOrientationAction action = new ChestOrientationAction(ros2, syncedRobot);
            action.fromMessage(message);
            actionArray[(int) message.getActionInformation().getActionIndex()] = action;
         }
         for (FootstepActionMessage message : footstepMessageReceiver.removeActionList(sequenceUpdateUUID))
         {
            FootstepAction action = new FootstepAction(ros2, syncedRobot, referenceFrameLibrary);
            action.fromMessage(message);
            actionArray[(int) message.getActionInformation().getActionIndex()] = action;
         }
         for (HandConfigurationActionMessage message : handConfigurationMessageReceiver.removeActionList(sequenceUpdateUUID))
         {
            HandConfigurationAction action = new HandConfigurationAction(ros2);
            action.fromMessage(message);
            actionArray[(int) message.getActionInformation().getActionIndex()] = action;
         }
         for (HandPoseActionMessage message : handPoseMessageReceiver.removeActionList(sequenceUpdateUUID))
         {
            HandPoseAction action = new HandPoseAction(ros2, referenceFrameLibrary);
            action.fromMessage(message);
            actionArray[(int) message.getActionInformation().getActionIndex()] = action;
         }
         for (HandWrenchActionMessage message : handWrenchMessageReceiver.removeActionList(sequenceUpdateUUID))
         {
            HandWrenchAction action = new HandWrenchAction(ros2);
            action.fromMessage(message);
            actionArray[(int) message.getActionInformation().getActionIndex()] = action;
         }
         for (PelvisHeightActionMessage message : pelvisHeightMessageReceiver.removeActionList(sequenceUpdateUUID))
         {
            PelvisHeightAction action = new PelvisHeightAction(ros2);
            action.fromMessage(message);
            actionArray[(int) message.getActionInformation().getActionIndex()] = action;
         }
         for (WaitDurationActionMessage message : waitDurationMessageReceiver.removeActionList(sequenceUpdateUUID))
         {
            WaitDurationAction action = new WaitDurationAction();
            action.fromMessage(message);
            actionArray[(int) message.getActionInformation().getActionIndex()] = action;
         }
         for (WalkActionMessage message : walkMessageReceiver.removeActionList(sequenceUpdateUUID))
         {
            WalkAction action = new WalkAction(ros2, syncedRobot, robotModel, footstepPlanner, referenceFrameLibrary);
            action.fromMessage(message);
            actionArray[(int) message.getActionInformation().getActionIndex()] = action;
         }

         // Remove from the map when we're done so it doesn't grow infinitely
         receivedMessagesForID.remove(sequenceUpdateUUID);

         actionSequence.clear();
         for (BehaviorAction action : actionArray)
            actionSequence.add(action);

         if (excecutionNextIndex > numberOfActionsReceived)
            excecutionNextIndex = numberOfActionsReceived;

         LogTools.info("Updated action sequence recieved with {} actions. Execution next index: {}", actionSequence.size(), excecutionNextIndex);
      }

      syncedRobot.update();

      for (var action : actionSequence)
         action.update();

      if (automaticExecutionSubscription.getMessageNotification().poll())
         automaticExecution = automaticExecutionSubscription.getMessageNotification().read().getData();
      if (executionNextIndexSubscription.getMessageNotification().poll())
         excecutionNextIndex = executionNextIndexSubscription.getMessageNotification().read().getData();

      sendStatus();

      if (automaticExecution)
      {
         boolean endOfSequence = excecutionNextIndex >= actionSequence.size();
         if (endOfSequence)
         {
            automaticExecution = false;
            currentlyExecutingAction = null;
         }
         else if (currentlyExecutingAction == null || !currentlyExecutingAction.isExecuting())
         {
            LogTools.info("Automatically executing action: {}", actionSequence.get(excecutionNextIndex).getClass().getSimpleName());
            executeNextAction();
         }
      }
      else if (manuallyExecuteSubscription.getMessageNotification().poll())
      {
         LogTools.info("Manually executing action: {}", actionSequence.get(excecutionNextIndex).getClass().getSimpleName());
         executeNextAction();
      }
   }

   private void executeNextAction()
   {
      currentlyExecutingAction = actionSequence.get(excecutionNextIndex);
      currentlyExecutingAction.executeAction();
      excecutionNextIndex++;
   }

   private void sendStatus()
   {
      executionNextIndexStatusMessage.setData(excecutionNextIndex);
      ros2.publish(EXECUTION_NEXT_INDEX_STATUS_TOPIC, executionNextIndexStatusMessage);
      automaticExecutionStatusMessage.setData(automaticExecution);
      ros2.publish(AUTOMATIC_EXECUTION_STATUS_TOPIC, automaticExecutionStatusMessage);

      // TODO
//      long updateUUID = UUID.randomUUID().getLeastSignificantBits();
//      ActionSequenceUpdateMessage actionSequenceUpdateMessage = new ActionSequenceUpdateMessage();
//      actionSequenceUpdateMessage.setSequenceUpdateUuid(updateUUID);
//      actionSequenceUpdateMessage.setSequenceSize(actionSequence.size());
//      ros2.publish(BehaviorActionSequence.SEQUENCE_COMMAND_TOPIC, actionSequenceUpdateMessage);
//
//      for (int i = 0; i < actionSequence.size(); i++)
//      {
//         BehaviorAction action = actionSequence.get(i);
//         if (action instanceof ArmJointAnglesAction armJointAnglesAction)
//         {
//            ArmJointAnglesActionMessage armJointAnglesActionMessage = new ArmJointAnglesActionMessage();
//            armJointAnglesActionMessage.getActionInformation().setActionIndex(i);
//            armJointAnglesActionMessage.getActionInformation().setSequenceUpdateUuid(updateUUID);
//            armJointAnglesAction.toMessage(armJointAnglesActionMessage);
//            ros2.publish(BehaviorActionSequence.ARM_JOINT_ANGLES_UPDATE_TOPIC, armJointAnglesActionMessage);
//         }
//         else if (action instanceof ChestOrientationAction chestOrientationAction)
//         {
//            ChestOrientationActionMessage chestOrientationActionMessage = new ChestOrientationActionMessage();
//            chestOrientationActionMessage.getActionInformation().setActionIndex(i);
//            chestOrientationActionMessage.getActionInformation().setSequenceUpdateUuid(updateUUID);
//            chestOrientationAction.toMessage(chestOrientationActionMessage);
//            ros2.publish(BehaviorActionSequence.CHEST_ORIENTATION_UPDATE_TOPIC, chestOrientationActionMessage);
//         }
//         else if (action instanceof FootstepAction footstepAction)
//         {
//            FootstepActionMessage footstepActionMessage = new FootstepActionMessage();
//            footstepActionMessage.getActionInformation().setActionIndex(i);
//            footstepActionMessage.getActionInformation().setSequenceUpdateUuid(updateUUID);
//            footstepAction.toMessage(footstepActionMessage);
//            ros2.publish(BehaviorActionSequence.FOOTSTEP_UPDATE_TOPIC, footstepActionMessage);
//         }
//         else if (action instanceof HandConfigurationAction handConfigurationAction)
//         {
//            HandConfigurationActionMessage handConfigurationActionMessage = new HandConfigurationActionMessage();
//            handConfigurationActionMessage.getActionInformation().setActionIndex(i);
//            handConfigurationActionMessage.getActionInformation().setSequenceUpdateUuid(updateUUID);
//            handConfigurationAction.toMessage(handConfigurationActionMessage);
//            ros2.publish(BehaviorActionSequence.HAND_CONFIGURATION_UPDATE_TOPIC, handConfigurationActionMessage);
//         }
//         else if (action instanceof HandPoseAction handPoseAction)
//         {
//            HandPoseActionMessage handPoseActionMessage = new HandPoseActionMessage();
//            handPoseActionMessage.getActionInformation().setActionIndex(i);
//            handPoseActionMessage.getActionInformation().setSequenceUpdateUuid(updateUUID);
//            handPoseAction.toMessage(handPoseActionMessage);
//            ros2.publish(BehaviorActionSequence.HAND_POSE_UPDATE_TOPIC, handPoseActionMessage);
//         }
//         else if (action instanceof HandWrenchAction handWrenchAction)
//         {
//            HandWrenchActionMessage handWrenchActionMessage = new HandWrenchActionMessage();
//            handWrenchActionMessage.getActionInformation().setActionIndex(i);
//            handWrenchActionMessage.getActionInformation().setSequenceUpdateUuid(updateUUID);
//            handWrenchAction.toMessage(handWrenchActionMessage);
//            ros2.publish(BehaviorActionSequence.HAND_WRENCH_UPDATE_TOPIC, handWrenchActionMessage);
//         }
//         else if (action instanceof PelvisHeightAction pelvisHeightAction)
//         {
//            PelvisHeightActionMessage pelvisHeightActionMessage = new PelvisHeightActionMessage();
//            pelvisHeightActionMessage.getActionInformation().setActionIndex(i);
//            pelvisHeightActionMessage.getActionInformation().setSequenceUpdateUuid(updateUUID);
//            pelvisHeightAction.toMessage(pelvisHeightActionMessage);
//            ros2.publish(BehaviorActionSequence.PELVIS_HEIGHT_UPDATE_TOPIC, pelvisHeightActionMessage);
//         }
//         else if (action instanceof WaitDurationAction waitDurationAction)
//         {
//            WaitDurationActionMessage waitDurationActionMessage = new WaitDurationActionMessage();
//            waitDurationActionMessage.getActionInformation().setActionIndex(i);
//            waitDurationActionMessage.getActionInformation().setSequenceUpdateUuid(updateUUID);
//            waitDurationAction.toMessage(waitDurationActionMessage);
//            ros2.publish(BehaviorActionSequence.WAIT_DURATION_UPDATE_TOPIC, waitDurationActionMessage);
//         }
//         else if (action instanceof WalkAction walkAction)
//         {
//            WalkActionMessage walkActionMessage = new WalkActionMessage();
//            walkActionMessage.getActionInformation().setActionIndex(i);
//            walkActionMessage.getActionInformation().setSequenceUpdateUuid(updateUUID);
//            walkAction.toMessage(walkActionMessage);
//            ros2.publish(BehaviorActionSequence.WALK_UPDATE_TOPIC, walkActionMessage);
//         }
//      }
   }
}
