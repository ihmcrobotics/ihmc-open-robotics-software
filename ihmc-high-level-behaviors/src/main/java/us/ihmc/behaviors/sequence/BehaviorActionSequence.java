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
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.Throttler;

import java.util.LinkedList;

/**
 * Manages running a sequence of actions on the robot with shared autonomy.
 * Since this class always currently gets it's instructions from the operator, it never loads
 * a sequence from file, and the sequence is always completely updated by the operator, even
 * when switching sequences entirely.
 */
public class BehaviorActionSequence
{
   public static final ROS2Topic<?> ROOT_TOPIC = ROS2Tools.IHMC_ROOT.withRobot("behavior_action_sequence");
   public static final ROS2Topic<?> COMMAND_TOPIC = ROOT_TOPIC.withInput();
   public static final ROS2Topic<?> STATUS_TOPIC = ROOT_TOPIC.withOutput();
   public static final ROS2Topic<ActionSequenceUpdateMessage> SEQUENCE_COMMAND_TOPIC
         = COMMAND_TOPIC.withType(ActionSequenceUpdateMessage.class).withSuffix("sequence_update");
   public static final ROS2Topic<ActionSequenceUpdateMessage> SEQUENCE_STATUS_TOPIC
         = STATUS_TOPIC.withType(ActionSequenceUpdateMessage.class).withSuffix("sequence_status");
   public static final ROS2Topic<Empty> MANUALLY_EXECUTE_NEXT_ACTION_TOPIC = COMMAND_TOPIC.withType(Empty.class).withSuffix("manually_execute_next_action");
   public static final ROS2Topic<Bool> AUTOMATIC_EXECUTION_COMMAND_TOPIC = COMMAND_TOPIC.withType(Bool.class).withSuffix("automatic_execution");
   public static final ROS2Topic<Bool> AUTOMATIC_EXECUTION_STATUS_TOPIC = STATUS_TOPIC.withType(Bool.class).withSuffix("automatic_execution");
   public static final ROS2Topic<Int32> EXECUTION_NEXT_INDEX_COMMAND_TOPIC = COMMAND_TOPIC.withType(Int32.class).withSuffix("execution_next_index");
   public static final ROS2Topic<Int32> EXECUTION_NEXT_INDEX_STATUS_TOPIC = STATUS_TOPIC.withType(Int32.class).withSuffix("execution_next_index");
   public static final ROS2Topic<HandPoseJointAnglesStatusMessage> HAND_POSE_JOINT_ANGLES_STATUS
         = STATUS_TOPIC.withType(HandPoseJointAnglesStatusMessage.class).withSuffix("hand_pose_joint_angles");

   private final DRCRobotModel robotModel;
   private final ROS2ControllerHelper ros2;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final ROS2SyncedRobotModel syncedRobot;
   private final FootstepPlanningModule footstepPlanner;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final WalkingControllerParameters walkingControllerParameters;

   private final LinkedList<BehaviorAction> actionSequence = new LinkedList<>();
   private final IHMCROS2Input<Empty> manuallyExecuteSubscription;
   private final IHMCROS2Input<Bool> automaticExecutionSubscription;
   private final IHMCROS2Input<Int32> executionNextIndexSubscription;
   private boolean automaticExecution = false;
   private int excecutionNextIndex = 0;
   private BehaviorAction currentlyExecutingAction = null;

   private final IHMCROS2Input<ActionSequenceUpdateMessage> updateSubscription;
   public final Int32 executionNextIndexStatusMessage = new Int32();
   public final Bool automaticExecutionStatusMessage = new Bool();

   private final Throttler oneHertzThrottler = new Throttler();
   private final ActionSequenceUpdateMessage actionSequenceStatusMessage = new ActionSequenceUpdateMessage();

   public BehaviorActionSequence(DRCRobotModel robotModel, ROS2ControllerHelper ros2, ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.robotModel = robotModel;
      this.ros2 = ros2;
      this.referenceFrameLibrary = referenceFrameLibrary;

      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2.getROS2NodeInterface());
      footstepPlanner = FootstepPlanningModuleLauncher.createModule(robotModel);
      footstepPlannerParameters = robotModel.getFootstepPlannerParameters();
      walkingControllerParameters = robotModel.getWalkingControllerParameters();

      addCommonFrames(referenceFrameLibrary, syncedRobot);
      referenceFrameLibrary.build();

      updateSubscription = ros2.subscribe(SEQUENCE_COMMAND_TOPIC);
      manuallyExecuteSubscription = ros2.subscribe(MANUALLY_EXECUTE_NEXT_ACTION_TOPIC);
      automaticExecutionSubscription = ros2.subscribe(AUTOMATIC_EXECUTION_COMMAND_TOPIC);
      executionNextIndexSubscription = ros2.subscribe(EXECUTION_NEXT_INDEX_COMMAND_TOPIC);
   }

   public static void addCommonFrames(ReferenceFrameLibrary referenceFrameLibrary, ROS2SyncedRobotModel syncedRobot)
   {
      referenceFrameLibrary.add(ReferenceFrame::getWorldFrame);
      referenceFrameLibrary.add(syncedRobot.getReferenceFrames()::getChestFrame);
      referenceFrameLibrary.add(syncedRobot.getReferenceFrames()::getMidFeetUnderPelvisFrame);
      referenceFrameLibrary.add(syncedRobot.getReferenceFrames()::getPelvisFrame);
      referenceFrameLibrary.add(syncedRobot.getReferenceFrames()::getPelvisZUpFrame);
      referenceFrameLibrary.add(syncedRobot.getReferenceFrames()::getMidFeetZUpFrame);
      referenceFrameLibrary.add(syncedRobot.getReferenceFrames()::getMidFootZUpGroundFrame);
   }

   public void update()
   {
      if (updateSubscription.getMessageNotification().poll()) // Do the update
      {
         automaticExecution = false;

         ActionSequenceUpdateMessage latestUpdateMessage = updateSubscription.getMessageNotification().read();

         BehaviorAction[] actionArray = new BehaviorAction[latestUpdateMessage.getSequenceSize()];

         for (ArmJointAnglesActionMessage message : latestUpdateMessage.getArmJointAnglesActions())
         {
            ArmJointAnglesAction action = new ArmJointAnglesAction(ros2);
            action.fromMessage(message);
            actionArray[(int) message.getActionInformation().getActionIndex()] = action;
         }
         for (ChestOrientationActionMessage message : latestUpdateMessage.getChestOrientationActions())
         {
            ChestOrientationAction action = new ChestOrientationAction(ros2, syncedRobot);
            action.fromMessage(message);
            actionArray[(int) message.getActionInformation().getActionIndex()] = action;
         }
         for (FootstepActionMessage message : latestUpdateMessage.getFootstepActions())
         {
            FootstepAction action = new FootstepAction(ros2, referenceFrameLibrary);
            action.fromMessage(message);
            actionArray[(int) message.getActionInformation().getActionIndex()] = action;
         }
         for (HandConfigurationActionMessage message : latestUpdateMessage.getHandConfigurationActions())
         {
            HandConfigurationAction action = new HandConfigurationAction(ros2);
            action.fromMessage(message);
            actionArray[(int) message.getActionInformation().getActionIndex()] = action;
         }
         for (HandPoseActionMessage message : latestUpdateMessage.getHandPoseActions())
         {
            HandPoseAction action = new HandPoseAction(ros2, referenceFrameLibrary, robotModel, syncedRobot);
            action.fromMessage(message);
            actionArray[(int) message.getActionInformation().getActionIndex()] = action;
         }
         for (HandWrenchActionMessage message : latestUpdateMessage.getHandWrenchActions())
         {
            HandWrenchAction action = new HandWrenchAction(ros2);
            action.fromMessage(message);
            actionArray[(int) message.getActionInformation().getActionIndex()] = action;
         }
         for (PelvisHeightActionMessage message : latestUpdateMessage.getPelvisHeightActions())
         {
            PelvisHeightAction action = new PelvisHeightAction(ros2);
            action.fromMessage(message);
            actionArray[(int) message.getActionInformation().getActionIndex()] = action;
         }
         for (WaitDurationActionMessage message : latestUpdateMessage.getWaitDurationActions())
         {
            WaitDurationAction action = new WaitDurationAction();
            action.fromMessage(message);
            actionArray[(int) message.getActionInformation().getActionIndex()] = action;
         }
         for (WalkActionMessage message : latestUpdateMessage.getWalkActions())
         {
            WalkAction action = new WalkAction(ros2,
                                               syncedRobot,
                                               footstepPlanner,
                                               footstepPlannerParameters,
                                               walkingControllerParameters,
                                               referenceFrameLibrary);
            action.fromMessage(message);
            actionArray[(int) message.getActionInformation().getActionIndex()] = action;
         }

         actionSequence.clear();
         for (BehaviorAction action : actionArray)
         {
            actionSequence.add(action);
         }

         if (excecutionNextIndex > latestUpdateMessage.getSequenceSize())
            excecutionNextIndex = latestUpdateMessage.getSequenceSize();

         LogTools.info("Updated action sequence recieved with {} actions. Execution next index: {}", actionSequence.size(), excecutionNextIndex);
      }

      syncedRobot.update();

      if (automaticExecutionSubscription.getMessageNotification().poll())
      {
         automaticExecution = automaticExecutionSubscription.getMessageNotification().read().getData();
      }
      if (executionNextIndexSubscription.getMessageNotification().poll())
      {
         excecutionNextIndex = executionNextIndexSubscription.getMessageNotification().read().getData();
         currentlyExecutingAction = null; // Set to null so we don't wait for that action to complete
      }

      for (int i = 0; i < actionSequence.size(); i++)
      {
         actionSequence.get(i).update(i, excecutionNextIndex);
      }

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

      if (oneHertzThrottler.run(1.0))
      {
         BehaviorActionSequenceTools.packActionSequenceUpdateMessage(actionSequence, actionSequenceStatusMessage);
         ros2.publish(BehaviorActionSequence.SEQUENCE_STATUS_TOPIC, actionSequenceStatusMessage);
      }
   }
}
