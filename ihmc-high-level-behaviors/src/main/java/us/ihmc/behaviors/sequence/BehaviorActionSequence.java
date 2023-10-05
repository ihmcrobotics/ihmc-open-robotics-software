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
import us.ihmc.behaviors.tools.HandWrenchCalculator;
import us.ihmc.behaviors.tools.walkingController.WalkingFootstepTracker;
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

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

/**
 * Manages running a sequence of actions on the robot with shared autonomy.
 * Since this class always currently gets its instructions from the operator, it never loads
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
   public static final ROS2Topic<std_msgs.msg.dds.String> EXECUTION_NEXT_INDEX_REJECTION_TOPIC = STATUS_TOPIC.withType(std_msgs.msg.dds.String.class).withSuffix("execution_next_index_rejection");
   public static final ROS2Topic<HandPoseJointAnglesStatusMessage> LEFT_HAND_POSE_JOINT_ANGLES_STATUS
         = STATUS_TOPIC.withType(HandPoseJointAnglesStatusMessage.class).withSuffix("left_hand_pose_joint_angles");
   public static final ROS2Topic<HandPoseJointAnglesStatusMessage> RIGHT_HAND_POSE_JOINT_ANGLES_STATUS
         = STATUS_TOPIC.withType(HandPoseJointAnglesStatusMessage.class).withSuffix("right_hand_pose_joint_angles");
   public static final ROS2Topic<BodyPartPoseStatusMessage> CHEST_POSE_STATUS
         = STATUS_TOPIC.withType(BodyPartPoseStatusMessage.class).withSuffix("chest_pose_status");
   public static final ROS2Topic<BodyPartPoseStatusMessage> PELVIS_POSE_VARIATION_STATUS
         = STATUS_TOPIC.withType(BodyPartPoseStatusMessage.class).withSuffix("pelvis_pose_status");
   public static final ROS2Topic<ActionsExecutionStatusMessage> ACTIONS_EXECUTION_STATUS
         = STATUS_TOPIC.withType(ActionsExecutionStatusMessage.class).withSuffix("execution_status");

   private final DRCRobotModel robotModel;
   private final ROS2ControllerHelper ros2;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final ROS2SyncedRobotModel syncedRobot;
   private final WalkingFootstepTracker footstepTracker;
   private final HandWrenchCalculator handWrenchCalculator;
   private final FootstepPlanningModule footstepPlanner;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final WalkingControllerParameters walkingControllerParameters;

   private final LinkedList<BehaviorActionExecutor> actionSequence = new LinkedList<>();
   private final IHMCROS2Input<Empty> manuallyExecuteSubscription;
   private final IHMCROS2Input<Bool> automaticExecutionSubscription;
   private final IHMCROS2Input<Int32> executionNextIndexSubscription;
   private boolean automaticExecution = false;
   private int executionNextIndex = 0;
   private final List<BehaviorActionExecutor> currentlyExecutingActions = new ArrayList<>();
   private BehaviorActionExecutor lastCurrentlyExecutingAction = null;

   private final IHMCROS2Input<ActionSequenceUpdateMessage> updateSubscription;
   public final Int32 executionNextIndexStatusMessage = new Int32();
   public final Bool automaticExecutionStatusMessage = new Bool();

   private final Throttler oneHertzThrottler = new Throttler();
   private final ActionSequenceUpdateMessage actionSequenceStatusMessage = new ActionSequenceUpdateMessage();
   private ActionExecutionStatusMessage nothingExecutingStatusMessage = new ActionExecutionStatusMessage();
   private final ActionsExecutionStatusMessage actionsExecutionStatusMessage = new ActionsExecutionStatusMessage();

   public BehaviorActionSequence(DRCRobotModel robotModel, ROS2ControllerHelper ros2, ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.robotModel = robotModel;
      this.ros2 = ros2;
      this.referenceFrameLibrary = referenceFrameLibrary;

      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2.getROS2NodeInterface());
      footstepTracker = new WalkingFootstepTracker(ros2.getROS2NodeInterface(), robotModel.getSimpleRobotName());
      handWrenchCalculator = new HandWrenchCalculator(syncedRobot);
      footstepPlanner = FootstepPlanningModuleLauncher.createModule(robotModel);
      footstepPlannerParameters = robotModel.getFootstepPlannerParameters();
      walkingControllerParameters = robotModel.getWalkingControllerParameters();

      addCommonFrames(referenceFrameLibrary, syncedRobot);

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

         BehaviorActionExecutor[] actionArray = new BehaviorActionExecutor[latestUpdateMessage.getSequenceSize()];

         for (ArmJointAnglesActionDefinitionMessage message : latestUpdateMessage.getArmJointAnglesActions())
         {
            ArmJointAnglesActionExecutor action = new ArmJointAnglesActionExecutor(robotModel, ros2);
            action.fromMessage(message);
            actionArray[(int) message.getActionInformation().getActionIndex()] = action;
         }
         for (BodyPartPoseActionDefinitionMessage message : latestUpdateMessage.getChestOrientationActions())
         {
            ChestOrientationActionExecutor action = new ChestOrientationActionExecutor(ros2, syncedRobot, referenceFrameLibrary);
            action.fromMessage(message);
            actionArray[(int) message.getActionInformation().getActionIndex()] = action;
         }
         for (FootstepPlanActionDefinitionMessage message : latestUpdateMessage.getFootstepPlanActions())
         {
            FootstepPlanActionExecutor action = new FootstepPlanActionExecutor(ros2, syncedRobot, footstepTracker, referenceFrameLibrary, walkingControllerParameters);
            action.fromMessage(message);
            actionArray[(int) message.getActionInformation().getActionIndex()] = action;
         }
         for (SakeHandCommandActionDefinitionMessage message : latestUpdateMessage.getSakeHandCommandActions())
         {
            SakeHandCommandActionExecutor action = new SakeHandCommandActionExecutor(ros2);
            action.fromMessage(message);
            actionArray[(int) message.getActionInformation().getActionIndex()] = action;
         }
         for (SidedBodyPartPoseActionDefinitionMessage message : latestUpdateMessage.getHandPoseActions())
         {
            HandPoseActionExecutor action = new HandPoseActionExecutor(ros2, referenceFrameLibrary, robotModel, syncedRobot, handWrenchCalculator);
            action.fromMessage(message);
            actionArray[(int) message.getActionInformation().getActionIndex()] = action;
         }
         for (HandWrenchActionDefinitionMessage message : latestUpdateMessage.getHandWrenchActions())
         {
            HandWrenchActionExecutor action = new HandWrenchActionExecutor(ros2);
            action.fromMessage(message);
            actionArray[(int) message.getActionInformation().getActionIndex()] = action;
         }
         for (BodyPartPoseActionDefinitionMessage message : latestUpdateMessage.getPelvisHeightActions())
         {
            PelvisHeightPitchActionExecutor action = new PelvisHeightPitchActionExecutor(ros2, referenceFrameLibrary, syncedRobot);
            action.fromMessage(message);
            actionArray[(int) message.getActionInformation().getActionIndex()] = action;
         }
         for (SidedBodyPartPoseActionDefinitionMessage message : latestUpdateMessage.getFootPoseActions())
         {
            FootPoseActionExecutor action = new FootPoseActionExecutor(ros2, syncedRobot, referenceFrameLibrary);
            action.fromMessage(message);
            actionArray[(int) message.getActionInformation().getActionIndex()] = action;
         }
         for (WaitDurationActionDefinitionMessage message : latestUpdateMessage.getWaitDurationActions())
         {
            WaitDurationActionExecutor action = new WaitDurationActionExecutor(ros2);
            action.fromMessage(message);
            actionArray[(int) message.getActionInformation().getActionIndex()] = action;
         }
         for (WalkActionDefinitionMessage message : latestUpdateMessage.getWalkActions())
         {
            WalkActionExecutor action = new WalkActionExecutor(ros2,
                                                               syncedRobot,
                                                               footstepTracker,
                                                               footstepPlanner,
                                                               footstepPlannerParameters,
                                                               walkingControllerParameters,
                                                               referenceFrameLibrary);
            action.fromMessage(message);
            actionArray[(int) message.getActionInformation().getActionIndex()] = action;
         }

         actionSequence.clear();
         for (BehaviorActionExecutor action : actionArray)
         {
            actionSequence.add(action);
         }

         if (executionNextIndex > latestUpdateMessage.getSequenceSize())
            executionNextIndex = latestUpdateMessage.getSequenceSize();

         LogTools.info("Updated action sequence received with {} actions. Execution next index: {}", actionSequence.size(), executionNextIndex);
      }

      syncedRobot.update();
      handWrenchCalculator.compute();

      if (automaticExecutionSubscription.getMessageNotification().poll())
      {
         automaticExecution = automaticExecutionSubscription.getMessageNotification().read().getData();
      }
      if (executionNextIndexSubscription.getMessageNotification().poll())
      {
         executionNextIndex = executionNextIndexSubscription.getMessageNotification().read().getData();
         lastCurrentlyExecutingAction = null; // Set to null so we don't wait for that action to complete
         currentlyExecutingActions.clear();
      }

      for (int actionIndex = 0; actionIndex < actionSequence.size(); actionIndex++)
      {
         boolean executeWithPreviousAction = false;
         if (actionIndex > 0)
            executeWithPreviousAction = actionSequence.get(actionIndex - 1).getExecuteWithNextAction();

         boolean firstConcurrentActionIsNextForExecution = actionSequence.get(actionIndex).getExecuteWithNextAction() && actionIndex == executionNextIndex;
         boolean otherConcurrentActionIsNextForExecution = executeWithPreviousAction && actionIndex == (executionNextIndex + getIndexShiftFromConcurrentActionRoot(actionIndex, executionNextIndex,true));
         boolean concurrentActionIsNextForExecution = firstConcurrentActionIsNextForExecution || otherConcurrentActionIsNextForExecution;
         actionSequence.get(actionIndex).update(actionIndex, executionNextIndex, concurrentActionIsNextForExecution);
      }

      actionsExecutionStatusMessage.getActionStatusList().clear();
      if (lastCurrentlyExecutingAction != null)
      {
         for (BehaviorActionExecutor currentlyExecutingAction : currentlyExecutingActions)
         {
            currentlyExecutingAction.updateCurrentlyExecuting();
            ActionExecutionStatusMessage currentlyExecutingActionMessage = actionsExecutionStatusMessage.getActionStatusList().add();
            currentlyExecutingActionMessage.set(currentlyExecutingAction.getExecutionStatusMessage());
         }
         ros2.publish(ACTIONS_EXECUTION_STATUS, actionsExecutionStatusMessage);
      }
      else
      {
         nothingExecutingStatusMessage = actionsExecutionStatusMessage.getActionStatusList().add();
         nothingExecutingStatusMessage.setActionIndex(-1);
         ros2.publish(ACTIONS_EXECUTION_STATUS, actionsExecutionStatusMessage);
      }

      sendStatus();

      if (automaticExecution)
      {
         boolean endOfSequence = executionNextIndex >= actionSequence.size();
         if (endOfSequence)
         {
            automaticExecution = false;
         }
         else if (lastCurrentlyExecutingAction == null && noCurrentActionIsExecuting())
         {
            do
            {
               LogTools.info("Automatically executing action: {}", actionSequence.get(executionNextIndex).getClass().getSimpleName());
               executeNextAction();
            }
            while (lastCurrentlyExecutingAction != null && lastCurrentlyExecutingAction.getExecuteWithNextAction());
         }
      }
      else if (manuallyExecuteSubscription.getMessageNotification().poll())
      {
         do
         {
            LogTools.info("Manually executing action: {}", actionSequence.get(executionNextIndex).getClass().getSimpleName());
            executeNextAction();
         }
         while (lastCurrentlyExecutingAction != null && lastCurrentlyExecutingAction.getExecuteWithNextAction());
      }

      if (lastCurrentlyExecutingAction != null && noCurrentActionIsExecuting())
      {
         lastCurrentlyExecutingAction = null;
         currentlyExecutingActions.clear();
      }
   }

   /**
    * @param actionIndex Index of the current action
    * @param executionNextIndex Index of the next action to be executed
    * @param executeWithPreviousAction Whether this action has to be executed at the same time of the previous one
    * @return Index shift in the actionSequence array from the current action to the first action of the same group of concurrent actions
    */
   private int getIndexShiftFromConcurrentActionRoot(int actionIndex, int executionNextIndex, boolean executeWithPreviousAction)
   {
      if (executeWithPreviousAction)
      {
         boolean isNotRootOfConcurrency = true;
         for (int j = 1; j <= actionIndex; j++)
         {
            boolean thisPreviousActionIsConcurrent = actionSequence.get(actionIndex - j).getExecuteWithNextAction();
            isNotRootOfConcurrency = thisPreviousActionIsConcurrent && executionNextIndex != (actionIndex - j + 1);
            if (!isNotRootOfConcurrency)
            {
               return (j - 1);
            }
            else if ((actionIndex - j) == 0 && thisPreviousActionIsConcurrent)
            {
               return j;
            }
         }
      }
      return -1;
   }

   private void executeNextAction()
   {
      boolean executeWithPreviousAction = false;
      if (lastCurrentlyExecutingAction != null)
         executeWithPreviousAction = lastCurrentlyExecutingAction.getExecuteWithNextAction();
      lastCurrentlyExecutingAction = actionSequence.get(executionNextIndex);
      // If automatic execution, we want to ensure it's able to execute before we perform the execution.
      // If it's unable to execute, disable automatic execution.
      if (automaticExecution)
      {
         if (!lastCurrentlyExecutingAction.canExecute())
         {
            automaticExecution = false;
            // Early return
            return;
         }
      }
      boolean concurrentActionIsNextForExecution = lastCurrentlyExecutingAction.getExecuteWithNextAction() || executeWithPreviousAction;
      lastCurrentlyExecutingAction.update(executionNextIndex, executionNextIndex + 1, concurrentActionIsNextForExecution);
      lastCurrentlyExecutingAction.triggerActionExecution();
      lastCurrentlyExecutingAction.updateCurrentlyExecuting();
      currentlyExecutingActions.add(lastCurrentlyExecutingAction);
      executionNextIndex++;
   }

   private boolean noCurrentActionIsExecuting()
   {
      boolean noCurrentActionIsExecuting = true;
      for (BehaviorActionExecutor currentlyExecutingAction : currentlyExecutingActions)
      {
         noCurrentActionIsExecuting &= !currentlyExecutingAction.isExecuting();
      }
      return noCurrentActionIsExecuting;
   }

   private void sendStatus()
   {
      executionNextIndexStatusMessage.setData(executionNextIndex);
      ros2.publish(EXECUTION_NEXT_INDEX_STATUS_TOPIC, executionNextIndexStatusMessage);
      automaticExecutionStatusMessage.setData(automaticExecution);
      ros2.publish(AUTOMATIC_EXECUTION_STATUS_TOPIC, automaticExecutionStatusMessage);

      if (oneHertzThrottler.run(1.0))
      {
         BehaviorActionSequenceTools.packActionSequenceUpdateMessage(actionSequence, actionSequenceStatusMessage);
         ros2.publish(BehaviorActionSequence.SEQUENCE_STATUS_TOPIC, actionSequenceStatusMessage);
      }
   }

   public void destroy()
   {
      syncedRobot.destroy();
      footstepPlanner.closeAndDispose();
      footstepTracker.destroy();
      updateSubscription.destroy();
      manuallyExecuteSubscription.destroy();
      automaticExecutionSubscription.destroy();
      executionNextIndexSubscription.destroy();
   }
}
