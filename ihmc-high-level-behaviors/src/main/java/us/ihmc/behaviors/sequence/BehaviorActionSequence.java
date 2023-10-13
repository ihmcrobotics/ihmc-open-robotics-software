package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.*;
import org.apache.commons.lang3.mutable.MutableLong;
import std_msgs.msg.dds.Bool;
import std_msgs.msg.dds.Empty;
import std_msgs.msg.dds.Int32;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.actions.*;
import us.ihmc.behaviors.tools.ROS2HandWrenchCalculator;
import us.ihmc.behaviors.tools.walkingController.WalkingFootstepTracker;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
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

   /** TODO: Make non-static and assign as actions are created. */
   public static final MutableLong NEXT_ID = new MutableLong();

   private final DRCRobotModel robotModel;
   private final ROS2ControllerHelper ros2;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final ROS2SyncedRobotModel syncedRobot;
   private final WalkingFootstepTracker footstepTracker;
   private final SideDependentList<ROS2HandWrenchCalculator> handWrenchCalculators = new SideDependentList<>();
   private final FootstepPlanningModule footstepPlanner;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final WalkingControllerParameters walkingControllerParameters;

   private final LinkedList<BehaviorActionExecutor> actionSequence = new LinkedList<>();
   private final IHMCROS2Input<Empty> manuallyExecuteSubscription;
   private final IHMCROS2Input<Bool> automaticExecutionSubscription;
   private final IHMCROS2Input<Int32> executionNextIndexSubscription;
   private boolean automaticExecution = false;
   private int executionNextIndex = 0;
   private final BehaviorActionExecutionStatusCalculator executionStatusCalculator = new BehaviorActionExecutionStatusCalculator();
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
      for (RobotSide side : RobotSide.values)
         handWrenchCalculators.put(side, new ROS2HandWrenchCalculator(side, syncedRobot));
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
      if (updateSubscription.getMessageNotification().poll()) // Do the update
      {
         ActionSequenceUpdateMessage latestUpdateMessage = updateSubscription.getMessageNotification().read();

         BehaviorActionExecutor[] actionArray = new BehaviorActionExecutor[latestUpdateMessage.getSequenceSize()];

         for (ArmJointAnglesActionStateMessage message : latestUpdateMessage.getArmJointAnglesActions())
         {
            ArmJointAnglesActionExecutor action = new ArmJointAnglesActionExecutor(this, robotModel, ros2);
            action.getState().fromMessage(message);
            actionArray[message.getActionState().getActionIndex()] = action;
         }
         for (ChestOrientationActionStateMessage message : latestUpdateMessage.getChestOrientationActions())
         {
            ChestOrientationActionExecutor action = new ChestOrientationActionExecutor(this, ros2, syncedRobot, referenceFrameLibrary);
            action.getState().fromMessage(message);
            actionArray[message.getActionState().getActionIndex()] = action;
         }
         for (FootstepPlanActionStateMessage message : latestUpdateMessage.getFootstepPlanActions())
         {
            FootstepPlanActionExecutor action = new FootstepPlanActionExecutor(this,
                                                                               ros2,
                                                                               syncedRobot,
                                                                               footstepTracker,
                                                                               referenceFrameLibrary,
                                                                               walkingControllerParameters);
            action.getState().fromMessage(message);
            actionArray[message.getActionState().getActionIndex()] = action;
         }
         for (SakeHandCommandActionStateMessage message : latestUpdateMessage.getSakeHandCommandActions())
         {
            SakeHandCommandActionExecutor action = new SakeHandCommandActionExecutor(this, ros2);
            action.getState().fromMessage(message);
            actionArray[message.getActionState().getActionIndex()] = action;
         }
         for (HandPoseActionStateMessage message : latestUpdateMessage.getHandPoseActions())
         {
            HandPoseActionExecutor action = new HandPoseActionExecutor(this, ros2, referenceFrameLibrary, robotModel, syncedRobot, handWrenchCalculators);
            action.getState().fromMessage(message);
            actionArray[message.getActionState().getActionIndex()] = action;
         }
         for (HandWrenchActionStateMessage message : latestUpdateMessage.getHandWrenchActions())
         {
            HandWrenchActionExecutor action = new HandWrenchActionExecutor(this, ros2);
            action.getState().fromMessage(message);
            actionArray[message.getActionState().getActionIndex()] = action;
         }
         for (PelvisHeightPitchActionStateMessage message : latestUpdateMessage.getPelvisHeightActions())
         {
            PelvisHeightPitchActionExecutor action = new PelvisHeightPitchActionExecutor(this, ros2, referenceFrameLibrary, syncedRobot);
            action.getState().fromMessage(message);
            actionArray[message.getActionState().getActionIndex()] = action;
         }
         for (WaitDurationActionStateMessage message : latestUpdateMessage.getWaitDurationActions())
         {
            WaitDurationActionExecutor action = new WaitDurationActionExecutor(this);
            action.getState().fromMessage(message);
            actionArray[message.getActionState().getActionIndex()] = action;
         }
         for (WalkActionStateMessage message : latestUpdateMessage.getWalkActions())
         {
            WalkActionExecutor action = new WalkActionExecutor(this,
                                                               ros2,
                                                               syncedRobot,
                                                               footstepTracker,
                                                               footstepPlanner,
                                                               footstepPlannerParameters,
                                                               walkingControllerParameters,
                                                               referenceFrameLibrary);
            action.getState().fromMessage(message);
            actionArray[message.getActionState().getActionIndex()] = action;
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
      for (RobotSide side : handWrenchCalculators.sides())
         handWrenchCalculators.get(side).compute();

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
         executionStatusCalculator.update(actionSequence, actionIndex, executionNextIndex);

         actionSequence.get(actionIndex).getState().setActionIndex(actionIndex);
         actionSequence.get(actionIndex).getState().setIsNextForExecution(executionStatusCalculator.getNextForExecution());
         actionSequence.get(actionIndex).getState().setIsToBeExecutedConcurrently(executionStatusCalculator.getExecuteWithPreviousAction()
                                                                                  || executionStatusCalculator.getExecuteWithNextAction());
         actionSequence.get(actionIndex).update();
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
            while (lastCurrentlyExecutingAction != null && lastCurrentlyExecutingAction.getDefinition().getExecuteWithNextAction());
         }
      }
      else if (manuallyExecuteSubscription.getMessageNotification().poll())
      {
         do
         {
            LogTools.info("Manually executing action: {}", actionSequence.get(executionNextIndex).getClass().getSimpleName());
            executeNextAction();
         }
         while (lastCurrentlyExecutingAction != null && lastCurrentlyExecutingAction.getDefinition().getExecuteWithNextAction());
      }

      if (lastCurrentlyExecutingAction != null && noCurrentActionIsExecuting())
      {
         lastCurrentlyExecutingAction = null;
         currentlyExecutingActions.clear();
      }
   }

   private void executeNextAction()
   {
      lastCurrentlyExecutingAction = actionSequence.get(executionNextIndex);
      // If automatic execution, we want to ensure it's able to execute before we perform the execution.
      // If it's unable to execute, disable automatic execution.
      if (automaticExecution)
      {
         if (!lastCurrentlyExecutingAction.getState().getCanExecute())
         {
            automaticExecution = false;
            // Early return
            return;
         }
      }
      lastCurrentlyExecutingAction.update();
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
         noCurrentActionIsExecuting &= !currentlyExecutingAction.getState().getIsExecuting();
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

   public int getExecutionNextIndex()
   {
      return executionNextIndex;
   }
}
