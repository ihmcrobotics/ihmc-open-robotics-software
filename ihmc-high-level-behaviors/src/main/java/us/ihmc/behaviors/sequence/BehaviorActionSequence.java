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
import us.ihmc.behaviors.sequence.ros2.ROS2BehaviorActionSequence;
import us.ihmc.behaviors.tools.ROS2HandWrenchCalculator;
import us.ihmc.behaviors.tools.walkingController.WalkingFootstepTracker;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
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

   /** TODO: Make non-static and assign as actions are created. */
   public static final MutableLong NEXT_ID = new MutableLong();

   private final DRCRobotModel robotModel;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final ROS2SyncedRobotModel syncedRobot;
   private final WalkingFootstepTracker footstepTracker;
   private final SideDependentList<ROS2HandWrenchCalculator> handWrenchCalculators = new SideDependentList<>();
   private final FootstepPlanningModule footstepPlanner;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final WalkingControllerParameters walkingControllerParameters;

   private final LinkedList<BehaviorActionExecutor> actionSequence = new LinkedList<>();
   private boolean automaticExecution = false;
   private int executionNextIndex = 0;
   private final BehaviorActionExecutionStatusCalculator executionStatusCalculator = new BehaviorActionExecutionStatusCalculator();
   private final List<BehaviorActionExecutor> currentlyExecutingActions = new ArrayList<>();
   private BehaviorActionExecutor lastCurrentlyExecutingAction = null;

   private final Throttler oneHertzThrottler = new Throttler();
   private final ActionSequenceUpdateMessage actionSequenceStatusMessage = new ActionSequenceUpdateMessage();
   private ActionExecutionStatusMessage nothingExecutingStatusMessage = new ActionExecutionStatusMessage();
   private final ActionsExecutionStatusMessage actionsExecutionStatusMessage = new ActionsExecutionStatusMessage();

   public BehaviorActionSequence(DRCRobotModel robotModel, ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.robotModel = robotModel;
      this.referenceFrameLibrary = referenceFrameLibrary;

      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2.getROS2NodeInterface());
      footstepTracker = new WalkingFootstepTracker(ros2.getROS2NodeInterface(), robotModel.getSimpleRobotName());
      for (RobotSide side : RobotSide.values)
         handWrenchCalculators.put(side, new ROS2HandWrenchCalculator(side, syncedRobot));
      footstepPlanner = FootstepPlanningModuleLauncher.createModule(robotModel);
      footstepPlannerParameters = robotModel.getFootstepPlannerParameters();
      walkingControllerParameters = robotModel.getWalkingControllerParameters();

      addCommonFrames(referenceFrameLibrary, syncedRobot);
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
      // Creates a whole new list of actions and sets their state

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
         ros2.publish(ROS2BehaviorActionSequence.ACTIONS_EXECUTION_STATUS, actionsExecutionStatusMessage);
      }
      else
      {
         nothingExecutingStatusMessage = actionsExecutionStatusMessage.getActionStatusList().add();
         nothingExecutingStatusMessage.setActionIndex(-1);
         ros2.publish(ROS2BehaviorActionSequence.ACTIONS_EXECUTION_STATUS, actionsExecutionStatusMessage);
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
      ros2.publish(ROS2BehaviorActionSequence.EXECUTION_NEXT_INDEX_STATUS_TOPIC, executionNextIndexStatusMessage);
      automaticExecutionStatusMessage.setData(automaticExecution);
      ros2.publish(ROS2BehaviorActionSequence.AUTOMATIC_EXECUTION_STATUS_TOPIC, automaticExecutionStatusMessage);

      if (oneHertzThrottler.run(1.0))
      {
         BehaviorActionSequenceTools.packActionSequenceUpdateMessage(actionSequence, actionSequenceStatusMessage);
         ros2.publish(ROS2BehaviorActionSequence.SEQUENCE_STATUS_TOPIC, actionSequenceStatusMessage);
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
