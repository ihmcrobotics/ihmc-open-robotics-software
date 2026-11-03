package us.ihmc.behaviors.behaviorTree.action.actions;

import controller_msgs.HighLevelStateMessage;
import controller_msgs.RLModelSelectionMessage;
import controller_msgs.RLPolicyState;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.ROS2Input;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.tools.NonWallTimer;

import java.util.Arrays;
import java.util.Collections;
import java.util.Map;
import java.util.WeakHashMap;

/**
 * Sends one request for the definition's controller state, then finishes once the controller reports
 * that state. With RL_CONTROL and an RL model name, it then selects that model and also waits for the
 * controller to run it. Fails if either is not reached within the timeout, for example because the
 * transition is not allowed from the current state.
 */
public class HighLevelStateActionExecutor extends ActionNodeExecutor<HighLevelStateActionState, HighLevelStateActionDefinition>
{
   /**
    * One RLPolicyState subscription per controller helper, shared by every action of this type.
    * Executors have no teardown, so a subscription per action would leak each time a tree is reloaded.
    */
   private static final Map<ROS2ControllerHelper, ROS2Input<RLPolicyState>> RL_POLICY_STATE_INPUTS = Collections.synchronizedMap(new WeakHashMap<>());

   private final NonWallTimer executionTimer = new NonWallTimer();
   private final RLModelSelectionMessage rlModelSelectionMessage = new RLModelSelectionMessage();
   private boolean rlModelRequestSent;

   public HighLevelStateActionExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new HighLevelStateActionState(id, rootNode.getState()), rootNode);
   }

   @Override
   public void update()
   {
      super.update();

      executionTimer.update(Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp()));

      RLPolicyState rlPolicyState = getLatestRLPolicyState();
      if (rlPolicyState != null)
      {
         String availableModels = String.join("\n", availableModelNames(rlPolicyState));
         if (!availableModels.equals(state.getAvailableRLModels().getValue()))
            state.getAvailableRLModels().setValue(availableModels);
      }
   }

   @Override
   public void triggerExecution()
   {
      super.triggerExecution();

      HighLevelControllerName requested = definition.getHighLevelControllerName();
      state.getLogger().info("Requesting controller state %s".formatted(requested));

      HighLevelStateMessage message = new HighLevelStateMessage();
      message.setHighLevelControllerName(requested.toByte());
      ros2ControllerHelper.publishToController(message);

      rlModelRequestSent = false;
      executionTimer.reset();
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      state.setNominalExecutionDuration(definition.getTimeout());
      state.setElapsedExecutionTime(executionTimer.getElapsedTime());

      HighLevelControllerName requested = definition.getHighLevelControllerName();
      HighLevelControllerName current = controllerStatusTracker.getLatestKnownState();
      if (current != requested)
      {
         failIfTimedOut("Controller is in %s, not %s".formatted(current, requested));
         return;
      }

      String rlModelName = definition.getRLModelName();
      if (requested != HighLevelControllerName.RL_CONTROL || rlModelName.isBlank())
      {
         state.setIsExecuting(false);
         return;
      }

      RLPolicyState rlPolicyState = getLatestRLPolicyState();
      if (rlPolicyState == null)
      {
         failIfTimedOut("No RL policy state from the controller");
         return;
      }

      String[] availableModels = availableModelNames(rlPolicyState);
      int modelIndex = Arrays.asList(availableModels).indexOf(rlModelName);
      if (modelIndex < 0)
      {
         state.getLogger().error("RL model %s is not one of %s".formatted(rlModelName, String.join(", ", availableModels)));
         state.setIsExecuting(false);
         state.setFailed(true);
         return;
      }

      if (Byte.toUnsignedInt(rlPolicyState.getCurrentModel()) == modelIndex)
      {
         state.setIsExecuting(false);
         return;
      }

      if (!rlModelRequestSent)
      {
         state.getLogger().info("Selecting RL model %s".formatted(rlModelName));
         rlModelSelectionMessage.setDesiredModel((byte) modelIndex);
         rlModelSelectionMessage.setExecuteDesiredModel(true);
         ros2ControllerHelper.publishToController(rlModelSelectionMessage);
         rlModelRequestSent = true;
      }
      failIfTimedOut("Controller is running RL model %s, not %s"
                           .formatted(availableModels[Math.min(Byte.toUnsignedInt(rlPolicyState.getCurrentModel()), availableModels.length - 1)], rlModelName));
   }

   private void failIfTimedOut(String reason)
   {
      if (executionTimer.isRunning(definition.getTimeout()))
         return;

      state.getLogger().error("%s after %.1f s".formatted(reason, definition.getTimeout()));
      state.setIsExecuting(false);
      state.setFailed(true);
   }

   private RLPolicyState getLatestRLPolicyState()
   {
      if (ros2ControllerHelper == null)
         return null;
      ROS2Input<RLPolicyState> input = RL_POLICY_STATE_INPUTS.computeIfAbsent(ros2ControllerHelper,
                                                                              helper -> helper.subscribeToController(RLPolicyState.class));
      // Before the first message the input holds an empty one, which would read as no models.
      return input.hasReceivedFirstMessage() ? input.getLatest() : null;
   }

   private static String[] availableModelNames(RLPolicyState rlPolicyState)
   {
      int count = Math.min(Byte.toUnsignedInt(rlPolicyState.getNumAvailableModels()), rlPolicyState.getAvailableModels().size());
      String[] names = new String[count];
      for (int i = 0; i < count; i++)
         names[i] = rlPolicyState.getAvailableModels().getAsString(i);
      return names;
   }
}
