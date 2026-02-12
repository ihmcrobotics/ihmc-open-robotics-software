package us.ihmc.behaviors.behaviorTree;

import org.apache.logging.log4j.Level;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;
import us.ihmc.behaviors.behaviorTree.action.actions.AbilityHandActionComms;
import us.ihmc.behaviors.behaviorTree.control.FallbackNodeExecutor;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneExecutor;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.ArrayList;
import java.util.List;

public class BehaviorTreeRootNodeExecutor extends BehaviorTreeNodeExecutor<BehaviorTreeRootNodeState, BehaviorTreeRootNodeDefinition>
      implements BehaviorTreeRootNode<BehaviorTreeNodeExecutor<?, ?>>
{
   private final BehaviorTreeExecutor tree;
   private final List<LeafNodeExecutor<?, ?>> orderedLeaves = new ArrayList<>();
   private final List<ActionNodeExecutor<?, ?>> orderedActions = new ArrayList<>();
   private final List<FallbackNodeExecutor> fallbackNodes = new ArrayList<>();
   private final List<LeafNodeExecutor<?, ?>> currentlyExecutingLeaves = new ArrayList<>();
   private final List<LeafNodeExecutor<?, ?>> failedLeaves = new ArrayList<>();
   private final List<LeafNodeExecutor<?, ?>> successfulLeaves = new ArrayList<>();

   public BehaviorTreeRootNodeExecutor(long id,
                                       BehaviorTreeExecutor tree,
                                       WorkspaceResourceDirectory saveFileDirectory,
                                       ROS2ControllerHelper ros2ControllerHelper,
                                       ROS2SyncedRobotModel syncedRobot,
                                       ControllerStatusTracker controllerStatusTracker,
                                       SideDependentList<AbilityHandActionComms> abilityHandComms,
                                       BehaviorTreeSceneExecutor scene)
   {
      super(new BehaviorTreeRootNodeState(id, tree.getCRDTInfo(), saveFileDirectory, syncedRobot.getRobotModel(), scene),
            ros2ControllerHelper,
            syncedRobot,
            controllerStatusTracker,
            abilityHandComms,
            scene);

      this.tree = tree;
   }

   @Override
   public void tick()
   {
      super.tick();

      // TODO: Tick children
   }

   private void updateNodeListsRecursive(BehaviorTreeNodeExecutor<?, ?> node)
   {
      for (BehaviorTreeNodeExecutor<?, ?> child : node.getChildren())
      {
         if (child instanceof LeafNodeExecutor<?, ?> leaf)
         {
            orderedLeaves.add(leaf);
            if (leaf.getState().getIsExecuting())
               currentlyExecutingLeaves.add(leaf);

            if (child instanceof ActionNodeExecutor<?, ?> actionNode)
               orderedActions.add(actionNode);
         }
         else if (child instanceof FallbackNodeExecutor fallbackNode)
         {
            fallbackNode.update();
            fallbackNodes.add(fallbackNode);
         }

         updateNodeListsRecursive(child);
      }
   }

   @Override
   public void update()
   {
      super.update();

      orderedLeaves.clear();
      orderedActions.clear();
      fallbackNodes.clear();
      currentlyExecutingLeaves.clear();
      failedLeaves.clear();
      successfulLeaves.clear();
      updateNodeListsRecursive(this);

      for (LeafNodeExecutor<?, ?> leaf : orderedLeaves)
         leaf.getState().validateDefinition(state.getOrderedLeaves());

      // Determine the concurrent group
      int next = state.getExecutionNextIndex();
      for (int i = 0; i < state.getOrderedLeaves().size(); i++)
      {
         LeafNodeExecutor<?, ?> leaf = orderedLeaves.get(i);
         int after = effectiveExecuteAfterLeafIndex(leaf);
         leaf.getState().setIsNextForExecution(i >= next && after < next);
      }

      for (LeafNodeExecutor<?, ?> currentlyExecutingLeaf : currentlyExecutingLeaves)
      {
         currentlyExecutingLeaf.updateCurrentlyExecuting();

         if (!currentlyExecutingLeaf.getState().getIsExecuting())
            leafCeasedExecution(currentlyExecutingLeaf);
      }
      currentlyExecutingLeaves.removeAll(failedLeaves);
      currentlyExecutingLeaves.removeAll(successfulLeaves);

      boolean tryExecuteNext = state.getAutomaticExecution() || state.pollManualExecutionRequested();
      executionLoop:
      while (tryExecuteNext)
      {
         if (isEndOfSequence())
         {
            state.getLogger().info("End of sequence.");
            state.setAutomaticExecution(false);
            break;
         }

         LeafNodeExecutor<?, ?> leafToExecute = orderedLeaves.get(state.getExecutionNextIndex());

         if (!state.getAutomaticExecution() && !leafToExecute.getState().getIsNextForExecution())
            break;

         for (FallbackNodeExecutor fallbackNode : fallbackNodes)
            if (fallbackNode.tryLeafIsBlocking(leafToExecute))
               break executionLoop;
         // Break if anything earlier than effective after execute is still going
         for (int i = effectiveExecuteAfterLeafIndex(leafToExecute); i >= 0; i--)
            if (state.getOrderedLeaves().get(i).getIsExecuting())
               break executionLoop;

         leafToExecute.update(); // Make sure can execute is up to date
         if (leafToExecute.getState().getCanExecute())
         {
            state.getLogger().info("%s executing leaf: %s (%s)".formatted(state.getAutomaticExecution() ? "Automatically" : "Manually",
                                                                          leafToExecute.getDefinition().getName(),
                                                                          leafToExecute.getClass().getSimpleName()));
            leafToExecute.triggerExecution();
            state.stepForwardNextExecutionIndex();
            if (!leafToExecute.getState().getIsExecuting()) // Handle immediately ceased execution
            {
               boolean isTryLeaf = leafCeasedExecution(leafToExecute);
               if (!isTryLeaf && leafToExecute.getState().getFailed())
                  tryExecuteNext = false;
            }
            else
               currentlyExecutingLeaves.add(leafToExecute);
         }
         else
         {
            state.getLogger().error("Cannot execute leaf: %s: %s\n%s".formatted(leafToExecute.getClass().getSimpleName(),
                                                                                leafToExecute.getDefinition().getName(),
                                                                                leafToExecute.getCantExecuteMessage()));
            tryExecuteNext = false;
         }
      }

      if (state.pollFailureResetRequested())
         for (int i = 0; i < state.getOrderedLeaves().size(); i++)
         {
            state.getOrderedLeaves().get(i).setFailed(false);
            state.getOrderedLeaves().get(i).setIsExecuting(false);
         }
   }

   private boolean leafCeasedExecution(LeafNodeExecutor<?, ?> leaf)
   {
      boolean failed = leaf.getState().getFailed();
      if (failed)
         failedLeaves.add(leaf);
      else
         successfulLeaves.add(leaf);

      String name = leaf.getDefinition().getName();
      if (leaf.getState() instanceof ActionNodeState<?> actionNodeState)
      {
         String resultMessage = failed ? "failed after" : "completed successfully in";
         double elapsedExecutionTime = actionNodeState.getElapsedExecutionTime();
         state.getLogger().log(failed ? Level.ERROR : Level.INFO, "Action %s %.2f s: %s".formatted(resultMessage, elapsedExecutionTime, name));
      }
      else
      {
         String resultMessage = failed ? "failed" : "completed successfully";
         state.getLogger().log(failed ? Level.ERROR : Level.INFO, "Leaf %s: %s".formatted(resultMessage, name));
      }

      boolean isTryLeaf = false;
      for (FallbackNodeExecutor fallbackNode : fallbackNodes)
         if (fallbackNode.getChildrenLeaves().contains(leaf))
            isTryLeaf |= fallbackNode.leafCeasedExecution(leaf);

      if (!isTryLeaf && failed)
      {
         state.getLogger().error("A leaf failed. Disabling automatic execution.\n   Failed: %s".formatted(leaf));
         state.setAutomaticExecution(false);
         state.setExecutionNextIndex(leaf.getState().getLeafIndex()); // It is convenient to stay at the failed node to debug/retry
      }

      return isTryLeaf;
   }

   private int effectiveExecuteAfterLeafIndex(LeafNodeExecutor<?, ?> leaf)
   {
      int i = leaf.getState().getLeafIndex();

      if (!state.getConcurrencyEnabled())
         return i - 1;

      int after = leaf.getState().getExecuteAfterLeafIndex();

      for (int j = after + 1; j < i; j++) // Might have to wait on nearer leaves
         after = Math.max(after, state.getOrderedLeaves().get(j).getExecuteAfterLeafIndex());

      for (FallbackNodeExecutor fallbackNode : fallbackNodes) // catch group can't execute with anything above catch
         if (fallbackNode.getCatchLeaves().contains(leaf))
            after = Math.max(after, fallbackNode.getCatchLeaves().get(0).getState().getLeafIndex() - 1);

      return after;
   }

   public boolean isEndOfSequence()
   {
      return state.getExecutionNextIndex() >= orderedLeaves.size();
   }

   public List<ActionNodeExecutor<?, ?>> getOrderedActions()
   {
      return orderedActions;
   }

   public List<LeafNodeExecutor<?, ?>> getOrderedLeaves()
   {
      return orderedLeaves;
   }

   public List<LeafNodeExecutor<?, ?>> getFailedLeaves()
   {
      return failedLeaves;
   }

   public List<LeafNodeExecutor<?, ?>> getSuccessfulLeaves()
   {
      return successfulLeaves;
   }

   public List<LeafNodeExecutor<?, ?>> getCurrentlyExecutingLeaves()
   {
      return currentlyExecutingLeaves;
   }

   // Getters are in here so there's not getters in base node for root stuff

   public BehaviorTreeExecutor getTree()
   {
      return tree;
   }

   public ROS2ControllerHelper getRos2ControllerHelper()
   {
      return ros2ControllerHelper;
   }

   public ROS2SyncedRobotModel getSyncedRobot()
   {
      return syncedRobot;
   }

   public ControllerStatusTracker getControllerStatusTracker()
   {
      return controllerStatusTracker;
   }

   public SideDependentList<AbilityHandActionComms> getAbilityHandComms()
   {
      return abilityHandComms;
   }

   public BehaviorTreeSceneExecutor getScene()
   {
      return scene;
   }
}
