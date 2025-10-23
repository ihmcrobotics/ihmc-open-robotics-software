package us.ihmc.behaviors.behaviorTree;

import gnu.trove.map.hash.TLongObjectHashMap;
import org.apache.logging.log4j.Level;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.condition.ConditionNodeState;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;
import us.ihmc.behaviors.behaviorTree.control.FallbackNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.log.LogTools;
import us.ihmc.perception.detections.DetectionManager;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.ArrayList;
import java.util.List;

public class BehaviorTreeRootNodeExecutor extends BehaviorTreeNodeExecutor<BehaviorTreeRootNodeState, BehaviorTreeRootNodeDefinition>
      implements BehaviorTreeRootNode<BehaviorTreeNodeExecutor<?, ?>>
{
   private final TLongObjectHashMap<BehaviorTreeNodeExecutor<?, ?>> idToNodeMap = new TLongObjectHashMap<>();
   private final List<LeafNodeExecutor<?, ?>> orderedLeaves = new ArrayList<>();
   private final List<ActionNodeExecutor<?, ?>> orderedActions = new ArrayList<>();
   private final List<FallbackNodeExecutor> fallbackNodes = new ArrayList<>();
   private final List<LeafNodeExecutor<?, ?>> currentlyExecutingLeaves = new ArrayList<>();
   private final List<LeafNodeExecutor<?, ?>> failedLeaves = new ArrayList<>();
   private final List<LeafNodeExecutor<?, ?>> successfulLeaves = new ArrayList<>();
   private final List<LeafNodeExecutor<?, ?>> failedLeavesWithoutFallback = new ArrayList<>();

   public BehaviorTreeRootNodeExecutor(long id,
                                       CRDTInfo crdtInfo,
                                       WorkspaceResourceDirectory saveFileDirectory,
                                       DRCRobotModel robotModel,
                                       ROS2ControllerHelper ros2ControllerHelper,
                                       ROS2SyncedRobotModel syncedRobot,
                                       ReferenceFrameLibrary referenceFrameLibrary,
                                       SceneGraph sceneGraph,
                                       DetectionManager detectionManager)
   {
      super(new BehaviorTreeRootNodeState(id, crdtInfo, saveFileDirectory));
   }

   @Override
   public void tick()
   {
      super.tick();

      // TODO: Tick children
   }

   @Override
   public void update()
   {
      super.update();

      idToNodeMap.clear();
      orderedLeaves.clear();
      orderedActions.clear();
      fallbackNodes.clear();
      currentlyExecutingLeaves.clear();
      updateSubtree(this);

      for (LeafNodeExecutor<?, ?> leaf : orderedLeaves)
      {
         leaf.getState().validateFields(state.getOrderedLeaves());
      }

      // Update concurrency ranks
      for (int i = 0; i < state.getOrderedLeaves().size(); i++)
      {
         state.getOrderedLeaves().get(i).setConcurrencyRank(1);

         int j = i - 1;
         for (; j >= 0; j--)
         {
            int thisExecuteAfterLeafIndex = state.getOrderedLeaves().get(i).getExecuteAfterLeafIndex();
            int executeAfterLeafIndexToCompare = state.getOrderedLeaves().get(j).getExecuteAfterLeafIndex();
            if (thisExecuteAfterLeafIndex == executeAfterLeafIndexToCompare)
            {
               state.getOrderedLeaves().get(i).setConcurrencyRank(2);
            }
         }
      }

      // Update is next for execution
      for (int i = 0; i < state.getOrderedLeaves().size(); i++)
      {
         int executionNextIndex = state.getExecutionNextIndex();
         if (i < executionNextIndex)
         {
            state.getOrderedLeaves().get(i).setIsNextForExecution(false);
         }
         else if (i == executionNextIndex)
         {
            state.getOrderedLeaves().get(i).setIsNextForExecution(true);
         }
         else if (state.getOrderedLeaves().get(i).getExecuteAfterLeafIndex() < executionNextIndex)
         {
            state.getOrderedLeaves().get(i).setIsNextForExecution(true);
         }
         else
         {
            state.getOrderedLeaves().get(i).setIsNextForExecution(false);
         }
      }

      failedLeaves.clear();
      successfulLeaves.clear();
      for (LeafNodeExecutor<?, ?> currentlyExecutingLeaf : currentlyExecutingLeaves)
      {
         currentlyExecutingLeaf.updateCurrentlyExecuting();

         // This leaf has completed on this update
         if (!currentlyExecutingLeaf.getState().getIsExecuting())
         {
            boolean success = !currentlyExecutingLeaf.getState().getFailed();
            if (success)
               successfulLeaves.add(currentlyExecutingLeaf);
            else
               failedLeaves.add(currentlyExecutingLeaf);

            String name = currentlyExecutingLeaf.getDefinition().getName();
            if (currentlyExecutingLeaf.getState() instanceof ActionNodeState<?> actionNodeState)
            {
               String resultMessage = success ? "completed successfully in" : "failed after";
               double elapsedExecutionTime = actionNodeState.getElapsedExecutionTime();
               LogTools.log(success ? Level.INFO : Level.ERROR, "Action %s %.2f s: %s".formatted(resultMessage, elapsedExecutionTime, name));
            }
            else
            {
               String resultMessage = success ? "completed successfully" : "failed";
               LogTools.log(success ? Level.INFO : Level.ERROR, "Leaf %s: %s".formatted(resultMessage, name));
            }

            if (currentlyExecutingLeaf.getState() instanceof ConditionNodeState conditionNodeState)
            {
               if (conditionNodeState.isConditionMet())
               {
                  state.setExecutionNextIndex(conditionNodeState.getLeafIndex() + 3); // TODO Fix fallback action instead of doing this
                  conditionNodeState.setConditionValue(false);
                  conditionNodeState.setEvaluatingConditionValue(false);
               }
            }
         }
      }
      currentlyExecutingLeaves.removeAll(failedLeaves);
      currentlyExecutingLeaves.removeAll(successfulLeaves);

      failedLeavesWithoutFallback.clear();
      failedLeavesWithoutFallback.addAll(failedLeaves);
      for (FallbackNodeExecutor fallbackNode : fallbackNodes)
      {
         fallbackNode.update();

         if (!fallbackNode.getFallbackLeaves().isEmpty())
         {
            failedLeavesWithoutFallback.removeAll(fallbackNode.getTryLeaves());
         }
      }

      // Handle skipping the fallback if try leaf group is successful
      boolean leafEnded = !failedLeaves.isEmpty() || !successfulLeaves.isEmpty();
      if (leafEnded && currentlyExecutingLeaves.isEmpty() && !isEndOfSequence())
      {
         LeafNodeExecutor<?, ?> nextNodeToExecute = orderedLeaves.get(state.getExecutionNextIndex());

         for (FallbackNodeExecutor fallbackNode : fallbackNodes)
         {
            if (fallbackNode.getFallbackLeaves().indexOf(nextNodeToExecute) == 0)
            {
               boolean anyFailed = false;
               for (LeafNodeExecutor<?, ?> tryLeaf : fallbackNode.getTryLeaves())
               {
                  if (tryLeaf.getState().getFailed())
                  {
                     anyFailed = true;
                     break;
                  }
               }

               if (anyFailed) // Nothing is executing and a tried leaf failed -- falling back
               {
                  LogTools.warn("Leaves failed, fallback leaves are next for execution.");
               }
               else // Nothing is executing and none of the try leaves failed -- skip fallback
               {
                  LeafNodeExecutor<?, ?> lastFallbackLeaf = fallbackNode.getFallbackLeaves().get(fallbackNode.getFallbackLeaves().size() - 1);
                  LogTools.info("Leaves successful, skipping fallback leaves(s)");
                  state.setExecutionNextIndex(lastFallbackLeaf.getState().getLeafIndex() + 1);
               }
            }
         }
      }

      if (state.getAutomaticExecution())
      {
         if (isEndOfSequence())
         {
            state.getLogger().info("End of sequence.");
            state.setAutomaticExecution(false);
         }
         else if (!failedLeavesWithoutFallback.isEmpty())
         {
            state.getLogger().error("A leaf failed. Disabling automatic execution.\n   Failed: %s".formatted(failedLeavesWithoutFallback));
            state.setAutomaticExecution(false);
         }
         else
         {
            while (shouldExecuteNextLeaf())
            {
               var nextLeaf = orderedLeaves.get(state.getExecutionNextIndex());
               state.getLogger().info("Automatically executing leaf: %s (%s)".formatted(nextLeaf.getDefinition().getName(),
                                                                                        nextLeaf.getClass().getSimpleName()));
               executeNextLeaf();
            }
         }
      }
      else if (state.pollManualExecutionRequested())
      {
         while (shouldExecuteNextLeaf())
         {
            var nextLeaf = orderedLeaves.get(state.getExecutionNextIndex());
            state.getLogger().info("Manually executing leaf: %s (%s)".formatted(nextLeaf.getDefinition().getName(),
                                                                                nextLeaf.getClass().getSimpleName()));
            executeNextLeaf();
         }
      }

      if (state.pollFailureResetRequested())
      {
         failedLeaves.clear();
         for (int i = 0; i < state.getOrderedLeaves().size(); i++)
         {
            state.getOrderedLeaves().get(i).setFailed(false);
         }
      }
   }

   public void updateSubtree(BehaviorTreeNodeExecutor<?, ?> node)
   {
      idToNodeMap.put(node.getState().getID(), node);

      for (BehaviorTreeNodeExecutor<?, ?> child : node.getChildren())
      {
         if (child instanceof LeafNodeExecutor<?, ?> leaf)
         {
            orderedLeaves.add(leaf);
            if (leaf.getState().getIsExecuting())
            {
               currentlyExecutingLeaves.add(leaf);
            }

            if (child instanceof ActionNodeExecutor<?, ?> actionNode)
               orderedActions.add(actionNode);
         }
         else if (child instanceof FallbackNodeExecutor fallbackNode)
         {
            fallbackNodes.add(fallbackNode);
         }

         updateSubtree(child);
      }
   }

   private void executeNextLeaf()
   {
      LeafNodeExecutor<?, ?> leafToExecute = orderedLeaves.get(state.getExecutionNextIndex());

      state.getLogger().info("Triggering leaf execution: %s (%s)".formatted(leafToExecute.getDefinition().getName(),
                                                                            leafToExecute.getClass().getSimpleName()));
      leafToExecute.update();
      leafToExecute.triggerExecution();
      currentlyExecutingLeaves.add(leafToExecute);
      state.stepForwardNextExecutionIndex();
   }

   private boolean shouldExecuteNextLeaf()
   {
      if (isEndOfSequence())
      {
         return false;
      }

      LeafNodeExecutor<?, ?> nextNodeToExecute = orderedLeaves.get(state.getExecutionNextIndex());

      // If a fallback sequence is up next, block if any corresponding try leaves are executing
      for (FallbackNodeExecutor fallbackNode : fallbackNodes)
      {
         if (fallbackNode.getFallbackLeaves().indexOf(nextNodeToExecute) == 0) // The first fallback leaf is next
         {
            for (LeafNodeExecutor<?, ?> tryLeaf : fallbackNode.getTryLeaves())
            {
               if (tryLeaf.getState().getIsExecuting())
               {
                  return false;
               }
            }
         }
      }

      if (!nextNodeToExecute.getState().getCanExecute())
      {
         state.getLogger().error("Cannot execute leaf: %s\n%s".formatted(nextNodeToExecute.getDefinition().getName(),
                                                                         nextNodeToExecute.getCantExecuteMessage()));
         state.setAutomaticExecution(false);
         return false;
      }

      if (state.getConcurrencyEnabled())
      {
         int executeAfterLeafIndex = nextNodeToExecute.getState().getExecuteAfterLeafIndex();

         if (executeAfterLeafIndex < 0) // Execute after beginning
         {
            return true;
         }
         else
         {
            return !orderedLeaves.get(executeAfterLeafIndex).getState().getIsExecuting();
         }
      }
      else
      {
         return currentlyExecutingLeaves.isEmpty();
      }
   }

   public boolean isEndOfSequence()
   {
      return state.getExecutionNextIndex() >= orderedLeaves.size();
   }
   
   public TLongObjectHashMap<BehaviorTreeNodeExecutor<?, ?>> getIDToNodeMap()
   {
      return idToNodeMap;
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
}
