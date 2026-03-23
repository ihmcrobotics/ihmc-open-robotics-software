package us.ihmc.behaviors.behaviorTree;

import gnu.trove.map.TObjectDoubleMap;
import gnu.trove.map.hash.TObjectDoubleHashMap;
import org.apache.commons.lang3.function.TriFunction;
import org.apache.logging.log4j.Level;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulation;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;
import us.ihmc.behaviors.behaviorTree.action.actions.AbilityHandActionComms;
import us.ihmc.behaviors.behaviorTree.control.FallbackNodeExecutor;
import us.ihmc.behaviors.behaviorTree.control.GotoNodeExecutor;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneExecutor;
import us.ihmc.behaviors.tools.interfaces.LogToolsLogger;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.ArrayList;
import java.util.List;

public class BehaviorTreeRootNodeExecutor extends BehaviorTreeNodeExecutor<BehaviorTreeRootNodeState, BehaviorTreeRootNodeDefinition>
      implements BehaviorTreeRootNode<BehaviorTreeNodeExecutor<?, ?>>
{
   private final BehaviorTreeExecutor tree;
   private final TriFunction<DRCRobotModel, ROS2NodeBuilder, RigidBodyTransformReadOnly, HumanoidKinematicsSimulation> kinematicsSimulationBuilder;
   private final List<LeafNodeExecutor<?, ?>> orderedLeaves = new ArrayList<>();
   private final List<ActionNodeExecutor<?, ?>> orderedActions = new ArrayList<>();
   private final List<FallbackNodeExecutor> fallbackNodes = new ArrayList<>();
   private final List<LeafNodeExecutor<?, ?>> currentlyExecutingLeaves = new ArrayList<>();
   private final List<LeafNodeExecutor<?, ?>> failedLeaves = new ArrayList<>();
   private final List<LeafNodeExecutor<?, ?>> successfulLeaves = new ArrayList<>();

   private ROS2Node previewROS2Node;
   private final ROS2ControllerHelper realROS2ControllerHelper;
   private final ROS2SyncedRobotModel realSyncedRobot;
   private final ControllerStatusTracker realControllerStatusTracker;
   private final SideDependentList<AbilityHandActionComms> realAbilityHandComms;
   private ROS2ControllerHelper previewROS2ControllerHelper;
   private ROS2SyncedRobotModel previewSyncedRobot;
   private ControllerStatusTracker previewControllerStatusTracker;
   private final SideDependentList<AbilityHandActionComms> previewAbilityHandComms = new SideDependentList<>();
   private boolean previewNeedsReset = false;
   private final TObjectDoubleMap<String> resetJointAngles = new TObjectDoubleHashMap<>();
   private HumanoidKinematicsSimulation previewSimulation;

   public BehaviorTreeRootNodeExecutor(
         long id,
         BehaviorTreeExecutor tree,
         WorkspaceResourceDirectory saveFileDirectory,
         ROS2ControllerHelper ros2ControllerHelper,
         TriFunction<DRCRobotModel, ROS2NodeBuilder, RigidBodyTransformReadOnly, HumanoidKinematicsSimulation> kinematicsSimulationBuilder,
         ROS2SyncedRobotModel syncedRobot,
         ControllerStatusTracker controllerStatusTracker,
         SideDependentList<AbilityHandActionComms> abilityHandComms,
         BehaviorTreeSceneExecutor scene
   )
   {
      super(new BehaviorTreeRootNodeState(id, tree.getCRDTInfo(), saveFileDirectory, syncedRobot.getRobotModel(), scene),
            ros2ControllerHelper,
            syncedRobot,
            controllerStatusTracker,
            abilityHandComms,
            scene);

      this.tree = tree;
      this.kinematicsSimulationBuilder = kinematicsSimulationBuilder;
      this.realROS2ControllerHelper = ros2ControllerHelper;
      this.realSyncedRobot = syncedRobot;
      this.realControllerStatusTracker = controllerStatusTracker;
      this.realAbilityHandComms = abilityHandComms;
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

      boolean previewMode = state.getPreviewModeEnabled();
      if (previewMode)
      {
         if (previewSimulation == null)
         {
            // Put preview simulation on a different domain ID
            ROS2NodeBuilder ros2NodeBuilder = new ROS2NodeBuilder().domainId(165); // TODO: Decide what domain is better
            previewROS2Node = ros2NodeBuilder.build("behavior_preview");
            previewROS2ControllerHelper = new ROS2ControllerHelper(previewROS2Node, robotModel.getSimpleRobotName());
            previewSyncedRobot = new ROS2SyncedRobotModel(rootNode.robotModel, previewROS2Node);
            previewControllerStatusTracker = new ControllerStatusTracker(new LogToolsLogger(), previewROS2ControllerHelper.getROS2Node(), previewSyncedRobot);
            for (RobotSide robotSide : RobotSide.values)
               previewAbilityHandComms.put(robotSide, new AbilityHandActionComms(robotSide, previewROS2ControllerHelper.getROS2Node()));
            for (OneDoFJointBasics oneDoFJoint : previewSyncedRobot.getFullRobotModel().getOneDoFJoints())
               resetJointAngles.put(oneDoFJoint.getName(), oneDoFJoint.getQ());
            RigidBodyTransformReadOnly walkingFrame = syncedRobot.getReferenceFrames().getMidFeetUnderPelvisFrame().getTransformToWorldFrame();
            previewSimulation = kinematicsSimulationBuilder.apply(robotModel, ros2NodeBuilder, walkingFrame);
         }

         previewSyncedRobot.update();
         for (RobotSide side : previewAbilityHandComms.sides())
            previewAbilityHandComms.get(side).update();

         if (previewNeedsReset)
         {
            previewNeedsReset = false;
            for (OneDoFJointBasics oneDoFJoint : realSyncedRobot.getFullRobotModel().getOneDoFJoints())
               resetJointAngles.put(oneDoFJoint.getName(), oneDoFJoint.getQ());
            previewSimulation.reinitialize(realSyncedRobot.getReferenceFrames().getPelvisFrame().getTransformToRoot(), resetJointAngles);
         }
      }
      else
         previewNeedsReset = true;

      scene.setSyncedRobot(previewMode ? previewSyncedRobot : realSyncedRobot);
      scene.update();

      BehaviorTreeTools.runForSubtreeNodes(this, node ->
      {
         node.ros2ControllerHelper = previewMode ? previewROS2ControllerHelper : realROS2ControllerHelper;
         node.syncedRobot = previewMode ? previewSyncedRobot : realSyncedRobot;
         node.controllerStatusTracker = previewMode ? previewControllerStatusTracker : realControllerStatusTracker;
         node.abilityHandComms = previewMode ? previewAbilityHandComms : realAbilityHandComms;
      });

      orderedLeaves.clear();
      orderedActions.clear();
      fallbackNodes.clear();
      currentlyExecutingLeaves.clear();
      failedLeaves.clear();
      successfulLeaves.clear();
      updateNodeListsRecursive(this);

      for (LeafNodeExecutor<?, ?> leaf : orderedLeaves)
         leaf.getState().validateDefinition(state.getOrderedNodes());

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
         // Break if the action to execute after is still executing
         int after = effectiveExecuteAfterLeafIndex(leafToExecute);
         if (after >= 0 && orderedLeaves.get(after).getState().getIsExecuting())
            break;

         leafToExecute.update(); // Make sure can execute is up to date
         if (leafToExecute.getState().getCanExecute())
         {
            state.getLogger().info("%s executing leaf: %s (%s)".formatted(state.getAutomaticExecution() ? "Automatically" : "Manually",
                                                                          leafToExecute.getDefinition().getName(),
                                                                          leafToExecute.getClass().getSimpleName()));
            leafToExecute.triggerExecution();
            if (!(leafToExecute instanceof GotoNodeExecutor))
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

      for (FallbackNodeExecutor fallbackNode : fallbackNodes) // catch group can't execute with anything above catch
         if (fallbackNode.getCatchLeaves().contains(leaf))
            after = Math.max(after, fallbackNode.getCatchLeaves().get(0).getState().getLeafIndex() - 1);

      return after;
   }

   @Override
   public void destroy()
   {
      super.destroy();

      if (previewROS2Node != null)
         previewROS2Node.destroy();
      if (previewSyncedRobot != null)
         previewSyncedRobot.destroy();
      if (previewSimulation != null)
         previewSimulation.destroy();
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
