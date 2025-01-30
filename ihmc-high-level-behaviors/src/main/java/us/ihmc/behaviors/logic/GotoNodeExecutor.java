package us.ihmc.behaviors.logic;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeTools;
import us.ihmc.behaviors.sequence.LeafNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class GotoNodeExecutor extends LeafNodeExecutor<GotoNodeState, GotoNodeDefinition>
{
   private final GotoNodeState state;
   private final GotoNodeDefinition definition;

   public GotoNodeExecutor(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new GotoNodeState(id, crdtInfo, saveFileDirectory));

      state = getState();
      definition = getDefinition();
   }


   @Override
   public void updateCurrentlyExecuting()
   {
      state.setIsExecuting(false); // Completes immediately
   }

   @Override
   public void triggerActionExecution()
   {
      super.triggerActionExecution();

      if (!definition.getGotoNext().getValue())
      {
         BehaviorTreeRootNodeExecutor rootExecutor = BehaviorTreeTools.findRootNode(this);
         BehaviorTreeNodeExecutor<?, ?> nodeToGoto = rootExecutor.getIDToNodeMap().get(definition.getGotoNodeID().getValue());
         if (nodeToGoto instanceof LeafNodeExecutor<?, ?> leafNode)
         {
            rootExecutor.getState().setExecutionNextIndex(leafNode.getState().getActionIndex());
         }
         else
         {
            state.setFailed(true);
            LogTools.error("Node to goto is not a leaf node.");
         }
      }
   }
}
