package us.ihmc.behaviors.logic;

import behavior_msgs.msg.dds.GotoNodeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeTools;
import us.ihmc.behaviors.sequence.LeafNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.List;

public class GotoNodeState extends LeafNodeState<GotoNodeDefinition>
{
   private final GotoNodeDefinition definition;

   public GotoNodeState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new GotoNodeDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      this.definition = getDefinition();
   }

   /**
    * Updates the definition gotoNodeName string for
    * saving an up to date human readable name in the JSON.
    * It also finds the correct node upon loading the name from JSON.
    */
   @Override
   public void validateFields(List<LeafNodeState<?>> leafNodes)
   {
      super.validateFields(leafNodes);

      String gotoNodeName = null;

      if (!definition.getGotoNext().getValue())
      {
         // We need to find the node by name
         // This happens when we load from JSON
         if (definition.getGotoNodeID().getValue() == 0)
         {
            for (BehaviorTreeNodeState<?> nodeToCompare : leafNodes)
            {
               if (nodeToCompare.getDefinition().getName().equals(definition.getGotoNodeName()))
               {
                  gotoNodeName = nodeToCompare.getDefinition().getName();
                  definition.getGotoNodeID().setValue(nodeToCompare.getID());
                  break;
               }
            }
         }
         else // Update the node's name for saving in human readable format
         {
            long gotoNodeID = definition.getGotoNodeID().getValue();
            for (BehaviorTreeNodeState<?> nodeToCompare : leafNodes)
            {
               if (nodeToCompare.getID() == gotoNodeID)
               {
                  gotoNodeName = nodeToCompare.getDefinition().getName();
               }
            }
         }
      }

      definition.updateAndSanitizeGotoNodeFields(gotoNodeName);
   }

   @Override
   public void update()
   {
      super.update();
   }

   public void toMessage(GotoNodeStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(GotoNodeStateMessage message)
   {
      getDefinition().fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }

   public LeafNodeState<?> findNodeToGoto()
   {
      BehaviorTreeRootNodeState rootState = BehaviorTreeTools.findRootNode(this);
      BehaviorTreeNodeState<?> nodeToGoto = rootState.getIDToNodeMap().get(definition.getGotoNodeID().getValue());
      if (nodeToGoto instanceof LeafNodeState<?> leafState)
      {
         return leafState;
      }
      else
      {
         LogTools.error("Node to goto is not a leaf node.");
         return null;
      }
   }
}
