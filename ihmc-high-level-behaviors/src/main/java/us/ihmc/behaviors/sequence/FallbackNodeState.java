package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.FallbackNodeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeTools;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.List;

public class FallbackNodeState extends BehaviorTreeNodeState<FallbackNodeDefinition>
{
   private final FallbackNodeDefinition definition;

   public FallbackNodeState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new FallbackNodeDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      definition = getDefinition();
   }

   @Override
   public void update()
   {
      super.update();

      BehaviorTreeRootNodeState rootNode = BehaviorTreeTools.findRootNode(this);
      List<ActionNodeState<?>> actionChildren = rootNode.getActionChildren();

      // Need to find action ID
      if (definition.getGotoActionName() != null && definition.getGotoActionID().getValue() == 0)
      {
         int matchedActions = 0;
         for (ActionNodeState<?> action : actionChildren)
         {
            if (action.getDefinition().getName().equals(definition.getGotoActionName()))
            {
               definition.getGotoActionID().setValue(action.getActionIndex());
               ++matchedActions;
            }
         }

         if (matchedActions > 1)
            LogTools.error("There were {} actions with the name {}.", matchedActions, definition.getGotoActionName());
      }

      // Update action name
      if (definition.getGotoActionID().getValue() != 0)
      {
         int matchedActions = 0;
         for (ActionNodeState<?> action : actionChildren)
         {
            definition.setGotoActionName(action.getDefinition().getName());
            ++matchedActions;
         }

         if (matchedActions > 1)
            LogTools.error("There were {} actions with the name {}.", matchedActions, definition.getGotoActionName());
      }
   }

   public void toMessage(FallbackNodeStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(FallbackNodeStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());
   }
}
