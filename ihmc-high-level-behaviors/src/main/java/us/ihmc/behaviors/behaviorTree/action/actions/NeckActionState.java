package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.NeckActionStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;

public class NeckActionState extends ActionNodeState<NeckActionDefinition>
{
   public NeckActionState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new NeckActionDefinition(rootNode.getDefinition()), rootNode);
   }

   @Override
   public void update()
   {
   }

   public void toMessage(NeckActionStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(NeckActionStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }
}
