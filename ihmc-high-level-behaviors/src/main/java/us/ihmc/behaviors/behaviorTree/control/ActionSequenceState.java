package us.ihmc.behaviors.behaviorTree.control;

import behavior_msgs.msg.dds.ActionSequenceStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;

public class ActionSequenceState extends BehaviorTreeNodeState<ActionSequenceDefinition>
{
   public ActionSequenceState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new ActionSequenceDefinition(rootNode.getDefinition()), rootNode);
   }

   @Override
   public void update()
   {
      super.update();
   }

   public void toMessage(ActionSequenceStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(ActionSequenceStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }
}
