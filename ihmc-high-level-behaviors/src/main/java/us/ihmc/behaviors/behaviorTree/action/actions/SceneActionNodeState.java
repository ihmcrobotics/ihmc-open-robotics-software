package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.SceneActionNodeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;

public class SceneActionNodeState extends ActionNodeState<SceneActionNodeDefinition>
{
   public SceneActionNodeState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new SceneActionNodeDefinition(rootNode.getDefinition()), rootNode);
   }

   public void toMessage(SceneActionNodeStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(SceneActionNodeStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }
}
