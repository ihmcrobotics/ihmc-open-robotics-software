package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.SceneActionStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;

public class SceneActionState extends ActionNodeState<SceneActionDefinition>
{
   public SceneActionState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new SceneActionDefinition(rootNode.getDefinition()), rootNode);
   }

   public void toMessage(SceneActionStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(SceneActionStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }
}
