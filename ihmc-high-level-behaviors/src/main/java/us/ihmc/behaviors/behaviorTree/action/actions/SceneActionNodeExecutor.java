package us.ihmc.behaviors.behaviorTree.action.actions;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;

public class SceneActionNodeExecutor extends ActionNodeExecutor<SceneActionNodeState, SceneActionNodeDefinition>
{
   public SceneActionNodeExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new SceneActionNodeState(id, rootNode.getState()), rootNode);
   }

   @Override
   public void update()
   {
      super.update();
   }

   @Override
   public void triggerExecution()
   {
      super.triggerExecution();

      state.getLogger().info("Executing scene action...");
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      // TODO: Implement scene action execution logic
      state.setIsExecuting(false); // Complete immediately for now
   }
}
