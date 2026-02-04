package us.ihmc.behaviors.behaviorTree.control.buildingExploration;

import behavior_msgs.msg.dds.BuildingExplorationStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.control.door.DoorTraversalState;

import javax.annotation.Nullable;
import java.util.Stack;

public class BuildingExplorationState extends BehaviorTreeNodeState<BuildingExplorationDefinition>
{
   @Nullable
   private DoorTraversalState doorTraversalState;

   public BuildingExplorationState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new BuildingExplorationDefinition(rootNode.getDefinition()), rootNode);
   }

   @Override
   public void update()
   {
      super.update();

      // We assume that there is a possible DoorTraversal as a child of BuildingExploration
      for (BehaviorTreeNodeState<?> child : getChildren())
      {
         if (child instanceof DoorTraversalState doorTraversalState)
         {
            this.doorTraversalState = doorTraversalState;
         }
      }
   }

   public void toMessage(BuildingExplorationStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(BuildingExplorationStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }

   @Nullable
   public DoorTraversalState getDoorTraversalState()
   {
      return doorTraversalState;
   }
}
