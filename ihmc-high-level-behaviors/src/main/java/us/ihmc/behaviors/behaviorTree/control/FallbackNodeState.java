package us.ihmc.behaviors.behaviorTree.control;

import behavior_msgs.msg.dds.FallbackNodeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.LeafNodeState;

import java.util.ArrayList;
import java.util.List;

public class FallbackNodeState extends BehaviorTreeNodeState<FallbackNodeDefinition>
{
   private final List<LeafNodeState<?>> childrenLeaves = new ArrayList<>();
   private final List<LeafNodeState<?>> tryLeaves = new ArrayList<>();
   private final List<LeafNodeState<?>> catchLeaves = new ArrayList<>();
   
   public FallbackNodeState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new FallbackNodeDefinition(rootNode.getDefinition()), rootNode);
   }

   @Override
   public void update()
   {
      super.update();

      childrenLeaves.clear();
      for (BehaviorTreeNodeState<?> child : getChildren())
         if (child instanceof LeafNodeState<?> leafNode)
            childrenLeaves.add(leafNode);

      tryLeaves.clear();
      catchLeaves.clear();
      if (!childrenLeaves.isEmpty())
      {
         int firstLeafIndex = childrenLeaves.get(0).getLeafIndex();

         for (LeafNodeState<?> child : childrenLeaves)
            if (child.getExecuteAfterLeafIndex() < firstLeafIndex)
               tryLeaves.add(child);
            else
               catchLeaves.add(child);
      }
   }

   public void toMessage(FallbackNodeStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(FallbackNodeStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }

   public List<LeafNodeState<?>> getChildrenLeaves()
   {
      return childrenLeaves;
   }

   public List<LeafNodeState<?>> getTryLeaves()
   {
      return tryLeaves;
   }

   public List<LeafNodeState<?>> getCatchLeaves()
   {
      return catchLeaves;
   }
}
