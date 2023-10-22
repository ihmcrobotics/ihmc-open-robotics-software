package us.ihmc.behaviors.behaviorTree;

import java.util.List;

public interface BehaviorTreeNode<T extends BehaviorTreeNode>
{
   List<T> getChildren();
}
