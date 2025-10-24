package us.ihmc.rdx.behaviorTree;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;

public class RDXBehaviorTreeNonRootNode<S extends BehaviorTreeNodeState<D>,
                                        D extends BehaviorTreeNodeDefinition> extends RDXBehaviorTreeNode<S, D>
{
   protected final RDXBehaviorTreeRootNode rootNode;
   protected final DRCRobotModel robotModel;

   public RDXBehaviorTreeNonRootNode(S state, RDXBehaviorTreeRootNode rootNode)
   {
      super(state);

      this.rootNode = rootNode;
      this.robotModel = rootNode.getState().getRobotModel();
   }

   /** For creating a basic node. */ // TODO: Should not exist???
   public RDXBehaviorTreeNonRootNode(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(id, rootNode.getDefinition().getCRDTInfo(), rootNode.getDefinition().getSaveFileDirectory());

      this.rootNode = rootNode;
      this.robotModel = rootNode.getState().getRobotModel();
   }
}
