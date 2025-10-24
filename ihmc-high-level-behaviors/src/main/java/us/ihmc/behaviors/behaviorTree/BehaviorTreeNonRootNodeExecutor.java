package us.ihmc.behaviors.behaviorTree;

import us.ihmc.avatar.drcRobot.DRCRobotModel;

public class BehaviorTreeNonRootNodeExecutor<S extends BehaviorTreeNodeState<D>,
                                             D extends BehaviorTreeNodeDefinition> extends BehaviorTreeNodeExecutor<S, D>
{
   protected final BehaviorTreeRootNodeExecutor rootNode;
   protected final DRCRobotModel robotModel;

   public BehaviorTreeNonRootNodeExecutor(S state, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(state);

      this.rootNode = rootNode;
      this.robotModel = rootNode.getState().getRobotModel();
   }

   /** For creating a basic node. */ // TODO: Should not exist???
   public BehaviorTreeNonRootNodeExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(id, rootNode.getDefinition().getCRDTInfo(), rootNode.getDefinition().getSaveFileDirectory());

      this.rootNode = rootNode;
      this.robotModel = rootNode.getState().getRobotModel();
   }
}
