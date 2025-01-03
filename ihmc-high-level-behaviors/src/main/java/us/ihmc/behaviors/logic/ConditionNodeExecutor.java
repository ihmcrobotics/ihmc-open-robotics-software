package us.ihmc.behaviors.logic;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class ConditionNodeExecutor extends BehaviorTreeNodeExecutor<ConditionNodeState, ConditionNodeDefinition>
{
   private final ConditionNodeState state;
   private final ConditionNodeDefinition definition;

   public ConditionNodeExecutor(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new ConditionNodeState(id, crdtInfo, saveFileDirectory));

      state = getState();
      definition = getDefinition();
   }

   @Override
   public void update()
   {
      super.update();
   }
}
