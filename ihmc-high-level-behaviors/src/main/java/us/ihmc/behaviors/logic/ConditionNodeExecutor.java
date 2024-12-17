package us.ihmc.behaviors.logic;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.communication.crdt.CRDTGlobalInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class ConditionNodeExecutor extends BehaviorTreeNodeExecutor<ConditionNodeState, ConditionNodeDefinition>
{
   private final ConditionNodeState state;
   private final ConditionNodeDefinition definition;

   public ConditionNodeExecutor(long id, CRDTGlobalInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
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
