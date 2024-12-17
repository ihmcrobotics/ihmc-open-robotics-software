package us.ihmc.behaviors.logic;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.communication.crdt.CRDTGlobalInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class GotoNodeExecutor extends BehaviorTreeNodeExecutor<GotoNodeState, GotoNodeDefinition>
{
   private final GotoNodeState state;
   private final GotoNodeDefinition definition;

   public GotoNodeExecutor(long id, CRDTGlobalInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new GotoNodeState(id, crdtInfo, saveFileDirectory));

      state = getState();
      definition = getDefinition();
   }

   @Override
   public void update()
   {
      super.update();
   }
}
