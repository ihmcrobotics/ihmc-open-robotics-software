package us.ihmc.behaviors.sequence;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class ActionSequenceExecutor extends BehaviorTreeNodeExecutor<ActionSequenceState, ActionSequenceDefinition>
{
   private final ActionSequenceState state;
   private final ActionSequenceDefinition definition;

   public ActionSequenceExecutor(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new ActionSequenceState(id, crdtInfo, saveFileDirectory));

      state = getState();
      definition = getDefinition();
   }

   @Override
   public void update()
   {
      super.update();
   }
}
