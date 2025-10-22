package us.ihmc.behaviors.behaviorTree.control;

import behavior_msgs.msg.dds.ActionSequenceStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class ActionSequenceState extends BehaviorTreeNodeState<ActionSequenceDefinition>
{
   public ActionSequenceState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new ActionSequenceDefinition(crdtInfo, saveFileDirectory), crdtInfo);
   }

   @Override
   public void update()
   {
      super.update();
   }

   public void toMessage(ActionSequenceStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(ActionSequenceStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }
}
