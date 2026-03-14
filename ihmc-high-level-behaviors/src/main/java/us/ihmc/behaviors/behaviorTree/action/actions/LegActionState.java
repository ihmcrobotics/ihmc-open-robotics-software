package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.LegActionStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;
import us.ihmc.communication.crdt.CRDTDetachableReferenceFrame;

public class LegActionState extends ActionNodeState<LegActionDefinition>
{
   private final CRDTDetachableReferenceFrame footFrame;

   public LegActionState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new LegActionDefinition(rootNode.getDefinition()), rootNode);

      footFrame = new CRDTDetachableReferenceFrame(scene::findFrameByName,
                                                   definition.getCRDTParentFrameName(),
                                                   definition.getFootToParentTransform());
   }

   @Override
   public void update()
   {
      footFrame.update();
   }

   public void toMessage(LegActionStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(LegActionStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }

   public CRDTDetachableReferenceFrame getFootFrame()
   {
      return footFrame;
   }
}
