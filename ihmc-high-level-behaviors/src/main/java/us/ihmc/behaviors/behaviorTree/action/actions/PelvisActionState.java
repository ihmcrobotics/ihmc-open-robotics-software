package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.PelvisActionStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;
import us.ihmc.communication.crdt.CRDTDetachableReferenceFrame;

public class PelvisActionState extends ActionNodeState<PelvisActionDefinition>
{
   private final CRDTDetachableReferenceFrame pelvisFrame;

   public PelvisActionState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new PelvisActionDefinition(rootNode.getDefinition()), rootNode);

      pelvisFrame = new CRDTDetachableReferenceFrame(scene::findFrameByName,
                                                     definition.getCRDTParentFrameName(),
                                                     definition.getPelvisToParentTransform());
   }

   @Override
   public void update()
   {
      pelvisFrame.update();
   }

   public void toMessage(PelvisActionStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(PelvisActionStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }

   public CRDTDetachableReferenceFrame getPelvisFrame()
   {
      return pelvisFrame;
   }
}
