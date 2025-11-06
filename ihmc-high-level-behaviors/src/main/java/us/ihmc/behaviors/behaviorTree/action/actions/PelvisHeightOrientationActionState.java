package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.PelvisHeightOrientationActionStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;
import us.ihmc.communication.crdt.CRDTDetachableReferenceFrame;

public class PelvisHeightOrientationActionState extends ActionNodeState<PelvisHeightOrientationActionDefinition>
{
   private final CRDTDetachableReferenceFrame pelvisFrame;

   public PelvisHeightOrientationActionState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new PelvisHeightOrientationActionDefinition(rootNode.getDefinition()), rootNode);

      pelvisFrame = new CRDTDetachableReferenceFrame(scene::findFrameByName,
                                                     definition.getCRDTParentFrameName(),
                                                     definition.getPelvisToParentTransform());
   }

   @Override
   public void update()
   {
      pelvisFrame.update();
   }

   public void toMessage(PelvisHeightOrientationActionStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(PelvisHeightOrientationActionStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }

   public CRDTDetachableReferenceFrame getPelvisFrame()
   {
      return pelvisFrame;
   }
}
