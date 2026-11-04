package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.RLWalkActionStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;
import us.ihmc.communication.crdt.CRDTDetachableReferenceFrame;

public class RLWalkActionState extends ActionNodeState<RLWalkActionDefinition>
{
   private final CRDTDetachableReferenceFrame goalFrame;

   public RLWalkActionState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new RLWalkActionDefinition(rootNode.getDefinition()), rootNode);

      goalFrame = new CRDTDetachableReferenceFrame(scene::findFrameByName, definition.getCRDTParentFrameName(), definition.getGoalToParentTransform());
   }

   @Override
   public void update()
   {
      goalFrame.update();
   }

   public void toMessage(RLWalkActionStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(RLWalkActionStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }

   public CRDTDetachableReferenceFrame getGoalFrame()
   {
      return goalFrame;
   }
}
