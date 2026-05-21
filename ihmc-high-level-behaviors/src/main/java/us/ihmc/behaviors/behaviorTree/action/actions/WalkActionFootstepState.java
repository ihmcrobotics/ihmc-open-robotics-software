package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.WalkActionFootstepStateMessage;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneState;
import us.ihmc.communication.crdt.CRDTDetachableReferenceFrame;
import us.ihmc.communication.crdt.CRDTBidirectionalString;

public class WalkActionFootstepState
{
   private final WalkActionFootstepDefinition definition;
   private final CRDTDetachableReferenceFrame soleFrame;
   /** The index is not CRDT synced because it's a simple local calculation. */
   private int index = -1;

   public WalkActionFootstepState(BehaviorTreeSceneState scene,
                                          CRDTBidirectionalString parentFrameName,
                                          WalkActionFootstepDefinition definition)
   {
      this.definition = definition;

      soleFrame = new CRDTDetachableReferenceFrame(scene::findFrameByName, parentFrameName, definition.getSoleToPlanFrameTransform());
   }

   public void update()
   {
      soleFrame.update();
   }

   public void toMessage(WalkActionFootstepStateMessage message)
   {

   }

   public void fromMessage(WalkActionFootstepStateMessage message)
   {

   }

   public WalkActionFootstepDefinition getDefinition()
   {
      return definition;
   }

   public CRDTDetachableReferenceFrame getSoleFrame()
   {
      return soleFrame;
   }

   public int getIndex()
   {
      return index;
   }

   public void setIndex(int index)
   {
      this.index = index;
   }
}
