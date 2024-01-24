package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage;
import us.ihmc.communication.crdt.CRDTDetachableReferenceFrame;
import us.ihmc.communication.crdt.CRDTUnidirectionalString;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class FootstepPlanActionFootstepState
{
   private final FootstepPlanActionFootstepDefinition definition;
   private final CRDTDetachableReferenceFrame soleFrame;
   /** The index is not CRDT synced because it's a simple local calculation. */
   private int index = -1;

   public FootstepPlanActionFootstepState(ReferenceFrameLibrary referenceFrameLibrary,
                                          CRDTUnidirectionalString parentFrameName,
                                          FootstepPlanActionFootstepDefinition definition)
   {
      this.definition = definition;

      soleFrame = new CRDTDetachableReferenceFrame(referenceFrameLibrary, parentFrameName, definition.getSoleToPlanFrameTransform());
   }

   public void update()
   {
      soleFrame.update();
   }

   public void toMessage(FootstepPlanActionFootstepStateMessage message)
   {

   }

   public void fromMessage(FootstepPlanActionFootstepStateMessage message)
   {

   }

   public FootstepPlanActionFootstepDefinition getDefinition()
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
