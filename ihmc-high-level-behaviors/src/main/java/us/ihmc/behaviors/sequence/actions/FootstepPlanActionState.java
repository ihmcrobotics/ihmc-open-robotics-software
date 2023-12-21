package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage;
import behavior_msgs.msg.dds.FootstepPlanActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.robotics.lists.RecyclingArrayListTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class FootstepPlanActionState extends ActionNodeState<FootstepPlanActionDefinition>
{
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private int numberOfAllocatedFootsteps = 0;
   private final RecyclingArrayList<FootstepPlanActionFootstepState> footsteps;
   private int totalNumberOfFootsteps;
   private int numberOfIncompleteFootsteps;

   public FootstepPlanActionState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, new FootstepPlanActionDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      this.referenceFrameLibrary = referenceFrameLibrary;

      footsteps = new RecyclingArrayList<>(() ->
         new FootstepPlanActionFootstepState(referenceFrameLibrary,
                                             this,
                                             RecyclingArrayListTools.getUnsafe(getDefinition().getFootsteps().getValueUnsafe(), numberOfAllocatedFootsteps++)));
   }

   @Override
   public void update()
   {
      RecyclingArrayListTools.synchronizeSize(footsteps, getDefinition().getFootsteps().getSize());

      for (int i = 0; i < footsteps.size(); i++)
      {
         footsteps.get(i).setIndex(i);
         footsteps.get(i).update();
      }
   }

   public void toMessage(FootstepPlanActionStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());

      message.setTotalNumberOfFootsteps(totalNumberOfFootsteps);
      message.setNumberOfIncompleteFootsteps(numberOfIncompleteFootsteps);

      message.getFootsteps().clear();
      for (FootstepPlanActionFootstepState footstep : footsteps)
      {
         footstep.toMessage(message.getFootsteps().add());
      }
   }

   public void fromMessage(FootstepPlanActionStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());

      totalNumberOfFootsteps = message.getTotalNumberOfFootsteps();
      numberOfIncompleteFootsteps = message.getNumberOfIncompleteFootsteps();

      footsteps.clear();
      for (FootstepPlanActionFootstepStateMessage footstep : message.getFootsteps())
      {
         footsteps.add().fromMessage(footstep);
      }
   }

   public RecyclingArrayList<FootstepPlanActionFootstepState> getFootsteps()
   {
      return footsteps;
   }

   public int getTotalNumberOfFootsteps()
   {
      return totalNumberOfFootsteps;
   }

   public void setTotalNumberOfFootsteps(int totalNumberOfFootsteps)
   {
      this.totalNumberOfFootsteps = totalNumberOfFootsteps;
   }

   public int getNumberOfIncompleteFootsteps()
   {
      return numberOfIncompleteFootsteps;
   }

   public void setNumberOfIncompleteFootsteps(int numberOfIncompleteFootsteps)
   {
      this.numberOfIncompleteFootsteps = numberOfIncompleteFootsteps;
   }
}
