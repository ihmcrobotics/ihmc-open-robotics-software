package us.ihmc.behaviors.roomExploration;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RoomExplorationExecutor extends BehaviorTreeNodeExecutor<RoomExplorationState, RoomExplorationDefinition>
{
   public RoomExplorationExecutor(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new RoomExplorationState(id, crdtInfo, saveFileDirectory));
   }

   @Override
   public void clock()
   {
      super.clock();
   }

   @Override
   public void tick()
   {
      super.tick();
   }

   @Override
   public void update()
   {
      super.update();
   }
}
