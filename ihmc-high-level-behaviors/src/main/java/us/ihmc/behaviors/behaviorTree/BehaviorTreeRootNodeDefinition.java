package us.ihmc.behaviors.behaviorTree;

import behavior_msgs.msg.dds.BehaviorTreeRootNodeDefinitionMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class BehaviorTreeRootNodeDefinition extends BehaviorTreeNodeDefinition
{
   public BehaviorTreeRootNodeDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, DRCRobotModel robotModel)
   {
      super(null, crdtInfo, saveFileDirectory, robotModel);
   }

   @Override
   public boolean hasChanges()
   {
      return false; // Root node cannot be saved
   }

   public void toMessage(BehaviorTreeRootNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());
   }

   public void fromMessage(BehaviorTreeRootNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());
   }

   // Getters are in here so there's not getters in base node for root stuff

   public WorkspaceResourceDirectory getSaveFileDirectory()
   {
      return saveFileDirectory;
   }

   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }
}
