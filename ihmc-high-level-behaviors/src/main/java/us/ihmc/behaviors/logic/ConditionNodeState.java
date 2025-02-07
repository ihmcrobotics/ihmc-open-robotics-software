package us.ihmc.behaviors.logic;

import behavior_msgs.msg.dds.ConditionNodeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTStatusLong;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class ConditionNodeState extends BehaviorTreeNodeState<ConditionNodeDefinition>
{
   private final CRDTStatusLong count;

   public ConditionNodeState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new ConditionNodeDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      count = new CRDTStatusLong(ROS2ActorDesignation.ROBOT, crdtInfo, 0);
   }

   @Override
   public void update()
   {
      super.update();
   }

   @Override
   public boolean hasStatus()
   {
      boolean hasStatus = false;
      hasStatus |= count.pollHasStatus();
      return hasStatus;
   }

   public void toMessage(ConditionNodeStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(ConditionNodeStateMessage message)
   {
      getDefinition().fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }
}
