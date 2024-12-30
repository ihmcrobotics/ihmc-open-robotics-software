package us.ihmc.behaviors.ai2r;

import behavior_msgs.msg.dds.AI2RNodeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeTools;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.behaviors.sequence.ActionSequenceState;
import us.ihmc.behaviors.sequence.actions.CheckPointNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.ArrayList;
import java.util.List;

public class AI2RNodeState extends BehaviorTreeNodeState<AI2RNodeDefinition>
{
   private BehaviorTreeRootNodeState actionSequence;
   private List<CheckPointNodeState> checkPoints = new ArrayList<>();

   public AI2RNodeState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new AI2RNodeDefinition(crdtInfo, saveFileDirectory), crdtInfo);
   }

   @Override
   public void update()
   {
      super.update();

      actionSequence = BehaviorTreeTools.findRootNode(this);
      checkPoints.clear();
      updateActionSubtree(this);
   }

   public void updateActionSubtree(BehaviorTreeNodeState<?> node)
   {
      for (BehaviorTreeNodeState<?> child : node.getChildren())
      {
         if (child instanceof ActionNodeState<?> actionNode)
         {
            if (actionNode instanceof CheckPointNodeState checkPoint)
            {
               checkPoints.add(checkPoint);
            }
         }
         if (child instanceof ActionSequenceState sequence)
            updateActionSubtree(child);
      }
   }

   public List<CheckPointNodeState> getCheckPoints()
   {
      return checkPoints;
   }

   public BehaviorTreeRootNodeState getActionSequence()
   {
      return actionSequence;
   }

   public void toMessage(AI2RNodeStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(AI2RNodeStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());
   }
}
