package us.ihmc.behaviors.behaviorTree.trashCan;

import behavior_msgs.msg.dds.DoorTraversalDefinitionMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalString;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class TrashCanInteractionDefinition extends BehaviorTreeNodeDefinition
{
   private final CRDTUnidirectionalString obstructedNode;

   public TrashCanInteractionDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      obstructedNode = new CRDTUnidirectionalString(ROS2ActorDesignation.OPERATOR, crdtInfo, "");
   }

   public void toMessage(DoorTraversalDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setObstructedNode(obstructedNode.toMessage());
   }

   public void fromMessage(DoorTraversalDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      obstructedNode.fromMessage(message.getObstructedNode());
   }

   public CRDTUnidirectionalString getObstructedNode()
   {
      return obstructedNode;
   }
}
