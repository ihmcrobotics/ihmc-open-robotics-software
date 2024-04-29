package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.WholeBodyCoordinatorStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalDouble;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

/**
 * A node for managing a preview of whole body motion across layered tasks including
 * walking and other controller states.
 *
 * This node has no definition.
 *
 * TODO: If this node doesn't end up doing anything else, let's just call it
 *   "whole body preview".
 */
public class WholeBodyCoordinationState extends BehaviorTreeNodeState<BehaviorTreeNodeDefinition>
{
   private final CRDTUnidirectionalDouble previewRequestedTime;

   public WholeBodyCoordinationState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new BehaviorTreeNodeDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      previewRequestedTime = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 1.0);
   }

   @Override
   public void update()
   {
      super.update();


   }

   public void toMessage(WholeBodyCoordinatorStateMessage message)
   {
      super.toMessage(message.getState());


      message.setPreviewRequestedTime(previewRequestedTime.toMessage());

   }

   public void fromMessage(WholeBodyCoordinatorStateMessage message)
   {
      super.fromMessage(message.getState());

      previewRequestedTime.fromMessage(message.getPreviewRequestedTime());
   }

   public CRDTUnidirectionalDouble getPreviewRequestedTime()
   {
      return previewRequestedTime;
   }
}
