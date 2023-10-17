package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.BehaviorTreeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeState;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.ros2.ROS2IOTopicQualifier;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;

public class ROS2BehaviorTreePublisher
{
   private ROS2IOTopicQualifier ioQualifier;
   private final BehaviorTreeStateMessage behaviorTreeStateMessage = new BehaviorTreeStateMessage();

   public void publish(BehaviorTreeState behaviorTreeState, ROS2PublishSubscribeAPI ros2PublishSubscribeAPI, ROS2IOTopicQualifier outgoingQualifier)
   {
      this.ioQualifier = ioQualifier;

      behaviorTreeStateMessage.setNextId(behaviorTreeState.getNextID().intValue());
      behaviorTreeStateMessage.getBehaviorTreeTypes().clear();
      behaviorTreeStateMessage.getBehaviorTreeIndices().clear();
      behaviorTreeStateMessage.getArmJointAnglesActions().clear();
      behaviorTreeStateMessage.getChestOrientationActions().clear();
      behaviorTreeStateMessage.getFootstepPlanActions().clear();
      behaviorTreeStateMessage.getHandPoseActions().clear();
      behaviorTreeStateMessage.getHandWrenchActions().clear();
      behaviorTreeStateMessage.getPelvisHeightActions().clear();
      behaviorTreeStateMessage.getSakeHandCommandActions().clear();
      behaviorTreeStateMessage.getWaitDurationActions().clear();
      behaviorTreeStateMessage.getWalkActions().clear();

      packSceneTreeToMessage(behaviorTreeState.getRootNode(), behaviorTreeStateMessage);

      ros2PublishSubscribeAPI.publish(AutonomyAPI.BEAVIOR_TREE.getTopic(ioQualifier), behaviorTreeStateMessage);
   }

   private void packSceneTreeToMessage(BehaviorTreeRootNode rootNode, BehaviorTreeStateMessage behaviorTreeStateMessage)
   {

   }
}
