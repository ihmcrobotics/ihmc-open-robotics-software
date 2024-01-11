package us.ihmc.behaviors.behaviorTree;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeExtensionSubtreeRebuilder;
import us.ihmc.behaviors.behaviorTree.ros2.ROS2BehaviorTreeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class BehaviorTreeExecutor
{
   private final CRDTInfo crdtInfo = new CRDTInfo(ROS2ActorDesignation.ROBOT, (int) ROS2BehaviorTreeState.SYNC_FREQUENCY);
   private final BehaviorTreeExecutorNodeBuilder nodeBuilder;
   private final BehaviorTreeExtensionSubtreeRebuilder treeRebuilder;
   private final BehaviorTreeState behaviorTreeState;
   private BehaviorTreeNodeExecutor<?, ?> rootNode;

   public BehaviorTreeExecutor(DRCRobotModel robotModel,
                               ROS2SyncedRobotModel syncedRobot,
                               ReferenceFrameLibrary referenceFrameLibrary,
                               ROS2ControllerHelper ros2ControllerHelper)
   {
      nodeBuilder = new BehaviorTreeExecutorNodeBuilder(robotModel, ros2ControllerHelper, syncedRobot, referenceFrameLibrary);
      treeRebuilder = new BehaviorTreeExtensionSubtreeRebuilder(this::getRootNode, crdtInfo);

      behaviorTreeState = new BehaviorTreeState(nodeBuilder, treeRebuilder, this::getRootNode, crdtInfo, null);
   }

   public void update()
   {
      if (rootNode != null)
      {
         rootNode.clock();

         rootNode.tick();

         update(rootNode);
      }
   }

   private void update(BehaviorTreeNodeExecutor<?, ?> node)
   {
      node.update();

      for (BehaviorTreeNodeExecutor<?, ?> child : node.getChildren())
      {
         update(child);
      }
   }

   public void destroy()
   {
      if (rootNode != null)
      {
         behaviorTreeState.modifyTreeTopology(topologyOperationQueue -> topologyOperationQueue.queueDestroySubtree(rootNode));
      }
   }

   public void setRootNode(BehaviorTreeNodeLayer<?, ?, ?, ?> rootNode)
   {
      this.rootNode = (BehaviorTreeNodeExecutor<?, ?>) rootNode;
   }

   public BehaviorTreeNodeExecutor<?, ?> getRootNode()
   {
      return rootNode;
   }

   public BehaviorTreeState getState()
   {
      return behaviorTreeState;
   }

   public CRDTInfo getCrdtInfo()
   {
      return crdtInfo;
   }
}
