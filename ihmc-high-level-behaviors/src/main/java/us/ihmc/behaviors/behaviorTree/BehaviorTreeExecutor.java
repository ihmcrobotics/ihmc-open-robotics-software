package us.ihmc.behaviors.behaviorTree;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.avatar.scs2.SCS2AvatarSimulationFactory;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeExtensionSubtreeRebuilder;
import us.ihmc.behaviors.behaviorTree.ros2.ROS2BehaviorTreeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.perception.detections.DetectionManager;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class BehaviorTreeExecutor
{
   private final CRDTInfo crdtInfo = new CRDTInfo(ROS2ActorDesignation.ROBOT, (int) ROS2BehaviorTreeState.SYNC_FREQUENCY);
   private final BehaviorTreeExecutorNodeBuilder nodeBuilder;
   private final BehaviorTreeExtensionSubtreeRebuilder treeRebuilder;
   private final BehaviorTreeState behaviorTreeState;
   private BehaviorTreeNodeExecutor<?, ?> rootNode;
   private final SCS2AvatarSimulation previewSimulation;

   public BehaviorTreeExecutor(DRCRobotModel robotModel,
                               ROS2SyncedRobotModel syncedRobot,
                               ReferenceFrameLibrary referenceFrameLibrary,
                               SceneGraph sceneGraph,
                               DetectionManager detectionManager,
                               ROS2ControllerHelper ros2ControllerHelper)
   {
      nodeBuilder = new BehaviorTreeExecutorNodeBuilder(robotModel, ros2ControllerHelper, syncedRobot, referenceFrameLibrary, sceneGraph, detectionManager);
      treeRebuilder = new BehaviorTreeExtensionSubtreeRebuilder(this::getRootNode, crdtInfo);

      behaviorTreeState = new BehaviorTreeState(nodeBuilder, treeRebuilder, this::getRootNode, crdtInfo, null);

      // TODO: Need to build a way to parameterize the Controller API topics to have a set for previewing
      //   Need to parameterize all behavior topics with preview vs. real robot
      SCS2AvatarSimulationFactory previewSimulationFactory = new SCS2AvatarSimulationFactory();
      previewSimulationFactory.setRobotModel(robotModel);
      // TODO: Probably need to change this to accept ROS2NodeInterface
      //   previewSimulationFactory.setRealtimeROS2Node();
      previewSimulationFactory.setDefaultHighLevelHumanoidControllerFactory();
      // TODO: Need a way of setting up robot in current configuration before you preview
      //   avatarSimulationFactory.setRobotInitialSetup(robotInitialSetup);
      previewSimulationFactory.setKinematicsSimulation(true);
      previewSimulationFactory.setUsePerfectSensors(true);
      previewSimulationFactory.setSimulationDT(1.0 / 1500.0);
      previewSimulationFactory.setSimulationDataRecordTickPeriod(20);
      previewSimulationFactory.setSimulationDataBufferDuration(5.0);
      previewSimulation = previewSimulationFactory.createAvatarSimulation();
      previewSimulation.setSystemExitOnDestroy(false);
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
