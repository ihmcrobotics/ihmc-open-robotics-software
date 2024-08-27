package us.ihmc.behaviors.ai2r;

import behavior_msgs.msg.dds.AI2RObjectMessage;
import behavior_msgs.msg.dds.AI2RStatusMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.resources.ResourceTools;
import us.ihmc.tools.thread.Throttler;

/**
 * For interfacing with external foundation models.
 */
public class AI2RNodeExecutor extends BehaviorTreeNodeExecutor<AI2RNodeState, AI2RNodeDefinition>
{
   private final ROS2ControllerHelper ros2;
   private final ROS2SyncedRobotModel syncedRobot;
   private final SceneGraph sceneGraph;
   private final Throttler statusThrottler = new Throttler().setFrequency(1.0);
   private final AI2RStatusMessage statusMessage = new AI2RStatusMessage();

   public AI2RNodeExecutor(long id,
                           CRDTInfo crdtInfo,
                           WorkspaceResourceDirectory saveFileDirectory,
                           ROS2ControllerHelper ros2,
                           ROS2SyncedRobotModel syncedRobot,
                           SceneGraph sceneGraph)
   {
      super(new AI2RNodeState(id, crdtInfo, saveFileDirectory));

      this.ros2 = ros2;
      this.syncedRobot = syncedRobot;
      this.sceneGraph = sceneGraph;

      for (String behaviorTreeFileName : ResourceTools.listResources("behaviorTrees", ".*"))
      {
         statusMessage.getAvailableBehaviors().add(behaviorTreeFileName);
      }

      ros2.subscribeViaVolatileCallback(AutonomyAPI.AI2R_COMMAND, message ->
      {
         LogTools.info("Received command message: %s".formatted(message));
      });
   }

   @Override
   public void update()
   {
      super.update();

      if (statusThrottler.run())
      {
         statusMessage.getRobotMidFeetUnderPelvisPoseInWorld().set(syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetUnderPelvisFrame));

         statusMessage.getObjects().clear();
         for (String nodeName : sceneGraph.getNodeNameList())
         {
            if (nodeName.contains("Door"))
            {
               SceneNode sceneNode = sceneGraph.getNamesToNodesMap().get(nodeName);

               AI2RObjectMessage objectMessage = statusMessage.getObjects().add();
               objectMessage.setObjectName(nodeName);
               objectMessage.getObjectPoseInWorld().set(sceneNode.getNodeFrame().getTransformToWorldFrame());
            }
         }

         ros2.publish(AutonomyAPI.AI2R_STATUS, statusMessage);
      }
   }
}
