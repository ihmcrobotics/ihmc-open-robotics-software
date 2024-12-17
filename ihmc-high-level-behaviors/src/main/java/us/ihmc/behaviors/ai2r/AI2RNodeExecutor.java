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
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.resources.ResourceTools;
import us.ihmc.commons.thread.Throttler;

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
   private final AI2RNodeState state;
   private boolean triedPush = false;

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
      state = getState();

      for (String behaviorTreeFileName : ResourceTools.listResources("behaviorTrees", ".*"))
      {
         statusMessage.getAvailableBehaviors().add(behaviorTreeFileName);
      }

      ros2.subscribeViaCallback(AutonomyAPI.AI2R_COMMAND, message ->
      {
         LogTools.info("Received command message: %s".formatted(message));

         String checkPointName = message.getBehaviorToExecuteAsString();
         for (int i=0; i < state.getCheckPoints().size(); i++)
         {
            if (state.getCheckPoints().get(i).getDefinition().getName().equals(checkPointName))
            {
               state.getActionSequence().setExecutionNextIndex(state.getCheckPoints().get(i).getActionIndex());
               state.getActionSequence().setAutomaticExecution(true);
               break;
            }
         }
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
            AI2RObjectMessage objectMessage = statusMessage.getObjects().add();
            objectMessage.setObjectName(nodeName);
            objectMessage.getObjectPoseInWorld().set(sceneGraph.getNamesToNodesMap().get(nodeName).getNodeFrame().getTransformToWorldFrame());
         }

         statusMessage.getAvailableBehaviors().resetQuick();
         for (int i =0; i< state.getCheckPoints().size(); i++)
         {
            String checkPointName = state.getCheckPoints().get(i).getDefinition().getName();
            if (!checkPointName.contains("END"))
               statusMessage.getAvailableBehaviors().add(checkPointName);
         }

         statusMessage.setFailedBehavior("");
         for (var actionChild : state.getActionSequence().getActionChildren())
         {
            if (actionChild.getFailed() && !state.getActionSequence().getAutomaticExecution())
            {
               // Find the previous checkpoint action by iterating backwards through the checkpoints
               for (int i = state.getCheckPoints().size() - 1; i >= 0; i--) {
                  var checkpoint = state.getCheckPoints().get(i);

                  // Check if the checkpoint is before the failed action
                  if (checkpoint.getActionIndex() < actionChild.getActionIndex())
                  {
                     // Retrieve the name of the closest previous checkpoint
                     String checkpointActionName = checkpoint.getDefinition().getName();

                     LogTools.info("Action failed at index: {}, closest previous checkpoint: {}",
                                   actionChild.getActionIndex(), checkpointActionName);

                     statusMessage.setFailedBehavior(checkpointActionName);
                     break;
                  }
               }
            }
         }

         ros2.publish(AutonomyAPI.AI2R_STATUS, statusMessage);
      }

      // Custom logic, since we don't have conditions implemented yet
      for (var checkPoint : state.getCheckPoints())
      {
         if (checkPoint.getDefinition().getName().contains("END OF PUSH") && checkPoint.getIsExecuting())
         {
            triedPush = true;
         }
         // Jump to end of sequence
         if (checkPoint.getDefinition().getName().contains("PULL") && checkPoint.getIsExecuting() && triedPush)
         {
            state.getActionSequence().setExecutionNextIndex(state.getCheckPoints().get(state.getCheckPoints().size()-1).getActionIndex());
            triedPush = false;
         }
      }
   }
}
