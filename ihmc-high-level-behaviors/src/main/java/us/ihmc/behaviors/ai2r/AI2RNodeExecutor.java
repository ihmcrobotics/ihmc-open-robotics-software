package us.ihmc.behaviors.ai2r;

import behavior_msgs.msg.dds.AI2RObjectMessage;
import behavior_msgs.msg.dds.AI2RStatusMessage;
import controller_msgs.msg.dds.AbortWalkingMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.sequence.LeafNodeState;
import us.ihmc.behaviors.sequence.actions.FootstepPlanActionState;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTStatusFootstepList;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.commons.thread.Throttler;

import java.util.ArrayList;
import java.util.List;

/**
 * Node that enables interaction with external reasoning modules
 */
public class AI2RNodeExecutor extends BehaviorTreeNodeExecutor<AI2RNodeState, AI2RNodeDefinition>
{
   private final ROS2ControllerHelper ros2;
   private final ROS2SyncedRobotModel syncedRobot;
   private final SceneGraph sceneGraph;
   private final Throttler statusThrottler = new Throttler().setFrequency(1.0);
   private final AI2RStatusMessage statusMessage = new AI2RStatusMessage();
   private final List<LeafNodeState<?>> failedLeaves = new ArrayList<>();
   private CRDTStatusFootstepList plannedSteps;
   private static final double DISTANCE_COLLISION_THRESHOLD = 0.3;

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

      ros2.subscribeViaCallback(AutonomyAPI.AI2R_COMMAND, message ->
      {
         LogTools.info("Received command message: %s".formatted(message));

         // Set goals for GoTo behavior
         String referenceFrame = message.getGotoReferenceFrameName().toString();
         Point3D goalStancePoint = message.getGotoGoalStancePoint();
         Point3D goalFocalPoint = message.getGotoGoalFocalPoint();
         for (var leaf : state.getActionSequence().getOrderedLeaves())
         {
            if (leaf.getDefinition().getName().contains("Go to Action") && leaf instanceof FootstepPlanActionState gotoActionState)
            {
               gotoActionState.getDefinition().setParentFrameName(referenceFrame);
               gotoActionState.getDefinition().getGoalStancePoint().getValue().set(goalStancePoint);
               gotoActionState.getDefinition().getGoalFocalPoint().getValue().set(goalFocalPoint);
               break;
            }
         }

         // Trigger specified behavior
         String checkPointName = message.getBehaviorToExecuteAsString();
         for (int i=0; i < state.getCheckPoints().size(); i++)
         {
            if (state.getCheckPoints().get(i).getDefinition().getName().equals(checkPointName))
            {
               for (int j = 0; j < failedLeaves.size(); j++)
               {
                  failedLeaves.get(j).setFailed(false);
               }
               failedLeaves.clear();
               state.getActionSequence().setExecutionNextIndex(state.getCheckPoints().get(i).getLeafIndex());
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
         for (var leaf : state.getActionSequence().getOrderedLeaves())
         {
            if (leaf.getFailed() && !state.getActionSequence().getAutomaticExecution())
            {
               // Find the previous checkpoint by iterating backwards through the checkpoints
               for (int i = state.getCheckPoints().size() - 1; i >= 0; i--) {
                  var checkpoint = state.getCheckPoints().get(i);

                  // Check if the checkpoint is before the failed leaf
                  if (checkpoint.getLeafIndex() < leaf.getLeafIndex())
                  {
                     // Retrieve the name of the closest previous checkpoint
                     String checkpointName = checkpoint.getDefinition().getName();

                     LogTools.info("Leaf failed at index: {}, closest previous checkpoint: {}",
                                   leaf.getLeafIndex(), checkpointName);

                     statusMessage.setFailedBehavior(checkpointName);
                     failedLeaves.add(leaf);
                     break;
                  }
               }
            }
         }
         ros2.publish(AutonomyAPI.AI2R_STATUS, statusMessage);
      }

      // Jump to end of sequence, once completed a behavior
      for (int i = 0; i < state.getCheckPoints().size(); i++)
      {
         // If we execute the end of behavior checkpoint, we communicate that in the status
         if (state.getCheckPoints().get(i).getDefinition().getName().contains("END OF") && state.getCheckPoints().get(i).getIsExecuting())
         {
            // ! WARNING !
            // Assuming checkpoints are only used at the beginning and end of a behaviors
            statusMessage.setCompletedBehavior(state.getCheckPoints().get(i - 1).getDefinition().getName());
            LogTools.info("Completed behavior: {}", statusMessage.getCompletedBehavior());
            // Jump to end of sequence
            state.getActionSequence().setExecutionNextIndex(state.getCheckPoints().get(state.getCheckPoints().size()-1).getLeafIndex());
         }
         else if (!state.getCheckPoints().get(i).getDefinition().getName().contains("END") && state.getCheckPoints().get(i).getIsExecuting())
         { // If we are executing another behavior checkpoint
            statusMessage.setCompletedBehavior("");
         }
      }

      // Check if Goto action is executing and if next steps are colliding with objects in the scene
      goToCollisionLoop:
      for (var leaf : state.getActionSequence().getOrderedLeaves())
      {
         if (leaf.getDefinition().getName().contains("Go to Action") && leaf instanceof FootstepPlanActionState gotoActionState)
         {
            if (gotoActionState.getIsExecuting())
            {
               if (plannedSteps == null)
               {
                  plannedSteps = gotoActionState.getPreviewFootsteps();
               }
               else // Check if the next step's pose is too close with any object in the scene
               {
                  int stepsLeft = gotoActionState.getNumberOfIncompleteFootsteps();
                  if (stepsLeft > 0)
                  {
                     Point3DReadOnly positionNextStep = plannedSteps.getPoseReadOnly(plannedSteps.getSize() - stepsLeft).getTranslation();
                     for (var object : statusMessage.getObjects())
                     {
                        if (!object.getObjectNameAsString().contains("SceneGraphRoot"))
                        {
                           Point3DReadOnly objectPosition = object.getObjectPoseInWorld().getTranslation();
                           if(positionNextStep.distanceXY(objectPosition) < DISTANCE_COLLISION_THRESHOLD)
                           {
                              gotoActionState.setFailed(true);
                              // Have the executor abort
                              ros2.publishToController(new AbortWalkingMessage());

                              plannedSteps = null;
                              break goToCollisionLoop;
                           }
                        }
                     }
                  }
               }
            }
            break;
         }
      }
   }
}
