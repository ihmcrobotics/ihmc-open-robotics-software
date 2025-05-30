package us.ihmc.behaviors.ai2r;

import behavior_msgs.msg.dds.AI2RActionFailureMessage;
import behavior_msgs.msg.dds.AI2RObjectMessage;
import behavior_msgs.msg.dds.AI2RStatusMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.logic.ConditionNodeDefinition;
import us.ihmc.behaviors.logic.ConditionNodeState;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.behaviors.sequence.LeafNodeState;
import us.ihmc.behaviors.sequence.actions.ChestOrientationActionState;
import us.ihmc.behaviors.sequence.actions.FootstepPlanActionState;
import us.ihmc.behaviors.sequence.actions.HandPoseActionState;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
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
   private final Throttler statusThrottler = new Throttler().setFrequency(30.0);
   private final AI2RStatusMessage statusMessage = new AI2RStatusMessage();
   private final List<LeafNodeState<?>> failedLeaves = new ArrayList<>();

   private boolean actionFailureMissingFrame = false;
   private final AI2RSkillEditor skillEditor = new AI2RSkillEditor();


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
      resetStatusMessage();

      ros2.subscribeViaCallback(AutonomyAPI.AI2R_COMMAND, message ->
      {
         LogTools.info("Received command message: %s".formatted(message));

         // Prepare commanded behavior
         String behaviorToExecuteName = message.getBehaviorToExecuteAsString();
         int commandedBehaviorIndex = -1;
         for (int i = 0; i < state.getCheckPoints().size(); i++)
         {
            if (state.getCheckPoints().get(i).getDefinition().getName().equals(behaviorToExecuteName))
            {
               commandedBehaviorIndex = state.getCheckPoints().get(i).getLeafIndex();
               break;
            }
         }

         // Generic adaptable skills
         skillEditor.adaptSkills(behaviorToExecuteName, state, message, commandedBehaviorIndex);

         // Trigger commanded behavior
         if (commandedBehaviorIndex >= 0)
         {
            // Reset state of failed leaves
            for (int j = 0; j < failedLeaves.size(); j++)
            {
               failedLeaves.get(j).setFailed(false);
            }
            failedLeaves.clear();
            actionFailureMissingFrame = false;
            state.getActionSequence().setExecutionNextIndex(commandedBehaviorIndex);
            state.getActionSequence().setAutomaticExecution(true);

            resetStatusMessage();
            LogTools.warn("Automatic execution");
            statusMessage.setBehaviorInProgress(behaviorToExecuteName);
         }
      });
   }

   private void resetStatusMessage()
   {
      statusMessage.setBehaviorInProgress("-");
      statusMessage.setCompletedBehavior("-");
      statusMessage.setFailedBehavior("-");
      statusMessage.getFailure().setActionName("-");
      statusMessage.getFailure().setActionType("-");
      statusMessage.getFailure().setMissingFrame(false);
      statusMessage.getFailure().getPositionError().set(new Point3D());
      statusMessage.getFailure().getOrientationError().set(new Quaternion());
   }

   @Override
   public void update()
   {
      super.update();

      if (statusThrottler.run())
      {
         statusMessage.getRobotMidFeetUnderPelvisPoseInWorld().set(syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetUnderPelvisFrame));

         setSceneInfo();
         setAvailableBehaviors();
         setFailedBehaviors();
         ros2.publish(AutonomyAPI.AI2R_STATUS, statusMessage);
      }

      endSequenceAfterBehaviorExecution();
      reportMissingFrameFailures();
   }

   private void setSceneInfo()
   {
      statusMessage.getObjects().clear();
      boolean isRoot = true;
      for (String nodeName : sceneGraph.getNodeNameList())
      {
         if (isRoot)
         {
            isRoot = false;
            continue;
         }
         AI2RObjectMessage objectMessage = statusMessage.getObjects().add();
         objectMessage.setObjectName(nodeName);
         ReferenceFrame nodeFrame = sceneGraph.getNamesToNodesMap().get(nodeName).getNodeFrame();
         objectMessage.getObjectPoseInWorld().set(nodeFrame.getTransformToWorldFrame());
         objectMessage.getObjectPoseInRobotFrame().set(nodeFrame.getTransformToDesiredFrame(syncedRobot.getReferenceFrames().getMidFeetUnderPelvisFrame()));
      }
   }

   private void setAvailableBehaviors()
   {
      statusMessage.getAvailableBehaviors().resetQuick();
      for (int i = 0; i < state.getCheckPoints().size(); i++)
      {
         String checkPointName = state.getCheckPoints().get(i).getDefinition().getName();
         if (!checkPointName.contains("END"))
            statusMessage.getAvailableBehaviors().add(checkPointName);
      }
   }

   private void setFailedBehaviors()
   {
      statusMessage.setFailedBehavior("-");
      for (var leaf : state.getActionSequence().getOrderedLeaves())
      {
         if (leaf.getFailed() && !state.getActionSequence().getAutomaticExecution())
         {
            // Find the previous checkpoint by iterating backwards through the checkpoints
            for (int i = state.getCheckPoints().size() - 1; i >= 0; i--)
            {
               var checkpoint = state.getCheckPoints().get(i);

               // Check if the checkpoint is before the failed leaf
               if (checkpoint.getLeafIndex() < leaf.getLeafIndex())
               {
                  // Retrieve the name of the closest previous checkpoint
                  String checkpointName = checkpoint.getDefinition().getName();

                  LogTools.info("Leaf failed at index: {}, closest previous checkpoint: {}", leaf.getLeafIndex(), checkpointName);

                  statusMessage.setFailedBehavior(checkpointName);
                  if (leaf instanceof ActionNodeState<?> action)
                  {
                     AI2RActionFailureMessage failureMessage = statusMessage.getFailure();
                     failureMessage.setActionName(action.getDefinition().getName());
                     if (action instanceof FootstepPlanActionState walkAction)
                     {
                        failureMessage.setActionFrame(walkAction.getDefinition().getParentFrameName());
                        failureMessage.setMissingFrame(actionFailureMissingFrame);
                        failureMessage.setActionType(walkAction.getDefinition().getClass().getSimpleName());
                     }
                     else if (action instanceof HandPoseActionState handPoseAction)
                     {
                        failureMessage.setOrientationTolerance(action.getOrientationDistanceToGoalTolerance());
                        failureMessage.setPositionTolerance(action.getPositionDistanceToGoalTolerance());

                        var desiredValue = handPoseAction.getCommandedTrajectory().getLastValueReadOnly();
                        var actualValue = handPoseAction.getCurrentPose().getValueReadOnly();

                        Quaternion errorOrientation = new Quaternion(actualValue.getOrientation());
                        errorOrientation.multiply(desiredValue.getOrientation());
                        failureMessage.getOrientationError().set(errorOrientation);

                        Point3D errorPosition = new Point3D(desiredValue.getPosition());
                        errorPosition.sub(actualValue.getPosition());
                        failureMessage.getPositionError().set(errorPosition);

                        failureMessage.setActionFrame(handPoseAction.getDefinition().getPalmParentFrameName());
                        failureMessage.setActionType(handPoseAction.getDefinition().getClass().getSimpleName());
                     }

                     if (action instanceof ChestOrientationActionState chestAction)
                     {
                        failureMessage.setActionFrame(chestAction.getDefinition().getParentFrameName());
                        failureMessage.setActionType(chestAction.getDefinition().getClass().getSimpleName());
                     }
                  }
                  if (leaf instanceof ConditionNodeState conditionNodeState)
                  {
                     AI2RActionFailureMessage failureMessage = statusMessage.getFailure();
                     failureMessage.setActionName(leaf.getDefinition().getName());
                     if (conditionNodeState.getDefinition().getType().getValue() == ConditionNodeDefinition.Type.PROXIMITY)
                     {
                        failureMessage.setMissingFrame(actionFailureMissingFrame);
                        double maxDistanceAllowed = conditionNodeState.getDefinition().getProximityCheck().getMaxDistanceToObject();
                        double currentDistance = conditionNodeState.getProximityCheck().getCurrentDistance().getValue();
                        double error = currentDistance - maxDistanceAllowed;
                        failureMessage.getPositionError().set(error, 0.0, 0.0);
                        failureMessage.setPositionTolerance(0.0);
                     }
                     failureMessage.setActionType(conditionNodeState.getDefinition().getClass().getSimpleName());
                  }
                  failedLeaves.add(leaf);
                  break;
               }
            }
         }
      }
   }

   private void reportMissingFrameFailures()
   {
      // Check if Goto action is executing and if next steps are colliding with objects in the scene
      goToCollisionLoop:
      for (var leaf : state.getActionSequence().getOrderedLeaves())
      {
         if (leaf.getIsNextForExecution())
         {
            if (!leaf.getCanExecute())
            {
               actionFailureMissingFrame = true;
            }
         }
      }
   }

   private void endSequenceAfterBehaviorExecution()
   {
      // Jump to end of sequence, once completed a behavior
      for (int i = 0; i < state.getCheckPoints().size(); i++)
      {
         // If we execute the end of behavior checkpoint, we communicate that in the status
         if (state.getCheckPoints().get(i).getDefinition().getName().contains("END OF") && state.getCheckPoints().get(i).getIsExecuting())
         {
            // ! WARNING !
            // Assuming checkpoints are only used at the beginning and end of a behaviors
            statusMessage.setCompletedBehavior(state.getCheckPoints().get(i - 1).getDefinition().getName());
            statusMessage.setBehaviorInProgress("-");
            LogTools.info("Completed behavior: {}", statusMessage.getCompletedBehavior());
            // Jump to end of sequence
            state.getActionSequence().setExecutionNextIndex(state.getCheckPoints().get(state.getCheckPoints().size() - 1).getLeafIndex());
         }
      }
   }
}
