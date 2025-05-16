package us.ihmc.behaviors.ai2r;

import behavior_msgs.msg.dds.AI2RActionFailureMessage;
import behavior_msgs.msg.dds.AI2RHandPoseAdaptationMessage;
import behavior_msgs.msg.dds.AI2RNavigationMessage;
import behavior_msgs.msg.dds.AI2RObjectMessage;
import behavior_msgs.msg.dds.AI2RStatusMessage;
import controller_msgs.msg.dds.AbortWalkingMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.behaviors.sequence.LeafNodeState;
import us.ihmc.behaviors.sequence.actions.ChestOrientationActionState;
import us.ihmc.behaviors.sequence.actions.FootstepPlanActionState;
import us.ihmc.behaviors.sequence.actions.HandPoseActionState;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTStatusFootstepList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
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
   private static final double DISTANCE_COLLISION_THRESHOLD = 0.6;
   private boolean navigationFailureForObstacle = false;
   private boolean actionFailureMissingFrame = false;
   private String navigationFailureObstacleName;

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
         LogTools.warn(commandedBehaviorIndex);

         // Generic adaptable skills
         // GoTo behavior - Navigation
         if (behaviorToExecuteName.contains("GOTO") && message.getAdaptingBehavior())
         {
            AI2RNavigationMessage navigationMessage = message.getNavigation();
            // Set goals for GoTo behavior
            String referenceFrame = navigationMessage.getReferenceFrameName().toString();
            double distanceToReferenceFrame = navigationMessage.getDistanceToFrame();

            for (var leaf : state.getActionSequence().getOrderedLeaves())
            {
               if (leaf.getDefinition().getName().toLowerCase().contains("go to action") && leaf instanceof FootstepPlanActionState gotoActionState)
               {
                  gotoActionState.getDefinition().setParentFrameName(referenceFrame);

                  FramePoint3D goalStancePoint = new FramePoint3D(gotoActionState.getParentFrame());
                  LogTools.info(gotoActionState.getParentFrame().getName());
                  goalStancePoint.addX(distanceToReferenceFrame);
                  goalStancePoint.changeFrame(ReferenceFrame.getWorldFrame());

                  FramePoint3D goalFocalPoint = new FramePoint3D(gotoActionState.getParentFrame());
                  goalFocalPoint.changeFrame(ReferenceFrame.getWorldFrame());

                  gotoActionState.getDefinition().getGoalStancePoint().getValue().set(goalStancePoint);
                  gotoActionState.getDefinition().getGoalFocalPoint().getValue().set(goalFocalPoint);
                  break;
               }
            }
         }
         // Object pick and place
         else if (behaviorToExecuteName.contains("PICKUP") || behaviorToExecuteName.contains("PLACE") && message.getAdaptingBehavior())
         {
            AI2RHandPoseAdaptationMessage handMessage = message.getHandPoseAdaptation();
            for (var leaf : state.getActionSequence().getOrderedLeaves())
            {
               if (leaf.getLeafIndex() > commandedBehaviorIndex &&
                   leaf.getDefinition().getName().contains(handMessage.getActionName()) &&
                   leaf instanceof HandPoseActionState handPoseActionState)
               {
                  handPoseActionState.getDefinition().setPalmParentFrameName(handMessage.getReferenceFrameNameAsString());
                  RigidBodyTransform adaptedPose = new RigidBodyTransform(handMessage.getNewOrientation(), handMessage.getNewPosition());
                  handPoseActionState.getDefinition().getPalmTransformToParent().setValue(adaptedPose ,1e-5);
                  break;
               }
            }
         }

         // Trigger commanded behavior
         if (commandedBehaviorIndex >= 0)
         {
            // Reset state of failed leaves
            for (int j = 0; j < failedLeaves.size(); j++)
            {
               failedLeaves.get(j).setFailed(false);
            }
            failedLeaves.clear();
            navigationFailureForObstacle = false;
            actionFailureMissingFrame = false;
            state.getActionSequence().setExecutionNextIndex(commandedBehaviorIndex);
            state.getActionSequence().setAutomaticExecution(true);
            statusMessage.setCompletedBehavior("-");
            statusMessage.getFailure().setActionName("-");
            statusMessage.getFailure().setCollisionName("-");
            statusMessage.getFailure().setMissingFrame(false);
            statusMessage.getFailure().getPositionError().set(new Point3D());
            statusMessage.getFailure().getOrientationError().set(new Quaternion());
            LogTools.warn("Automatic execution");
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

         setSceneInfo();
         setAvailableBehaviors();
         setFailedBehaviors();
         LogTools.info("Status: failed {}, missing frame {}, collision with {}", statusMessage.getFailure().getActionName(), actionFailureMissingFrame, navigationFailureObstacleName);
         ros2.publish(AutonomyAPI.AI2R_STATUS, statusMessage);
      }

      endSequenceAfterBehaviorExecution();
      reportNavigationFailures();
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
      statusMessage.setFailedBehavior("");
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
                        if (navigationFailureForObstacle)
                        {
                           failureMessage.setCollisionName(navigationFailureObstacleName);
                        }
                        if (actionFailureMissingFrame)
                        {
                           failureMessage.setMissingFrame(actionFailureMissingFrame);
                        }
                     }
                     else
                     {
                        failureMessage.setOrientationTolerance(action.getOrientationDistanceToGoalTolerance());
                        failureMessage.setPositionTolerance(action.getPositionDistanceToGoalTolerance());

                        var desiredValue = action.getCommandedTrajectory().getLastValueReadOnly();
                        var actualValue = action.getCurrentPose().getValueReadOnly();

                        Quaternion errorOrientation = new Quaternion(actualValue.getOrientation());
                        errorOrientation.multiply(desiredValue.getOrientation());
                        failureMessage.getOrientationError().set(errorOrientation);

                        Point3D errorPosition = new Point3D(desiredValue.getPosition());
                        errorPosition.sub(actualValue.getPosition());
                        failureMessage.getPositionError().set(errorPosition);
                     }

                     if (action instanceof HandPoseActionState handAction)
                     {
                        failureMessage.setActionFrame(handAction.getDefinition().getPalmParentFrameName());
                     }
                     if (action instanceof ChestOrientationActionState chestAction)
                     {
                        failureMessage.setActionFrame(chestAction.getDefinition().getParentFrameName());
                     }
                  }
                  failedLeaves.add(leaf);
                  break;
               }
            }
         }
      }
   }

   private void reportNavigationFailures()
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
                  if (stepsLeft > plannedSteps.getSize())
                     plannedSteps = gotoActionState.getPreviewFootsteps();
                  if (stepsLeft > 0 && plannedSteps.getSize() >= stepsLeft + 1)
                  {
                     Point3DReadOnly positionNextStep = plannedSteps.getPoseReadOnly(plannedSteps.getSize() - stepsLeft + 1).getTranslation();
                     for (var object : statusMessage.getObjects())
                     {
                        Point3DReadOnly objectPosition = object.getObjectPoseInWorld().getTranslation();
                        if (positionNextStep.distanceXY(objectPosition) < DISTANCE_COLLISION_THRESHOLD)
                        {
                           gotoActionState.setFailed(true);
                           navigationFailureForObstacle = true;
                           navigationFailureObstacleName = object.getObjectNameAsString();
                           // Have the executor abort
                           ros2.publishToController(new AbortWalkingMessage());

                           plannedSteps = null;
                           break goToCollisionLoop;
                        }
                     }
                  }
                  else
                  {
                     LogTools.warn("Cannot check collision of next step");
                  }
               }
            }
            break;
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
            LogTools.info("Completed behavior: {}", statusMessage.getCompletedBehavior());
            // Jump to end of sequence
            state.getActionSequence().setExecutionNextIndex(state.getCheckPoints().get(state.getCheckPoints().size() - 1).getLeafIndex());
         }
      }
   }
}
