package us.ihmc.behaviors.behaviorTree.control.ai2r;

import behavior_msgs.msg.dds.AI2RActionFailureMessage;
import behavior_msgs.msg.dds.AI2RObjectMessage;
import behavior_msgs.msg.dds.AI2RStatusMessage;
import controller_msgs.msg.dds.AbortWalkingMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.condition.ConditionNodeDefinition.ConditionNodeType;
import us.ihmc.behaviors.behaviorTree.condition.ConditionNodeState;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;
import us.ihmc.behaviors.behaviorTree.LeafNodeState;
import us.ihmc.behaviors.behaviorTree.action.actions.ChestOrientationActionState;
import us.ihmc.behaviors.behaviorTree.action.actions.FootstepPlanActionState;
import us.ihmc.behaviors.behaviorTree.action.actions.HandPoseActionState;
import us.ihmc.behaviors.behaviorTree.action.actions.WaitDurationActionState;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectState;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.commons.thread.Throttler;

import java.util.ArrayList;
import java.util.List;

/**
 * Node that enables interaction with external reasoning modules
 */
public class AI2RNodeExecutor extends BehaviorTreeNodeExecutor<AI2RNodeState, AI2RNodeDefinition>
{
   private final Throttler statusThrottler = new Throttler().setFrequency(10.0);
   private final AI2RStatusMessage statusMessage = new AI2RStatusMessage();
   private final List<LeafNodeState<?>> failedLeaves = new ArrayList<>();

   private static final boolean CHECK_COLLISION_WITH_OBJECTS = false;
   private static final double DISTANCE_COLLISION_THRESHOLD = 0.6;
   private final BehaviorTreeRootNodeState actionSequence;
   private boolean navigationFailureForObstacle = false;
   private String navigationFailureObstacleName;
   private boolean actionFailureMissingFrame = false;
   private final AI2RSkillEditor skillEditor = new AI2RSkillEditor();

   public AI2RNodeExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new AI2RNodeState(id, rootNode.getState()), rootNode);

      actionSequence = rootNode.getState();

      resetStatusMessage();

      ros2ControllerHelper.subscribeViaCallback(AutonomyAPI.AI2R_COMMAND, message ->
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
            navigationFailureForObstacle = false;
            actionSequence.setExecutionNextIndex(commandedBehaviorIndex);
            actionSequence.setAutomaticExecution(true);

            resetStatusMessage();
            LogTools.warn("Automatic execution");
            statusMessage.setBehaviorInProgress(behaviorToExecuteName);
         }
      });
   }

   private void resetStatusMessage()
   {
      navigationFailureForObstacle = false;
      navigationFailureObstacleName = "";
      actionFailureMissingFrame = false;
      statusMessage.setBehaviorInProgress("-");
      statusMessage.setCompletedBehavior("-");
      statusMessage.setFailedBehavior("-");
      statusMessage.setGraspSide(RobotSide.RIGHT.toByte());
      statusMessage.getFailure().setActionName("-");
      statusMessage.getFailure().setActionType("-");
      statusMessage.getFailure().setCollisionName("-");
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
         ros2ControllerHelper.publish(AutonomyAPI.AI2R_STATUS, statusMessage);
      }
      endSequenceAfterBehaviorExecution();
      executeBehaviorLogic();
   }

   private void setSceneInfo()
   {
      statusMessage.getObjects().clear();
      for (BehaviorTreeSceneObjectState object : scene.getObjects())
      {
         AI2RObjectMessage objectMessage = statusMessage.getObjects().add();
         objectMessage.setObjectName(object.getName());
         ReferenceFrame nodeFrame = object.getReferenceFrame();
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
      for (var leaf : actionSequence.getOrderedLeaves())
      {
         if (leaf.getFailed() && !actionSequence.getAutomaticExecution())
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
                  statusMessage.setBehaviorInProgress("-");
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
                           LogTools.info("Detected footstep collision with {}", navigationFailureObstacleName);
                        }
                        failureMessage.setMissingFrame(actionFailureMissingFrame);
                        failureMessage.setActionType(walkAction.getDefinition().getClass().getSimpleName());
                     }
                     else if (action instanceof HandPoseActionState handPoseAction)
                     {
                        failureMessage.setOrientationTolerance(action.getOrientationDistanceToGoalTolerance());
                        failureMessage.setPositionTolerance(action.getPositionDistanceToGoalTolerance());

                        if (!handPoseAction.getCommandedTrajectory().isEmpty())
                        {
                           var desiredValue = handPoseAction.getCommandedTrajectory().getLastValueReadOnly();
                           var actualValue = handPoseAction.getCurrentPose().getValueReadOnly();

                           Quaternion errorOrientation = new Quaternion(actualValue.getOrientation());
                           errorOrientation.multiply(desiredValue.getOrientation());
                           failureMessage.getOrientationError().set(errorOrientation);

                           Point3D errorPosition = new Point3D(desiredValue.getPosition());
                           errorPosition.sub(actualValue.getPosition());
                           failureMessage.getPositionError().set(errorPosition);
                        }

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
                     if (conditionNodeState.getDefinition().getConditionType().getValue() == ConditionNodeType.PROXIMITY)
                     {
                        failureMessage.setMissingFrame(actionFailureMissingFrame);
                        failureMessage.setActionFrame(conditionNodeState.getDefinition().getProximityCheck().getFrameNameA());
                        double maxDistanceAllowed = conditionNodeState.getDefinition().getProximityCheck().getMaxDistance();
                        double currentDistance = conditionNodeState.getProximityCheck().getVectorBToA().norm();
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

   private void executeBehaviorLogic()
   {
      boolean trackingObjectsInProgress = false;
      leavesLoop:
      for (var leaf : actionSequence.getOrderedLeaves())
      {
         // Check if actions can't execute because of missing frames
         if (leaf.getIsNextForExecution())
         {
            if (!leaf.getCanExecute())
            {
               actionFailureMissingFrame = true;
               leaf.setFailed(true);
               failedLeaves.add(leaf);
            }
         }

         // Check if we are executing Receive or pick up  object action
         if (leaf.getParent().getDefinition().getName().contains("ReceiveObject") || leaf.getParent().getDefinition().getName().contains("PickUpObject"))
         {
            trackingObjectsInProgress |= leaf.getIsExecuting();
            if (leaf.getDefinition().getName().contains("Grasp") && leaf.getIsExecuting())
            {
               String objectGrasped = skillEditor.getObjectGrasped();
               RobotSide graspSide = skillEditor.getGraspSide();
               statusMessage.setObjectGrasped(objectGrasped);
               statusMessage.setGraspSide(graspSide.toByte());
               MovingReferenceFrame handControlFrame = syncedRobot.getFullRobotModel().getHandControlFrame(graspSide);
               RigidBodyTransform objectPoseInHandFrame = scene.getObject(objectGrasped).getReferenceFrame().getTransformToDesiredFrame(handControlFrame);
               statusMessage.getTransformGraspedObjectHand().set(objectPoseInHandFrame);
            }
         }

         if (leaf.getParent().getDefinition().getName().contains("Place"))
         {
            if (leaf.getDefinition().getName().contains("Release") && leaf.getIsExecuting())
            {
               statusMessage.setObjectGrasped("");
            }
         }

         // Check if we are executing Scan action and active/de-active foundationPose tracking
         if (leaf.getDefinition().getName().contains("SCANNING") && leaf instanceof WaitDurationActionState waitActionState)
         {
            trackingObjectsInProgress |= waitActionState.getIsExecuting();
         }

         // Check if Goto action is executing and if next steps are colliding with objects in the scene
         if (CHECK_COLLISION_WITH_OBJECTS)
         {
            if (leaf.getDefinition().getName().contains("Go to Action") && leaf instanceof FootstepPlanActionState gotoActionState)
            {
               if (gotoActionState.getIsExecuting())
               {
                  var footsteps = controllerStatusTracker.getFootstepTracker().getFootsteps();
                  // Check if the next step's pose is too close with any object in the scene
                  int stepsLeft = gotoActionState.getNumberOfIncompleteFootsteps();
                  if (stepsLeft > 3 && footsteps.size() > stepsLeft)
                  {
                     Point3DReadOnly positionNextNextStep = footsteps.get(footsteps.size()-1 - stepsLeft + 2).getLocation();
                     for (var object : statusMessage.getObjects())
                     {
                        Point3DReadOnly objectPosition = object.getObjectPoseInWorld().getTranslation();
                        if (positionNextNextStep.distanceXY(objectPosition) < DISTANCE_COLLISION_THRESHOLD)
                        {
                           gotoActionState.setFailed(true);
                           failedLeaves.add(gotoActionState);
                           navigationFailureForObstacle = true;
                           navigationFailureObstacleName = object.getObjectNameAsString();
                           // Have the executor abort
                           ros2ControllerHelper.publishToController(new AbortWalkingMessage());

                           break leavesLoop;
                        }
                     }
                  }
                  else
                  {
                     LogTools.warn("Cannot check collision of next step");
                  }
               }
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
            actionSequence.setExecutionNextIndex(state.getCheckPoints().get(state.getCheckPoints().size() - 1).getLeafIndex());
         }
      }
   }
}