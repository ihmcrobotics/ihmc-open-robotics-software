package us.ihmc.behaviors.behaviorTree.control.ai2r;

import behavior_msgs.AI2RActionFailureMessage;
import behavior_msgs.AI2RObjectMessage;
import behavior_msgs.AI2RScanMessage;
import behavior_msgs.AI2RStatusMessage;
import controller_msgs.RobotConfigurationData;
import controller_msgs.AbortWalkingMessage;
import toolbox_msgs.KinematicsToolboxOutputStatus;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeExecutor;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.LeafNodeState;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;
import us.ihmc.behaviors.behaviorTree.action.actions.SpineActionState;
import us.ihmc.behaviors.behaviorTree.action.actions.WalkActionState;
import us.ihmc.behaviors.behaviorTree.action.actions.ArmActionState;
import us.ihmc.behaviors.behaviorTree.action.actions.SceneActionState;
import us.ihmc.behaviors.behaviorTree.action.actions.WaitActionState;
import us.ihmc.behaviors.behaviorTree.condition.ConditionNodeDefinition.ConditionNodeType;
import us.ihmc.behaviors.behaviorTree.condition.ConditionNodeState;
import us.ihmc.behaviors.behaviorTree.control.GotoNodeDefinition;
import us.ihmc.behaviors.behaviorTree.control.CheckpointNodeState;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectState;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2log.ROS2LogRecord;
import us.ihmc.communication.ros2log.ROS2LogSerialization;
import us.ihmc.communication.ros2log.ROS2LogTimeSource;
import us.ihmc.communication.ros2.tf2.ROS2MutableFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.fastddsjava.cdr.idl.IDLStringSequence;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.RawImagePublisher;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.yolo.YOLOv8InstantDetection;
import us.ihmc.perception.detections.yolo.YOLOv8Tools;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Topic;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import static us.ihmc.communication.ros2.tf2.ROS2FrameTools.CAMERA_TO_OPTICAL_ROTATION;

/**
 * Node that enables interaction with external reasoning modules
 */
public class AI2RNodeExecutor extends BehaviorTreeNodeExecutor<AI2RNodeState, AI2RNodeDefinition>
{
   private static final double GO_TO_RANDOM_XY_RANGE_METERS = 2.0;
   private final Throttler statusThrottler = new Throttler().setFrequency(10.0);
   private final AI2RStatusMessage statusMessage = new AI2RStatusMessage();
   private final List<LeafNodeState<?>> failedLeaves = new ArrayList<>();

   private static final boolean CHECK_COLLISION_WITH_OBJECTS = false;
   private static final double DISTANCE_COLLISION_THRESHOLD = 0.6;
   private final BehaviorTreeRootNodeState actionSequence;
   private boolean navigationFailureForObstacle = false;
   private String navigationFailureObstacleName;
   private boolean actionFailureMissingFrame = false;
   private final AI2RSkillEditor skillEditor;

   private final Notification publishAnnotatedImage = new Notification();
   private final RawImagePublisher imagePublisher;
   private final ROS2MutableFrame annotatedImageFrame;
   private final ROS2Topic<KinematicsToolboxOutputStatus> kstOutputTopic;
   private final KinematicsToolboxOutputStatus kinematicsStatus = new KinematicsToolboxOutputStatus();
   private final RigidBodyTransform initialWalkingPose = new RigidBodyTransform();
   private final RigidBodyTransform relativePelvisPose = new RigidBodyTransform();

   private final Random random = new Random();
   private final Point3D baseGoToStancePoint = new Point3D();
   private final Point3D baseGoToFocalPoint = new Point3D();
   private boolean randomizationSessionActive = false;
   private boolean randomizationRunInProgress = false;
   private int randomizationsCompleted = 0;
   private int randomizationTargetCount = 0;
   private int goToCheckpointLeafIndex = -1;
   private String goToCheckpointName = "";
   private WalkActionState goToAction = null;
   private ROS2LogRecord ros2LogRecord = null;

   public AI2RNodeExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new AI2RNodeState(id, rootNode.getState()), rootNode);

      actionSequence = rootNode.getState();
      kstOutputTopic = KinematicsStreamingToolboxModule.getOutputStatusTopic(robotModel.getSimpleRobotName());

      imagePublisher = new RawImagePublisher(ros2ControllerHelper.getROS2Node());
      annotatedImageFrame = new ROS2MutableFrame("vlm_annotated_image_frame", ReferenceFrame.getWorldFrame());

      skillEditor = new AI2RSkillEditor(this, rootNode);

      resetStatusMessage();

      List<Integer> nonFingerIndices = FullRobotModelUtils.getAllJointsExcludingHandsIndices(syncedRobot.getFullRobotModel());
      float[] nonFingerValues = new float[nonFingerIndices.size()];
      ROS2Publisher<KinematicsToolboxOutputStatus> publisher = ros2ControllerHelper.getROS2Node().createPublisher(kstOutputTopic);
      syncedRobot.addRobotConfigurationDataReceivedCallback(() ->
      {
         if (ros2LogRecord != null)
         {
            RobotConfigurationData robotConfigurationData = syncedRobot.getLatestRobotConfigurationData();
            kinematicsStatus.setSequenceId(robotConfigurationData.getSequenceId());

            relativePelvisPose.set(robotConfigurationData.getRootOrientation().getQuaternion(), robotConfigurationData.getRootPosition().getPoint());
            initialWalkingPose.inverseTransform(relativePelvisPose);

            kinematicsStatus.getDesiredRootPosition().set(relativePelvisPose.getTranslation());
            kinematicsStatus.getDesiredRootOrientation().set(relativePelvisPose.getRotation());
            kinematicsStatus.getDesiredRootLinearVelocity().set(robotConfigurationData.getPelvisLinearVelocity());
            kinematicsStatus.getDesiredRootAngularVelocity().set(robotConfigurationData.getPelvisAngularVelocity());

            for (int i = 0; i < nonFingerValues.length; i++)
               nonFingerValues[i] = robotConfigurationData.getJointAngles().get(nonFingerIndices.get(i));
            kinematicsStatus.getDesiredJointAngles().clear();
            kinematicsStatus.getDesiredJointAngles().addAll(nonFingerValues);
            for (int i = 0; i < nonFingerValues.length; i++)
               nonFingerValues[i] = robotConfigurationData.getJointVelocities().get(nonFingerIndices.get(i));
            kinematicsStatus.getDesiredJointVelocities().clear();
            kinematicsStatus.getDesiredJointVelocities().addAll(nonFingerValues);

            publisher.publish(kinematicsStatus);
         }
      });

      ros2ControllerHelper.subscribeViaCallback(AutonomyAPI.AI2R_COMMAND, message ->
      {
         LogTools.info("Received command message: %s".formatted(message));

         // Prepare commanded behavior
         String behaviorToExecuteName = message.getBehaviorToExecuteAsString();
         if (behaviorToExecuteName.toLowerCase().contains("scan"))
         {
            publishAnnotatedImage.set();
         }
         int commandedBehaviorIndex = -1;
         for (int i = 0; i < state.getCheckpoints().size(); i++)
         {
            if (state.getCheckpoints().get(i).getDefinition().getName().equals(behaviorToExecuteName))
            {
               commandedBehaviorIndex = state.getCheckpoints().get(i).getLeafIndex();
               break;
            }
         }

         // Generic adaptable skills
         skillEditor.adaptSkills(behaviorToExecuteName, state, message);

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

      failedLeaves.clear();
      if (statusThrottler.run())
      {
         statusMessage.getRobotMidFeetUnderPelvisPoseInWorld().set(syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetUnderPelvisFrame));
         setSceneInfo();
         setAvailableBehaviors();
         setFailedBehaviors();
         ros2ControllerHelper.publish(AutonomyAPI.AI2R_STATUS, statusMessage);
      }

      if (publishAnnotatedImage.poll())
         publishYOLOAnnotatedImage();

      endSequenceAfterBehaviorExecution();
      executeBehaviorLogic();
      handleRandomizedGoToRecording();
   }

   private void handleRandomizedGoToRecording()
   {
      if (!definition.getRandomizeGoToActionEnabled())
      {
         if (randomizationSessionActive)
            resetRandomizationSession();
         return;
      }

      if (!randomizationSessionActive)
      {
         goToAction = findGoToAction();
         CheckpointNodeState goToCheckpoint = findGoToCheckpoint();
         if (goToAction == null || goToCheckpoint == null)
            return;

         goToCheckpointLeafIndex = goToCheckpoint.getLeafIndex();
         goToCheckpointName = goToCheckpoint.getDefinition().getName();
         baseGoToStancePoint.set(goToAction.getDefinition().getGoalStancePoint().getValueReadOnly());
         baseGoToFocalPoint.set(goToAction.getDefinition().getGoalFocalPoint().getValueReadOnly());
         randomizationTargetCount = Math.max(1, definition.getNumberOfRandomizationsValue());
         definition.setNumberOfRandomizationsValue(randomizationTargetCount);
         definition.modify();
         randomizationsCompleted = 0;
         randomizationRunInProgress = false;
         randomizationSessionActive = true;
      }

      if (randomizationsCompleted >= randomizationTargetCount)
      {
         definition.setRandomizeGoToActionEnabled(false);
         definition.modify();
         resetRandomizationSession();
         return;
      }

      if (!randomizationRunInProgress && !actionSequence.getAutomaticExecution())
      {
         randomizeGoToGoalXY(goToAction);
         actionSequence.setExecutionNextIndex(goToCheckpointLeafIndex);
         actionSequence.setAutomaticExecution(true);
         statusMessage.setBehaviorInProgress(goToCheckpointName);
         startGoToRecording();
         randomizationRunInProgress = true;
      }
      else if (randomizationRunInProgress && !actionSequence.getAutomaticExecution())
      {
         stopGoToRecording();
         randomizationsCompleted++;
         definition.setNumberOfRandomizationsValue(Math.max(0, randomizationTargetCount - randomizationsCompleted));
         definition.modify();
         randomizationRunInProgress = false;
      }
   }

   private WalkActionState findGoToAction()
   {
      for (var leaf : actionSequence.getOrderedLeaves())
      {
         String leafNameLower = leaf.getDefinition().getName().toLowerCase();
         if (leaf instanceof WalkActionState walkActionState && (leafNameLower.contains("go to action") || leafNameLower.contains("goto")))
            return walkActionState;
      }
      return null;
   }

   private CheckpointNodeState findGoToCheckpoint()
   {
      for (CheckpointNodeState checkpoint : state.getCheckpoints())
      {
         String checkpointNameLower = checkpoint.getDefinition().getName().toLowerCase();
         if (checkpointNameLower.contains("goto") || checkpointNameLower.contains("go to"))
            return checkpoint;
      }
      return null;
   }

   private void randomizeGoToGoalXY(WalkActionState walkActionState)
   {
      // Always randomize from the session's initial values, never from the previous randomized result.
      Point3D randomizedStancePoint = new Point3D(baseGoToStancePoint);
      Point3D randomizedFocalPoint = new Point3D(baseGoToFocalPoint);

      // Translate stance/focal together so position randomization does not accidentally change yaw.
      double translationX = randomOffset();
      double translationY = randomOffset();
      randomizedStancePoint.add(translationX, translationY, 0.0);
      randomizedFocalPoint.add(translationX, translationY, 0.0);

      // Apply an explicit yaw randomization by rotating focal around stance.
      double yawOffset = randomYawOffsetGaussian();
      double vectorX = randomizedFocalPoint.getX() - randomizedStancePoint.getX();
      double vectorY = randomizedFocalPoint.getY() - randomizedStancePoint.getY();
      double cosYaw = Math.cos(yawOffset);
      double sinYaw = Math.sin(yawOffset);
      double rotatedX = vectorX * cosYaw - vectorY * sinYaw;
      double rotatedY = vectorX * sinYaw + vectorY * cosYaw;
      randomizedFocalPoint.set(randomizedStancePoint.getX() + rotatedX, randomizedStancePoint.getY() + rotatedY, randomizedFocalPoint.getZ());

      walkActionState.getDefinition().getGoalStancePoint().getValueAndModify().set(randomizedStancePoint);
      walkActionState.getDefinition().getGoalFocalPoint().getValueAndModify().set(randomizedFocalPoint);
   }

   private double randomOffset()
   {
      return (random.nextDouble() * 2.0 - 1.0) * GO_TO_RANDOM_XY_RANGE_METERS;
   }

   private static final double MAX_YAW_DEG = 90.0;
   private static final double MAX_YAW_RAD = Math.toRadians(MAX_YAW_DEG);

   // Choose sigma so most samples lie within [-MAX_YAW_RAD, MAX_YAW_RAD]
   // For example: 3σ ≈ MAX  ⇒  σ = MAX/3
   private static final double YAW_STD_RAD = MAX_YAW_RAD / 3.0;

   private double randomYawOffsetGaussian()
   {
      // Standard normal (mean 0, std 1).
      double g = random.nextGaussian();

      // Scale to desired std dev in radians
      double yaw = g * YAW_STD_RAD;

      // Clamp hard to [-MAX_YAW_RAD, MAX_YAW_RAD]
      if (yaw > MAX_YAW_RAD)
         yaw = MAX_YAW_RAD;
      else if (yaw < -MAX_YAW_RAD)
         yaw = -MAX_YAW_RAD;

      return yaw;
   }


   private void startGoToRecording()
   {
      if (ros2LogRecord != null)
         return;

      initialWalkingPose.set(syncedRobot.getReferenceFrames().getMidFeetUnderPelvisFrame().getTransformToRoot());
      ros2LogRecord = new ROS2LogRecord(robotModel.getSimpleRobotName(), List.of(kstOutputTopic), ROS2LogTimeSource.SYSTEM, ROS2LogSerialization.JSON);
      ros2LogRecord.start();
   }

   private void stopGoToRecording()
   {
      if (ros2LogRecord == null)
         return;

      ROS2LogRecord ros2LogRecordLocal = ros2LogRecord;
      ThreadTools.startAThread(() ->
      {
         ros2LogRecordLocal.stop();
         ThreadTools.parkAtLeast(1.0);
         ros2LogRecordLocal.destroy();
      }, "StopAI2RGoToLog");
      ros2LogRecord = null;
   }

   private void resetRandomizationSession()
   {
      stopGoToRecording();

      // Restore the original points captured at the beginning of the session.
      if (goToAction != null)
      {
         goToAction.getDefinition().getGoalStancePoint().getValueAndModify().set(baseGoToStancePoint);
         goToAction.getDefinition().getGoalFocalPoint().getValueAndModify().set(baseGoToFocalPoint);
      }

      randomizationSessionActive = false;
      randomizationRunInProgress = false;
      randomizationsCompleted = 0;
      randomizationTargetCount = 0;
      goToCheckpointLeafIndex = -1;
      goToCheckpointName = "";
      goToAction = null;
   }

   private void addNode()
   {
      BehaviorTreeExecutor tree = rootNode.getTree();
      BehaviorTreeNodeExecutor<?, ?> node = tree.getNodeBuilder().createNode(GotoNodeDefinition.class, tree.getAndIncrementNextID(), rootNode);
      node.getDefinition().modify();
      LogTools.info("Creating node: {}:{}", node.getDefinition().getName(), node.getState().getID());
      tree.getTopologyChangeQueue().queueAppendChildModify(this, node);
      tree.modifyTreeTopology();
   }

   private void removeNode(String nodeName)
   {
      for (BehaviorTreeNodeExecutor<?, ?> child : getChildren())
         if (child.getDefinition().getName().equals(nodeName))
         {
            BehaviorTreeExecutor tree = rootNode.getTree();
            LogTools.info("Removing node: {}:{}", child.getDefinition().getName(), child.getState().getID());
            tree.getTopologyChangeQueue().queueDetachChildModify(child);
            tree.modifyTreeTopology();
            child.destroy();
         }
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
      statusMessage.getAvailableBehaviors().clear();
      for (int i = 0; i < state.getCheckpoints().size(); i++)
      {
         String checkPointName = state.getCheckpoints().get(i).getDefinition().getName();
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
            for (int i = state.getCheckpoints().size() - 1; i >= 0; i--)
            {
               var checkpoint = state.getCheckpoints().get(i);
               // Check if the checkpoint is before the failed leaf
               if (checkpoint.getLeafIndex() < leaf.getLeafIndex())
               {
                  // Retrieve the name of the closest previous checkpoint
                  String checkpointName = checkpoint.getDefinition().getName();
                  statusMessage.setFailedBehavior(checkpointName);
                  statusMessage.setBehaviorInProgress("-");
                  if (leaf instanceof ActionNodeState<?> action)
                  {
                     AI2RActionFailureMessage failureMessage = statusMessage.getFailure();
                     failureMessage.setActionName(action.getDefinition().getName());
                     if (action instanceof WalkActionState walkAction)
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
                     else if (action instanceof ArmActionState armAction)
                     {
                        failureMessage.setOrientationTolerance(action.getOrientationDistanceToGoalTolerance());
                        failureMessage.setPositionTolerance(action.getPositionDistanceToGoalTolerance());

                        if (!armAction.getCommandedTrajectory().isEmpty())
                        {
                           var desiredValue = armAction.getCommandedTrajectory().getLastValueReadOnly();
                           var actualValue = armAction.getCurrentPose().getValueReadOnly();

                           Quaternion errorOrientation = new Quaternion(actualValue.getOrientation());
                           errorOrientation.multiply(desiredValue.getOrientation());
                           failureMessage.getOrientationError().set(errorOrientation);

                           Point3D errorPosition = new Point3D(desiredValue.getPosition());
                           errorPosition.sub(actualValue.getPosition());
                           failureMessage.getPositionError().set(errorPosition);
                        }

                        failureMessage.setActionFrame(armAction.getDefinition().getPalmParentFrameName());
                        failureMessage.setActionType(armAction.getDefinition().getClass().getSimpleName());
                     }

                     if (action instanceof SpineActionState chestAction)
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
                        failureMessage.getPositionError().getPoint().set(error, 0.0, 0.0);
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

         // Check if Goto action is executing and if next steps are colliding with objects in the scene
         if (CHECK_COLLISION_WITH_OBJECTS)
         {
            if (leaf.getDefinition().getName().contains("Go to Action") && leaf instanceof WalkActionState gotoActionState)
            {
               if (gotoActionState.getIsExecuting())
               {
                  var footsteps = controllerStatusTracker.getFootstepTracker().getFootsteps();
                  // Check if the next step's pose is too close with any object in the scene
                  int stepsLeft = gotoActionState.getNumberOfIncompleteFootsteps();
                  if (stepsLeft > 3 && footsteps.size() > stepsLeft)
                  {
                     Point3DReadOnly positionNextNextStep = footsteps.get(footsteps.size()-1 - stepsLeft + 2).getLocation().getPoint();
                     for (var object : statusMessage.getObjects())
                     {
                        Point3DReadOnly objectPosition = object.getObjectPoseInWorld().getPose().getPosition();
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
      for (int i = 0; i < state.getCheckpoints().size(); i++)
      {
         // If we execute the end of behavior checkpoint, we communicate that in the status
         if (state.getCheckpoints().get(i).getDefinition().getName().contains("END OF") && state.getCheckpoints().get(i).getIsExecuting())
         {
            // ! WARNING !
            // Assuming checkpoints are only used at the beginning and end of a behavior
            statusMessage.setCompletedBehavior(state.getCheckpoints().get(i - 1).getDefinition().getName());
            statusMessage.setBehaviorInProgress("-");
            // Jump to end of sequence
            actionSequence.setExecutionNextIndex(state.getCheckpoints().get(state.getCheckpoints().size() - 1).getLeafIndex());

            // If SCAN failed to find certain objects, reset failure
            // TODO do something else if scan cannot find all objects?
            if (state.getCheckpoints().get(i).getDefinition().getName().contains("SCAN"))
            {
               for (var leaf : actionSequence.getOrderedLeaves())
               {
                  if (leaf.getFailed() && leaf instanceof SceneActionState)
                  {
                     leaf.setFailed(false);
                  }
               }
               failedLeaves.clear();
               statusMessage.setFailedBehavior("-");
               statusMessage.getFailure().setActionName("-");
               statusMessage.getFailure().setActionType("-");
               statusMessage.getFailure().setCollisionName("-");
            }
         }
      }
   }

   private void publishYOLOAnnotatedImage()
   {
      List<YOLOv8InstantDetection> yoloDetections = new ArrayList<>();
      Map<YOLOv8InstantDetection, Integer> detectionIdMap = new HashMap<>();
      RawImage colorImage = null;
      RawImage annotatedImage = null;

      for (PersistentDetection persistentDetection : scene.getPersistentDetections())
      {
         if (persistentDetection.getInstantDetectionClass() != YOLOv8InstantDetection.class || !persistentDetection.isStable())
            continue;

         YOLOv8InstantDetection detection = (YOLOv8InstantDetection) persistentDetection.getMostRecentDetection();
         if (colorImage == null)
         {
            colorImage = detection.getColorImage().get();
            if (colorImage == null)
               continue;

            annotatedImage = new RawImage(colorImage);
         }
         yoloDetections.add(detection);
         detectionIdMap.put(detection, persistentDetection.getID());
      }

      if (annotatedImage == null)
         return;

      YOLOv8Tools.drawObjectOutlines(colorImage.getCpuImageMat(), annotatedImage.getCpuImageMat(), yoloDetections, detection ->
      {
         int id = detectionIdMap.get(detection);
         return id + ": " + detection.getDetectedObjectName();
      });

      RigidBodyTransform transformToWorld = new RigidBodyTransform(annotatedImage.getTransformToWorld());
      transformToWorld.appendOrientation(CAMERA_TO_OPTICAL_ROTATION);
      annotatedImageFrame.setNewTransformToParent(transformToWorld);
      annotatedImageFrame.update();

      imagePublisher.publishImage(PerceptionAPI.YOLO_VLM_ANNOTATED_IMAGE, annotatedImage, annotatedImageFrame);
      imagePublisher.publishImage(PerceptionAPI.YOLO_VML_ANNOTATED_IMAGE_CAMERA_INFO, annotatedImage, annotatedImageFrame);

      colorImage.release();
      annotatedImage.release();
   }

   @Override
   public void destroy()
   {
      super.destroy();

      resetRandomizationSession();
      imagePublisher.close();
      annotatedImageFrame.remove();
   }
}
