package us.ihmc.behaviors.behaviorTree.control.ai2r;

import behavior_msgs.msg.dds.AI2RActionFailureMessage;
import behavior_msgs.msg.dds.AI2RObjectMessage;
import behavior_msgs.msg.dds.AI2RStatusMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.AbortWalkingMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeExecutor;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.LeafNodeState;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;
import us.ihmc.behaviors.behaviorTree.action.actions.ArmActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.PelvisActionState;
import us.ihmc.behaviors.behaviorTree.action.actions.SpineActionState;
import us.ihmc.behaviors.behaviorTree.action.actions.WalkActionState;
import us.ihmc.behaviors.behaviorTree.action.actions.ArmActionState;
import us.ihmc.behaviors.behaviorTree.action.actions.SceneActionState;
import us.ihmc.avatar.arm.PresetArmConfiguration;
import us.ihmc.commons.RandomNumbers;
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
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.RawImagePublisher;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.yolo.YOLOv8InstantDetection;
import us.ihmc.perception.detections.yolo.YOLOv8Tools;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Topic;

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
   private double[] defaultRightHandJointAngles = new double[ArmActionDefinition.MAX_NUMBER_OF_JOINTS];
   private double[] defaultLeftHandJointAngles = new double[ArmActionDefinition.MAX_NUMBER_OF_JOINTS];
   private PresetArmConfiguration defaultRightHandPreset = null;
   private PresetArmConfiguration defaultLeftHandPreset = null;
   private final double[] defaultSpineJointAngles = new double[3];
   private final RigidBodyTransform defaultPelvisTransform = new RigidBodyTransform();
   private double defaultRightHandDuration = 4.0;
   private double defaultLeftHandDuration = 4.0;
   private double defaultSpineDuration = 4.0;
   private double defaultPelvisDuration = 4.0;
   private boolean wholeBodyDefaultsInitialized = false;
   private boolean wholeBodyRandomizationSessionActive = false;
   private boolean wholeBodyRandomizationRunInProgress = false;
   private int wholeBodyRandomizationsCompleted = 0;
   private int wholeBodyRandomizationTargetCount = 0;
   private int wholeBodyStartLeafIndex = -1;
   private String wholeBodyBehaviorName = "WHOLE BODY";
   private ArmActionState wholeBodyRightArmAction = null;
   private ArmActionState wholeBodyLeftArmAction = null;
   private SpineActionState wholeBodySpineAction = null;
   private PelvisActionState wholeBodyPelvisAction = null;
   private static final double PELVIS_PITCH_MIN_RADIANS = 0.0;
   private static final double PELVIS_PITCH_MAX_RADIANS = Math.toRadians(30.0);
   private static final double PELVIS_HEIGHT_MIN_METERS = 0.56;
   private static final double PELVIS_HEIGHT_MAX_METERS = 0.94;
   private static final double PELVIS_HEIGHT_CENTER_METERS = 0.9;
   /**
    * For centered Gaussian sampling, this is the target probability mass within the chosen
    * reference distance from the center. Example: 0.90 means 90% of samples lie within that distance.
    */
   private static final double CENTERED_GAUSSIAN_TARGET_PROBABILITY = 0.80;
   private static final double ARM_DURATION_MEAN_SECONDS = 1.5;
   private static final double ARM_DURATION_MIN_SECONDS = 0.8;
   private static final double ARM_DURATION_MAX_SECONDS = 4.0;
   private static final double DURATION_MIN_SECONDS = 1.0;
   private static final double DURATION_MAX_SECONDS = 4.0;

   public AI2RNodeExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new AI2RNodeState(id, rootNode.getState()), rootNode);

      actionSequence = rootNode.getState();
      kstOutputTopic = KinematicsStreamingToolboxModule.getOutputStatusTopic(robotModel.getSimpleRobotName());

      imagePublisher = new RawImagePublisher(ros2ControllerHelper.getROS2Node());
      annotatedImageFrame = new ROS2MutableFrame("vlm_annotated_image_frame", ReferenceFrame.getWorldFrame());

      skillEditor = new AI2RSkillEditor(this, rootNode);

      resetStatusMessage();

      defaultLeftHandJointAngles = robotModel.getPresetArmConfiguration(RobotSide.LEFT, PresetArmConfiguration.N_POSE);
      defaultRightHandJointAngles = robotModel.getPresetArmConfiguration(RobotSide.RIGHT, PresetArmConfiguration.N_POSE);
      List<Integer> nonFingerIndices = FullRobotModelUtils.getAllJointsExcludingHandsIndices(syncedRobot.getFullRobotModel());
      float[] nonFingerValues = new float[nonFingerIndices.size()];
      ROS2Publisher<KinematicsToolboxOutputStatus> publisher = ros2ControllerHelper.getROS2Node().createPublisher(kstOutputTopic);
      syncedRobot.addRobotConfigurationDataReceivedCallback(() ->
      {
         if (ros2LogRecord != null)
         {
            RobotConfigurationData robotConfigurationData = syncedRobot.getLatestRobotConfigurationData();
            kinematicsStatus.setSequenceId(robotConfigurationData.getSequenceId());

            relativePelvisPose.set(robotConfigurationData.getRootOrientation(), robotConfigurationData.getRootPosition());
            initialWalkingPose.inverseTransform(relativePelvisPose);

            kinematicsStatus.getDesiredRootPosition().set(relativePelvisPose.getTranslation());
            kinematicsStatus.getDesiredRootOrientation().set(relativePelvisPose.getRotation());
            kinematicsStatus.getDesiredRootLinearVelocity().set(robotConfigurationData.getPelvisLinearVelocity());
            kinematicsStatus.getDesiredRootAngularVelocity().set(robotConfigurationData.getPelvisAngularVelocity());

            for (int i = 0; i < nonFingerValues.length; i++)
               nonFingerValues[i] = robotConfigurationData.getJointAngles().get(nonFingerIndices.get(i));
            kinematicsStatus.getDesiredJointAngles().clear();
            kinematicsStatus.getDesiredJointAngles().add(nonFingerValues);
            for (int i = 0; i < nonFingerValues.length; i++)
               nonFingerValues[i] = robotConfigurationData.getJointVelocities().get(nonFingerIndices.get(i));
            kinematicsStatus.getDesiredJointVelocities().clear();
            kinematicsStatus.getDesiredJointVelocities().add(nonFingerValues);

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
      handleWholeBodyActionRandomizationRequest();
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
         startRecording();
         randomizationRunInProgress = true;
      }
      else if (randomizationRunInProgress && !actionSequence.getAutomaticExecution())
      {
         stopRecording();
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


   private void startRecording()
   {
      if (ros2LogRecord != null)
         return;

      initialWalkingPose.set(syncedRobot.getReferenceFrames().getMidFeetUnderPelvisFrame().getTransformToRoot());
      ros2LogRecord = new ROS2LogRecord(robotModel.getSimpleRobotName(), List.of(kstOutputTopic), ROS2LogTimeSource.SYSTEM, ROS2LogSerialization.JSON);
      ros2LogRecord.start();
   }

   private void stopRecording()
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
      stopRecording();

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

   private void handleWholeBodyActionRandomizationRequest()
   {
      if (!definition.getRandomizeWholeBodyActionEnabled())
      {
         if (wholeBodyRandomizationSessionActive)
            resetWholeBodyRandomizationSession();
         return;
      }

      if (!wholeBodyRandomizationSessionActive)
      {
         wholeBodyRightArmAction = null;
         wholeBodyLeftArmAction = null;
         wholeBodySpineAction = null;
         wholeBodyPelvisAction = null;
         wholeBodyStartLeafIndex = Integer.MAX_VALUE;

         for (LeafNodeState<?> leaf : actionSequence.getOrderedLeaves())
         {
            String leafNameLower = leaf.getDefinition().getName().toLowerCase();
            if (!leafNameLower.contains("whole body"))
               continue;

            wholeBodyStartLeafIndex = Math.min(wholeBodyStartLeafIndex, leaf.getLeafIndex());

            if (leaf instanceof ArmActionState armActionState)
            {
               if (armActionState.getDefinition().getSide() == RobotSide.RIGHT)
                  wholeBodyRightArmAction = armActionState;
               else
                  wholeBodyLeftArmAction = armActionState;
            }
            else if (leaf instanceof SpineActionState spineActionState)
            {
               wholeBodySpineAction = spineActionState;
            }
            else if (leaf instanceof PelvisActionState pelvisActionState)
            {
               wholeBodyPelvisAction = pelvisActionState;
            }
         }

         if (wholeBodyRightArmAction == null && wholeBodyLeftArmAction == null && wholeBodySpineAction == null && wholeBodyPelvisAction == null)
         {
            for (LeafNodeState<?> leaf : actionSequence.getOrderedLeaves())
            {
               wholeBodyStartLeafIndex = Math.min(wholeBodyStartLeafIndex, leaf.getLeafIndex());

               if (leaf instanceof ArmActionState armActionState)
               {
                  if (armActionState.getDefinition().getSide() == RobotSide.RIGHT && wholeBodyRightArmAction == null)
                     wholeBodyRightArmAction = armActionState;
                  else if (armActionState.getDefinition().getSide() == RobotSide.LEFT && wholeBodyLeftArmAction == null)
                     wholeBodyLeftArmAction = armActionState;
               }
               else if (leaf instanceof SpineActionState spineActionState && wholeBodySpineAction == null)
               {
                  wholeBodySpineAction = spineActionState;
               }
               else if (leaf instanceof PelvisActionState pelvisActionState && wholeBodyPelvisAction == null)
               {
                  wholeBodyPelvisAction = pelvisActionState;
               }
            }
         }

         if (wholeBodyRightArmAction == null && wholeBodyLeftArmAction == null && wholeBodySpineAction == null && wholeBodyPelvisAction == null)
            return;

         if (wholeBodyStartLeafIndex == Integer.MAX_VALUE)
            wholeBodyStartLeafIndex = actionSequence.getOrderedLeaves().isEmpty() ? -1 : actionSequence.getOrderedLeaves().get(0).getLeafIndex();
         if (wholeBodyStartLeafIndex < 0)
            return;

         initializeWholeBodyDefaults(wholeBodyRightArmAction, wholeBodyLeftArmAction, wholeBodySpineAction, wholeBodyPelvisAction);
         wholeBodyRandomizationTargetCount = Math.max(1, definition.getNumberOfRandomizationsValue());
         definition.setNumberOfRandomizationsValue(wholeBodyRandomizationTargetCount);
         definition.modify();
         wholeBodyRandomizationsCompleted = 0;
         wholeBodyRandomizationRunInProgress = false;
         wholeBodyRandomizationSessionActive = true;
         startRecording();
      }

      if (wholeBodyRandomizationsCompleted >= wholeBodyRandomizationTargetCount)
      {
         definition.setRandomizeWholeBodyActionEnabled(false);
         definition.modify();
         resetWholeBodyRandomizationSession();
         return;
      }

      if (!wholeBodyRandomizationRunInProgress && !actionSequence.getAutomaticExecution())
      {
         int[] mask = sampleWholeBodyMask();
         definition.setLeftHandMask(mask[0]);
         definition.setRightHandMask(mask[1]);
         definition.setSpineMask(mask[2]);
         definition.setPelvisMask(mask[3]);
         definition.modify();

         if (wholeBodyRightArmAction != null)
         {
            if (mask[1] == 1)
               applyRightHandSetting(wholeBodyRightArmAction);
         }

         if (wholeBodyLeftArmAction != null)
         {
            if (mask[0] == 1)
               applyLeftHandSetting(wholeBodyLeftArmAction);
         }

         if (wholeBodySpineAction != null)
         {
            if (mask[2] == 1)
               applySpineSetting(wholeBodySpineAction);
            else
               restoreSpineDefaults(wholeBodySpineAction);
         }

         if (wholeBodyPelvisAction != null)
         {
            if (mask[3] == 1)
               applyPelvisSetting(wholeBodyPelvisAction);
            else
               restorePelvisDefaults(wholeBodyPelvisAction);
         }

         actionSequence.setExecutionNextIndex(wholeBodyStartLeafIndex);
         actionSequence.setAutomaticExecution(true);
         statusMessage.setBehaviorInProgress(wholeBodyBehaviorName);
         wholeBodyRandomizationRunInProgress = true;
      }
      else if (wholeBodyRandomizationRunInProgress && !actionSequence.getAutomaticExecution())
      {
         wholeBodyRandomizationsCompleted++;
         definition.setNumberOfRandomizationsValue(Math.max(0, wholeBodyRandomizationTargetCount - wholeBodyRandomizationsCompleted));
         definition.modify();
         wholeBodyRandomizationRunInProgress = false;
      }
   }

   private void resetWholeBodyRandomizationSession()
   {
      stopRecording();

      if (wholeBodyRightArmAction != null)
         restoreRightHandDefaults(wholeBodyRightArmAction);
      if (wholeBodyLeftArmAction != null)
         restoreLeftHandDefaults(wholeBodyLeftArmAction);
      if (wholeBodySpineAction != null)
         restoreSpineDefaults(wholeBodySpineAction);
      if (wholeBodyPelvisAction != null)
         restorePelvisDefaults(wholeBodyPelvisAction);

      wholeBodyRandomizationSessionActive = false;
      wholeBodyRandomizationRunInProgress = false;
      wholeBodyRandomizationsCompleted = 0;
      wholeBodyRandomizationTargetCount = 0;
      wholeBodyStartLeafIndex = -1;
      wholeBodyRightArmAction = null;
      wholeBodyLeftArmAction = null;
      wholeBodySpineAction = null;
      wholeBodyPelvisAction = null;
   }

   private void initializeWholeBodyDefaults(ArmActionState rightArmAction, ArmActionState leftArmAction, SpineActionState spineAction, PelvisActionState pelvisAction)
   {
      if (wholeBodyDefaultsInitialized)
         return;

      if (rightArmAction != null)
      {
         var rightArmDefinition = rightArmAction.getDefinition();
         defaultRightHandDuration = rightArmDefinition.getTrajectoryDuration();
         defaultRightHandPreset = rightArmDefinition.getPreset();
         for (int i = 0; i < defaultRightHandJointAngles.length; i++)
            defaultRightHandJointAngles[i] = rightArmDefinition.getJointAngles().getValueReadOnly(i);
      }

      if (leftArmAction != null)
      {
         var leftArmDefinition = leftArmAction.getDefinition();
         defaultLeftHandDuration = leftArmDefinition.getTrajectoryDuration();
         defaultLeftHandPreset = leftArmDefinition.getPreset();
         for (int i = 0; i < defaultLeftHandJointAngles.length; i++)
            defaultLeftHandJointAngles[i] = leftArmDefinition.getJointAngles().getValueReadOnly(i);
      }

      if (spineAction != null)
      {
         for (int i = 0; i < defaultSpineJointAngles.length; i++)
            defaultSpineJointAngles[i] = spineAction.getDefinition().getJointAngles().getValueReadOnly(i);
         defaultSpineDuration = spineAction.getDefinition().getTrajectoryDuration();
      }

      if (pelvisAction != null)
      {
         defaultPelvisTransform.set(pelvisAction.getDefinition().getPelvisToParentTransform().getValueReadOnly());
         defaultPelvisDuration = pelvisAction.getDefinition().getTrajectoryDuration();
      }

      wholeBodyDefaultsInitialized = true;
   }

   private void applyRightHandSetting(ArmActionState rightArmAction)
   {
      var actionDefinition = rightArmAction.getDefinition();
      double[] randomizedJointPositions = generateRandomJointPositions(random, getArmJoints(RobotSide.RIGHT));
      actionDefinition.setUsePredefinedJointAngles(true);
      actionDefinition.setPreset(null); // CUSTOM_ANGLES
      for (int i = 0; i < Math.min(actionDefinition.getJointAngles().getLength(), randomizedJointPositions.length); i++)
         actionDefinition.getJointAngles().setValue(i, randomizedJointPositions[i]);
      actionDefinition.setTrajectoryDuration(randomArmDuration());
      actionDefinition.modify();
   }

   private void restoreRightHandDefaults(ArmActionState rightArmAction)
   {
      var actionDefinition = rightArmAction.getDefinition();
      actionDefinition.setUsePredefinedJointAngles(true);
      actionDefinition.setPreset(defaultRightHandPreset);
      for (int i = 0; i < defaultRightHandJointAngles.length; i++)
         actionDefinition.getJointAngles().setValue(i, defaultRightHandJointAngles[i]);
      actionDefinition.setTrajectoryDuration(defaultRightHandDuration);
      actionDefinition.modify();
   }

   private void applyLeftHandSetting(ArmActionState leftArmAction)
   {
      var actionDefinition = leftArmAction.getDefinition();
      double[] randomizedJointPositions = generateRandomJointPositions(random, getArmJoints(RobotSide.LEFT));
      actionDefinition.setUsePredefinedJointAngles(true);
      actionDefinition.setPreset(null); // CUSTOM_ANGLES
      for (int i = 0; i < Math.min(actionDefinition.getJointAngles().getLength(), randomizedJointPositions.length); i++)
         actionDefinition.getJointAngles().setValue(i, randomizedJointPositions[i]);
      actionDefinition.setTrajectoryDuration(randomArmDuration());
      actionDefinition.modify();
   }

   private void restoreLeftHandDefaults(ArmActionState leftArmAction)
   {
      var actionDefinition = leftArmAction.getDefinition();
      actionDefinition.setUsePredefinedJointAngles(true);
      actionDefinition.setPreset(defaultLeftHandPreset);
      for (int i = 0; i < defaultLeftHandJointAngles.length; i++)
         actionDefinition.getJointAngles().setValue(i, defaultLeftHandJointAngles[i]);
      actionDefinition.setTrajectoryDuration(defaultLeftHandDuration);
      actionDefinition.modify();
   }

   private OneDoFJointBasics[] getArmJoints(RobotSide side)
   {
      ArmJointName[] armJointNames = syncedRobot.getRobotModel().getJointMap().getArmJointNames(side);
      OneDoFJointBasics[] armJoints = new OneDoFJointBasics[armJointNames.length];
      for (int i = 0; i < armJointNames.length; i++)
         armJoints[i] = syncedRobot.getFullRobotModel().getArmJoint(side, armJointNames[i]);
      return armJoints;
   }

   private static double[] generateRandomJointPositions(Random random, OneDoFJointBasics[] armJoints)
   {
      double[] desiredJointPositions = new double[armJoints.length];
      for (int i = 0; i < armJoints.length; i++)
      {
         OneDoFJointBasics joint = armJoints[i];
         desiredJointPositions[i] = RandomNumbers.nextDouble(random, joint.getJointLimitLower(), joint.getJointLimitUpper());
      }
      return desiredJointPositions;
   }

   private void applySpineSetting(SpineActionState spineAction)
   {
      var actionDefinition = spineAction.getDefinition();
      for (int i = 0; i < defaultSpineJointAngles.length; i++)
         actionDefinition.getJointAngles().setValue(i, defaultSpineJointAngles[i]);

      SpineJointName[] spineJointNames = syncedRobot.getRobotModel().getJointMap().getSpineJointNames();
      int yawIndex = -1;
      double yawMin = Double.NEGATIVE_INFINITY;
      double yawMax = Double.POSITIVE_INFINITY;
      int spineJointCount = Math.min(spineJointNames.length, defaultSpineJointAngles.length);
      for (int i = 0; i < spineJointCount; i++)
      {
         if (spineJointNames[i] == SpineJointName.SPINE_YAW)
         {
            yawIndex = i;
            var yawJoint = syncedRobot.getFullRobotModel().getSpineJoint(spineJointNames[i]);
            yawMin = yawJoint.getJointLimitLower();
            yawMax = yawJoint.getJointLimitUpper();
            break;
         }
      }

      if (yawIndex >= 0)
      {
         double yawReferenceDistance = Math.min(Math.abs(yawMin), Math.abs(yawMax));
         double yawSigma = computeGaussianSigmaForReferenceDistance(yawReferenceDistance, CENTERED_GAUSSIAN_TARGET_PROBABILITY);
         double randomizedYaw = sampleTruncatedGaussian(0.0, yawMin, yawMax, yawSigma);
         actionDefinition.getJointAngles().setValue(yawIndex, randomizedYaw);
      }

      actionDefinition.setTrajectoryDuration(randomDuration());
      actionDefinition.modify();
   }

   private void restoreSpineDefaults(SpineActionState spineAction)
   {
      var definition = spineAction.getDefinition();
      for (int i = 0; i < defaultSpineJointAngles.length; i++)
         definition.getJointAngles().setValue(i, defaultSpineJointAngles[i]);
      definition.setTrajectoryDuration(defaultSpineDuration);
      definition.modify();
   }

   private void applyPelvisSetting(PelvisActionState pelvisAction)
   {
      var actionDefinition = pelvisAction.getDefinition();
      var transform = actionDefinition.getPelvisToParentTransform().getValueAndModify();
      transform.set(defaultPelvisTransform);

      double pelvisPitchReferenceDistance = PELVIS_PITCH_MAX_RADIANS - PELVIS_PITCH_MIN_RADIANS;
      double pelvisPitchSigma = computeGaussianSigmaForReferenceDistance(pelvisPitchReferenceDistance, CENTERED_GAUSSIAN_TARGET_PROBABILITY);
      double pelvisPitch = sampleTruncatedGaussian(PELVIS_PITCH_MIN_RADIANS,
                                                   PELVIS_PITCH_MIN_RADIANS,
                                                   PELVIS_PITCH_MAX_RADIANS,
                                                   pelvisPitchSigma);
      transform.getRotation().setYawPitchRoll(defaultPelvisTransform.getRotation().getYaw(), pelvisPitch, defaultPelvisTransform.getRotation().getRoll());

      double pelvisHeightReferenceDistance = 0.1;
      double pelvisHeightSigma = computeGaussianSigmaForReferenceDistance(pelvisHeightReferenceDistance, CENTERED_GAUSSIAN_TARGET_PROBABILITY);
      double pelvisHeight = sampleTruncatedGaussian(PELVIS_HEIGHT_CENTER_METERS,
                                                    PELVIS_HEIGHT_MIN_METERS,
                                                    PELVIS_HEIGHT_MAX_METERS,
                                                    pelvisHeightSigma);
      transform.getTranslation().setZ(pelvisHeight);

      actionDefinition.setTrajectoryDuration(randomDuration());
      actionDefinition.modify();
   }

   private void restorePelvisDefaults(PelvisActionState pelvisAction)
   {
      var actionDefinition = pelvisAction.getDefinition();
      actionDefinition.getPelvisToParentTransform().getValueAndModify().set(defaultPelvisTransform);
      actionDefinition.setTrajectoryDuration(defaultPelvisDuration);
      actionDefinition.modify();
   }

   private int[] sampleWholeBodyMask()
   {
      int[] mask = {0, 0, 0, 0}; // [leftArm, rightArm, torso, pelvis]
      mask[1] = random.nextDouble() < definition.getProbabilityRightArmEnabled() ? 1 : 0;
      mask[0] = random.nextDouble() < definition.getProbabilityLeftArmEnabled() ? 1 : 0;
      mask[3] = random.nextDouble() < definition.getProbabilityPelvisEnabled() ? 1 : 0;
      mask[2] = random.nextDouble() < definition.getProbabilityTorsoEnabled() ? 1 : 0;
      return mask;
   }

   private double randomDuration()
   {
      return DURATION_MIN_SECONDS + random.nextDouble() * (DURATION_MAX_SECONDS - DURATION_MIN_SECONDS);
   }

   private double randomArmDuration()
   {
      double armDurationReferenceDistance = 0.5;
      double armDurationSigma = computeGaussianSigmaForReferenceDistance(armDurationReferenceDistance, CENTERED_GAUSSIAN_TARGET_PROBABILITY);
      return sampleTruncatedGaussian(ARM_DURATION_MEAN_SECONDS, ARM_DURATION_MIN_SECONDS, ARM_DURATION_MAX_SECONDS, armDurationSigma);
   }

   private static double clamp(double value, double min, double max)
   {
      return Math.max(min, Math.min(max, value));
   }

   /**
    * Samples a Gaussian distribution centered at {@code mean}, truncated to [{@code min}, {@code max}].
    */
   private double sampleTruncatedGaussian(double mean, double min, double max, double stdDev)
   {
      if (min > max)
      {
         double tmp = min;
         min = max;
         max = tmp;
      }

      if (max - min < 1.0e-12)
         return min;

      double clampedMean = clamp(mean, min, max);
      double safeStdDev = Math.max(stdDev, 1.0e-6);

      for (int i = 0; i < 64; i++)
      {
         double sample = clampedMean + random.nextGaussian() * safeStdDev;
         if (sample >= min && sample <= max)
            return sample;
      }

      return clamp(clampedMean, min, max);
   }

   /**
    * Computes Gaussian sigma so that:
    * P(|x - mean| <= referenceDistance) = targetProbability.
    */
   private static double computeGaussianSigmaForReferenceDistance(double referenceDistance, double targetProbability)
   {
      double safeReferenceDistance = Math.max(Math.abs(referenceDistance), 1.0e-6);
      double safeTargetProbability = clamp(targetProbability, 1.0e-6, 1.0 - 1.0e-6);
      double cumulative = 0.5 * (1.0 + safeTargetProbability);
      double zScore = inverseStandardNormalCdf(cumulative);
      return safeReferenceDistance / Math.max(Math.abs(zScore), 1.0e-6);
   }

   /**
    * Approximation from Peter J. Acklam's inverse-normal algorithm.
    */
   private static double inverseStandardNormalCdf(double p)
   {
      double clampedP = clamp(p, 1.0e-12, 1.0 - 1.0e-12);

      double[] a = {-3.969683028665376e+01, 2.209460984245205e+02, -2.759285104469687e+02, 1.383577518672690e+02, -3.066479806614716e+01, 2.506628277459239e+00};
      double[] b = {-5.447609879822406e+01, 1.615858368580409e+02, -1.556989798598866e+02, 6.680131188771972e+01, -1.328068155288572e+01};
      double[] c = {-7.784894002430293e-03, -3.223964580411365e-01, -2.400758277161838e+00, -2.549732539343734e+00, 4.374664141464968e+00, 2.938163982698783e+00};
      double[] d = {7.784695709041462e-03, 3.224671290700398e-01, 2.445134137142996e+00, 3.754408661907416e+00};

      double pLow = 0.02425;
      double pHigh = 1.0 - pLow;

      if (clampedP < pLow)
      {
         double q = Math.sqrt(-2.0 * Math.log(clampedP));
         return (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5])
                / ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0);
      }
      if (clampedP <= pHigh)
      {
         double q = clampedP - 0.5;
         double r = q * q;
         return (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]) * q
                / (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1.0);
      }

      double q = Math.sqrt(-2.0 * Math.log(1.0 - clampedP));
      return -(((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5])
             / ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0);
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
      statusMessage.getAvailableBehaviors().resetQuick();
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
