package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage;
import behavior_msgs.msg.dds.SceneActionNodeDefinitionMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectDefinition;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectExecutor;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectState;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectType;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseInstantDetection;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseObject;
import us.ihmc.perception.detections.yolo.YOLOv8InstantDetection;
import us.ihmc.tools.Timer;

public class SceneActionNodeExecutor extends ActionNodeExecutor<SceneActionNodeState, SceneActionNodeDefinition>
{
   private final Timer timer = new Timer();
   private final Throttler throttler = new Throttler().setFrequency(1.0);
   private final Point3D cameraPosition = new Point3D();
   private final Vector3D detectionToCamera = new Vector3D();

   public SceneActionNodeExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new SceneActionNodeState(id, rootNode.getState()), rootNode);
   }

   @Override
   public void update()
   {
      super.update();
   }

   @Override
   public void triggerExecution()
   {
      super.triggerExecution();

      state.getLogger().info("Executing scene action for object type: {}", definition.getName());

      timer.reset();
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      state.setElapsedExecutionTime(timer.getElapsedTime());

      double timeout = 5.0;
      if (!timer.isRunning(timeout))
      {
         state.getLogger().error("Timed out after %.1f s without finding a suitable detection.".formatted(timeout));
         state.setFailed(true);
         state.setIsExecuting(false);
         return;
      }

      boolean printDebug = throttler.run();

      // Find a close stable detection
      PersistentDetection bestDetection = null;
      double closestDistanceSquared = Double.MAX_VALUE;
      BehaviorTreeSceneObjectDefinition desiredObjectDefinition = definition.getSceneObjectDefinition();
      cameraPosition.set(syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getExperimentalCameraFrame).getTranslation());
      for (PersistentDetection detection : scene.getPersistentDetections())
      {
         if (detection.getMostRecentDetection() instanceof IsaacROSFoundationPoseInstantDetection fpDetection)
         {
            if (!(desiredObjectDefinition.getObjectType() == BehaviorTreeSceneObjectType.FOUNDATION_POSE))
            {
               if (printDebug)
                  state.getLogger().warn("Need %s but found FoundationPose name: %s".formatted(desiredObjectDefinition.getObjectType().name(),
                                                                                               fpDetection.getObject().name()));

               continue;
            }

            IsaacROSFoundationPoseObject desiredFPType = desiredObjectDefinition.getFoundationPoseObjectType();
            if (fpDetection.getObject() != desiredFPType)
            {
               if (printDebug)
                  state.getLogger().warn("Need FP object class type {} but found {}", desiredFPType.name(), fpDetection.getObject().name());

               continue;
            }
         }
         else if (desiredObjectDefinition.getObjectType() == BehaviorTreeSceneObjectType.DOOR_PANEL)
         {
            if (printDebug)
               state.getLogger().warn("Door panel not yet implemented.");

            continue;
         }
         else if (detection.getMostRecentDetection() instanceof YOLOv8InstantDetection yoloDetection)
         {
            if (!(desiredObjectDefinition.getObjectType() == BehaviorTreeSceneObjectType.YOLO_ONLY))
            {
               if (printDebug)
                  state.getLogger().warn("Need %s but found YOLOv8 name: %s"
                                               .formatted(desiredObjectDefinition.getObjectType().name(),
                                                          detection.getMostRecentDetection().getDetectedObjectClass()));

               continue;
            }

            String desiredYOLOClass = desiredObjectDefinition.getYoloClassName();
            if (!yoloDetection.getDetectedObjectClass().equals(desiredYOLOClass))
            {
               if (printDebug)
                  state.getLogger().warn("Need YOLO class type {} but found {}", desiredYOLOClass, yoloDetection.getDetectedObjectClass());

               continue;
            }
         }

         int minimumHistorySize = 5;
         if (detection.getHistorySize() < minimumHistorySize)
         {
            if (printDebug)
               state.getLogger().warn("Need history size of at least {} but found {}", minimumHistorySize, detection.getHistorySize());

            continue;
         }

         detectionToCamera.set(detection.getFilteredTransform().getTranslation());
         detectionToCamera.sub(cameraPosition);
         double distanceSquared = detectionToCamera.normSquared();

         if (distanceSquared < closestDistanceSquared)
         {
            closestDistanceSquared = distanceSquared;
            bestDetection = detection;
         }
      }

      if (bestDetection == null)
      {
         if (printDebug)
            state.getLogger().warn("No suitable persistent detection found. There are currently {} persistent detections.",
                                    scene.getPersistentDetections().size());
         return;
      }

      state.getLogger().info("Found persistent detection with history size: {}", bestDetection.getHistorySize());

      // Check if a scene object of this type already exists
      BehaviorTreeSceneObjectExecutor targetSceneObject = null;
      for (BehaviorTreeSceneObjectState object : scene.getObjects())
      {
         if (object.getObjectType() == desiredObjectDefinition.getObjectType())
         {
            boolean match = desiredObjectDefinition.getObjectType() == BehaviorTreeSceneObjectType.YOLO_ONLY
                            && object.getYoloClassName().equals(desiredObjectDefinition.getYoloClassName());
            match |= desiredObjectDefinition.getObjectType() == BehaviorTreeSceneObjectType.FOUNDATION_POSE
                     && object.getFoundationPoseObjectType() == desiredObjectDefinition.getFoundationPoseObjectType();
            if (match)
            {
               targetSceneObject = (BehaviorTreeSceneObjectExecutor) object;
               break;
            }
         }
      }

      if (targetSceneObject != null)
      {
         state.getLogger().info("Updating existing scene object for type: {}", desiredObjectDefinition.getName());
         targetSceneObject.setPersistentDetection(bestDetection);
      }
      else
      {
         state.getLogger().info("Creating new scene object for type: {}", desiredObjectDefinition.getName());

         BehaviorTreeSceneObjectDefinitionMessage message = new BehaviorTreeSceneObjectDefinitionMessage();
         desiredObjectDefinition.toMessage(message);
         targetSceneObject = (BehaviorTreeSceneObjectExecutor) scene.createObject(message);
         targetSceneObject.setPersistentDetection(bestDetection);
         scene.getObjects().add(targetSceneObject);
         scene.getObjectsModifiable().modify();
      }

      state.setIsExecuting(false);
   }
}
