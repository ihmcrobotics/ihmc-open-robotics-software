package us.ihmc.behaviors.behaviorTree.action.actions;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectExecutor;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectState;
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

      state.getLogger().info("Executing scene action for object type: {}", definition.getObjectType().name());

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
      IsaacROSFoundationPoseObject objectType = definition.getObjectType();
      cameraPosition.set(syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getExperimentalCameraFrame).getTranslation());
      for (PersistentDetection detection : scene.getPersistentDetections())
      {
         if (detection.getMostRecentDetection() instanceof IsaacROSFoundationPoseInstantDetection fpDetection)
         {
            if (!definition.getUseFoundationPose())
            {
               if (printDebug)
                  state.getLogger().warn("Need YOLO but found FoundationPose name: %s".formatted(fpDetection.getObject().name()));

               continue;
            }

            if (fpDetection.getObject() != objectType)
            {
               if (printDebug)
                  state.getLogger().warn("Need object class type {} but found {}", objectType.name(), fpDetection.getObject().name());

               continue;
            }
         }
         else if (detection.getMostRecentDetection() instanceof YOLOv8InstantDetection yoloDetection)
         {
            if (definition.getUseFoundationPose())
            {
               if (printDebug)
                  state.getLogger().warn("Need FoundationPose but found YOLOv8 name: %s"
                                               .formatted(detection.getMostRecentDetection().getDetectedObjectClass()));

               continue;
            }

            if (!yoloDetection.getDetectedObjectClass().equals(objectType.yoloClass))
            {
               if (printDebug)
                  state.getLogger().warn("Need object class type {} but found {}", objectType.yoloClass, yoloDetection.getDetectedObjectClass());

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
         if (object.getObjectType() == objectType)
         {
            targetSceneObject = (BehaviorTreeSceneObjectExecutor) object;
            break;
         }
      }

      if (targetSceneObject != null)
      {
         state.getLogger().info("Updating existing scene object for type: {}", objectType.titleCaseName);
         targetSceneObject.setPersistentDetection(bestDetection);
      }
      else
      {
         state.getLogger().info("Creating new scene object for type: {}", objectType.titleCaseName);

         targetSceneObject = (BehaviorTreeSceneObjectExecutor) scene.createObject(objectType);
         targetSceneObject.setPersistentDetection(bestDetection);
         scene.getObjects().add(targetSceneObject);
         scene.getObjectsModifiable().modify();
      }

      state.setIsExecuting(false);
   }
}
