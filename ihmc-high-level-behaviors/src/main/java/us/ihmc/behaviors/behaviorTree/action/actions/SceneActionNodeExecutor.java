package us.ihmc.behaviors.behaviorTree.action.actions;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectExecutor;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectState;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseObject;
import us.ihmc.tools.Timer;

public class SceneActionNodeExecutor extends ActionNodeExecutor<SceneActionNodeState, SceneActionNodeDefinition>
{
   private final Timer timer = new Timer();

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
      double timeout = 5.0;
      if (!timer.isRunning(timeout))
      {
         state.setFailed(true);
         state.setIsExecuting(false);
         return;
      }

      // Find a close stable detection
      PersistentDetection bestDetection = null;
      double closestDistanceSquared = Double.MAX_VALUE;
      IsaacROSFoundationPoseObject objectType = definition.getObjectType();
      for (PersistentDetection detection : scene.getPersistentDetections())
      {
         if (!detection.getDetectedObjectClass().equals(objectType.yoloClass))
            continue;

         int minimumHistorySize = 5;
         if (detection.getHistorySize() < minimumHistorySize)
            continue;

         double distanceSquared = detection.getFilteredTransformToCamera().getTranslation().normSquared();

         if (distanceSquared < closestDistanceSquared)
         {
            closestDistanceSquared = distanceSquared;
            bestDetection = detection;
         }
      }

      if (bestDetection == null)
      {
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
