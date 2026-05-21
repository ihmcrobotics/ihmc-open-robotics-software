package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.BehaviorTreeSceneObjectDefinitionMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.actions.SceneActionDefinition.SceneActionType;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneCompositeFrameExecutor;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneDoorFrameExecutor;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneDoorPanelExecutor;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectExecutor;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectState;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectType;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseInstantDetection;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseObject;
import us.ihmc.perception.detections.yolo.SyncedYOLOv8ModelParameters;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor;
import us.ihmc.perception.detections.yolo.YOLOv8InstantDetection;
import us.ihmc.tools.Timer;

public class SceneActionExecutor extends ActionNodeExecutor<SceneActionState, SceneActionDefinition>
{
   private final Timer timer = new Timer();
   private final Throttler throttler = new Throttler().setFrequency(1.0);
   private final Point3D cameraPosition = new Point3D();
   private final Vector3D detectionToCamera = new Vector3D();
   private boolean printDebug = false;

   public SceneActionExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new SceneActionState(id, rootNode.getState()), rootNode);
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

      boolean isFreeze = definition.getSceneActionType().getValue() == SceneActionType.FREEZE_OBJECT;
      boolean isDelete = definition.getSceneActionType().getValue() == SceneActionType.DELETE_OBJECT;
      if (definition.getSceneActionType().getValue() == SceneActionType.CLEAR_SCENE)
      {
         scene.removeAllObjects();
         state.setIsExecuting(false);
      }
      else if (definition.getSceneActionType().getValue() == SceneActionType.FREEZE_SCENE)
      {
         for (BehaviorTreeSceneObjectState object : scene.getObjects())
            object.freeze();
         state.setIsExecuting(false);
      }
      else if (definition.getSceneActionType().getValue() == SceneActionType.CONFIGURE_PERSISTENT_DETECTIONS)
      {
         scene.setPoseFilterAlpha(definition.getPoseFilterAlpha());
         scene.setAcceptanceConfidence(definition.getAcceptanceConfidence());
         scene.setStabilityFrequency(definition.getStabilityFrequency());
         scene.setHistoryDuration(definition.getHistoryDuration());
         state.setIsExecuting(false);
      }
      else if (definition.getSceneActionType().getValue() == SceneActionType.CONFIGURE_YOLO)
      {
         YOLOv8DetectionExecutor yolo = scene.getYOLO();
         yolo.disableAllModels();
         for (int i = 0; i < definition.getEnabledYoloModels().getSize(); i++)
         {
            SyncedYOLOv8ModelParameters definitionParameters
                  = definition.getSyncableYOLOModelParameters()[definition.getEnabledYoloModels().getValueReadOnly(i)];
            yolo.enableModel(definitionParameters.getModelName());
            yolo.getParameters().getModelParameters().get(definitionParameters.getModelName()).set(definitionParameters);
         }
         state.setIsExecuting(false);
      }
      else if (isFreeze || isDelete)
         freezeOrDelete(isFreeze);
      else // Setup object
         setupObject();
   }

   private void freezeOrDelete(boolean isFreeze)
   {
      BehaviorTreeSceneObjectState matchedObject = null;
      for (BehaviorTreeSceneObjectState object : scene.getObjects())
      {
         if (definition.getSceneObjectDefinition().getObjectType() == BehaviorTreeSceneObjectType.DOOR_PANEL
             && object instanceof BehaviorTreeSceneDoorPanelExecutor)
            matchedObject = object;
         else if (definition.getSceneObjectDefinition().getObjectType() == BehaviorTreeSceneObjectType.DOOR_FRAME
                  && object instanceof BehaviorTreeSceneDoorFrameExecutor)
            matchedObject = object;
         else if (definition.getSceneObjectDefinition().getObjectType() == BehaviorTreeSceneObjectType.FOUNDATION_POSE
                  && object.getObjectType() == BehaviorTreeSceneObjectType.FOUNDATION_POSE
                  && object.getFoundationPoseObjectType() == definition.getSceneObjectDefinition().getFoundationPoseObjectType())
            matchedObject = object;
         else if (definition.getSceneObjectDefinition().getObjectType() == BehaviorTreeSceneObjectType.YOLO_ONLY
                  && object.getYoloClassName().equals(definition.getSceneObjectDefinition().getYoloClassName()))
            matchedObject = object;
      }

      if (matchedObject == null)
      {
         if (isFreeze)
            state.getLogger().error("Failed to find a suitable object to freeze: %s".formatted(definition.getSceneObjectDefinition().getName()));
         else
            state.getLogger().error("Failed to find a suitable object to delete: %s".formatted(definition.getSceneObjectDefinition().getName()));
      }
      else
      {
         if (isFreeze)
         {
            state.getLogger().info("Freezing object: %s".formatted(matchedObject.getName()));
            matchedObject.freeze();
         }
         else
         {
            state.getLogger().info("Deleting object: %s".formatted(matchedObject.getName()));
            scene.removeObject(matchedObject);
         }
      }

      state.setFailed(isFreeze && matchedObject == null); // Don't fail if deleting and object was not found
      state.setIsExecuting(false);
   }

   private void setupObject()
   {
      if (rootNode.getState().getPreviewModeEnabled())
      {
         state.getLogger().info("Preview mode enabled. Adding nominal object pose for: {}", definition.getSceneObjectDefinition().getName());

         BehaviorTreeSceneObjectState existingObject = null;
         for (BehaviorTreeSceneObjectState object : scene.getObjects())
            if (object.getObjectType() == definition.getSceneObjectDefinition().getObjectType())
            {
               boolean match = definition.getSceneObjectDefinition().getObjectType() == BehaviorTreeSceneObjectType.YOLO_ONLY
                               && object.getYoloClassName().equals(definition.getSceneObjectDefinition().getYoloClassName());
               match |= definition.getSceneObjectDefinition().getObjectType() == BehaviorTreeSceneObjectType.FOUNDATION_POSE
                        && object.getFoundationPoseObjectType() == definition.getSceneObjectDefinition().getFoundationPoseObjectType();
               match |= definition.getSceneObjectDefinition().getObjectType() != BehaviorTreeSceneObjectType.YOLO_ONLY
                        && definition.getSceneObjectDefinition().getObjectType() != BehaviorTreeSceneObjectType.FOUNDATION_POSE;
               if (match)
               {
                  existingObject = object;
                  break;
               }
            }

         RigidBodyTransform nominalWorldPose = new RigidBodyTransform();
         nominalWorldPose.set(scene.findFrameByName("Walking").getTransformToRoot());
         nominalWorldPose.multiply(definition.getNominalObjectPose().getValueReadOnly());

         if (existingObject != null)
            existingObject.setTransformToWorld(nominalWorldPose);
         else
         {
            BehaviorTreeSceneObjectDefinitionMessage message = new BehaviorTreeSceneObjectDefinitionMessage();
            definition.getSceneObjectDefinition().toMessage(message);
            BehaviorTreeSceneObjectState nominalObject = scene.createObject(message);
            nominalObject.setTransformToWorld(nominalWorldPose);
            scene.addObject(nominalObject);
         }
         state.setIsExecuting(false);
         return;
      }

      double timeout = definition.getTimeout();
      if (!timer.isRunning(timeout))
      {
         state.getLogger().error("Timed out after %.1f s without finding a suitable detection.".formatted(timeout));
         state.setFailed(true);
         state.setIsExecuting(false);
         return;
      }

      printDebug = throttler.run();
      cameraPosition.set(syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getExperimentalCameraFrame).getTranslation());

      boolean success = switch (definition.getSceneObjectDefinition().getObjectType())
      {
         case COMPOSITE_FRAME -> setupCompositeFrameDetection();
         case DOOR_PANEL -> setupDoorPanelDetection();
         case DOOR_FRAME -> setupDoorFrameDetection();
         default -> setupSinglePersistentDetection();
      };

      if (success)
         state.setIsExecuting(false);
   }

   private boolean setupCompositeFrameDetection()
   {
      ReferenceFrame frameA = scene.findFrameByName(definition.getSceneObjectDefinition().getCompositeFrameA());
      ReferenceFrame frameB = scene.findFrameByName(definition.getSceneObjectDefinition().getCompositeFrameB());
      if (frameA == null || frameB == null)
      {
         if (printDebug)
            state.getLogger()
                 .warn("Failed to find frames: %s = %s and %s = %s".formatted(
                       definition.getSceneObjectDefinition().getCompositeFrameA(), frameA,
                       definition.getSceneObjectDefinition().getCompositeFrameB(), frameB));
         return false;
      }

      BehaviorTreeSceneCompositeFrameExecutor target = null;
      for (BehaviorTreeSceneObjectState object : scene.getObjects())
      {
         if (object instanceof BehaviorTreeSceneCompositeFrameExecutor compositeFrame
          && compositeFrame.getCompositeFrameName().equals(definition.getSceneObjectDefinition().getCompositeFrameName()))
         {
            target = compositeFrame;
            break;
         }
      }

      if (target != null)
      {
         state.getLogger().info("Updating existing composite frame: {}", definition.getSceneObjectDefinition().getCompositeFrameName());
         target.setCompositeFrameA(definition.getSceneObjectDefinition().getCompositeFrameA());
         target.setCompositeFrameB(definition.getSceneObjectDefinition().getCompositeFrameB());
         target.setCompositeFrameType(definition.getSceneObjectDefinition().getCompositeFrameType());
         target.setCompositeDistance(definition.getSceneObjectDefinition().getCompositeDistance());
         target.unfreeze();
      }
      else
      {
         state.getLogger().info("Creating new composite frame: {}", definition.getSceneObjectDefinition().getCompositeFrameName());

         BehaviorTreeSceneObjectDefinitionMessage message = new BehaviorTreeSceneObjectDefinitionMessage();
         definition.getSceneObjectDefinition().toMessage(message);
         target = (BehaviorTreeSceneCompositeFrameExecutor) scene.createObject(message);
         target.update();
         scene.addObject(target);
      }

      return true;
   }

   private boolean setupDoorPanelDetection()
   {
      // First, find the closest door opening mechanism (lever, knob, push bar, or pull handle)
      PersistentDetection openingMechanismDetection = null;
      double closestMechanismDistanceSquared = Double.MAX_VALUE;

      for (PersistentDetection detection : scene.getPersistentDetections())
      {
         // Only consider YOLOv8 detections
         if (!(detection.getMostRecentDetection() instanceof YOLOv8InstantDetection yoloDetection))
            continue;

         String className = yoloDetection.getDetectedObjectClass();
         boolean isOpeningMechanism = className.equals("door_lever")
                                      || className.equals("door_knob")
                                      || className.equals("door_push_bar")
                                      || className.equals("door_pull_handle");

         if (!isOpeningMechanism)
            continue;

         int minimumHistorySize = definition.getMinimumHistorySize();
         if (detection.getHistorySize() < minimumHistorySize)
         {
            if (printDebug)
               state.getLogger().warn("Door opening mechanism has history size {} but need at least {}",
                                      detection.getHistorySize(), minimumHistorySize);
            continue;
         }

         detectionToCamera.set(detection.getFilteredTransform().getTranslation());
         detectionToCamera.sub(cameraPosition);
         double distanceSquared = detectionToCamera.normSquared();

         if (distanceSquared < closestMechanismDistanceSquared)
         {
            closestMechanismDistanceSquared = distanceSquared;
            openingMechanismDetection = detection;
         }
      }

      if (openingMechanismDetection == null)
      {
         if (printDebug)
            state.getLogger().warn("No suitable door opening mechanism found (door_lever, door_knob, door_push_bar, or door_pull_handle).");
         return false;
      }

      state.getLogger().info("Found door opening mechanism: {} with history size: {}",
                             openingMechanismDetection.getMostRecentDetection().getDetectedObjectClass(),
                             openingMechanismDetection.getHistorySize());

      // Now find the closest door_panel detection to the opening mechanism
      PersistentDetection doorPanelDetection = null;
      double closestPanelToMechanismDistanceSquared = Double.MAX_VALUE;
      Point3D openingMechanismPosition = new Point3D(openingMechanismDetection.getFilteredTransform().getTranslation());
      Vector3D panelToMechanism = new Vector3D();

      for (PersistentDetection detection : scene.getPersistentDetections())
      {
         // Only consider YOLOv8 detections
         if (!(detection.getMostRecentDetection() instanceof YOLOv8InstantDetection yoloDetection))
            continue;

         String className = yoloDetection.getDetectedObjectClass();
         if (!className.equals("door_panel"))
            continue;

         int minimumHistorySize = definition.getMinimumHistorySize();
         if (detection.getHistorySize() < minimumHistorySize)
         {
            if (printDebug)
               state.getLogger().warn("Door panel has history size {} but need at least {}",
                                      detection.getHistorySize(), minimumHistorySize);
            continue;
         }

         // Find closest panel to opening mechanism
         panelToMechanism.set(detection.getFilteredTransform().getTranslation());
         panelToMechanism.sub(openingMechanismPosition);
         double distanceToMechanismSquared = panelToMechanism.normSquared();

         if (distanceToMechanismSquared < closestPanelToMechanismDistanceSquared)
         {
            closestPanelToMechanismDistanceSquared = distanceToMechanismSquared;
            doorPanelDetection = detection;
         }
      }

      if (doorPanelDetection == null)
      {
         if (printDebug)
            state.getLogger().warn("No suitable door_panel found.");
         return false;
      }

      // Check that the closest panel is within 2 meters of the opening mechanism
      double distanceToMechanism = Math.sqrt(closestPanelToMechanismDistanceSquared);
      if (distanceToMechanism > 2.0)
      {
         if (printDebug)
            state.getLogger().warn("Closest door_panel is %.2f m from opening mechanism, must be within 2.0 m".formatted(distanceToMechanism));
         return false;
      }

      state.getLogger().info("Found door_panel with history size: %d at distance %.2f m from opening mechanism".formatted(
                             doorPanelDetection.getHistorySize(),
                             distanceToMechanism));

      // Check if a door panel scene object already exists
      BehaviorTreeSceneDoorPanelExecutor targetSceneObject = null;
      for (BehaviorTreeSceneObjectState object : scene.getObjects())
      {
         if (object instanceof BehaviorTreeSceneDoorPanelExecutor doorPanelExecutor)
         {
            targetSceneObject = doorPanelExecutor;
            break;
         }
      }

      if (targetSceneObject != null)
      {
         state.getLogger().info("Updating existing door panel scene object");
         targetSceneObject.setPersistentDetection(openingMechanismDetection);
         targetSceneObject.setDoorPanelPersistentDetection(doorPanelDetection);
         targetSceneObject.unfreeze();
      }
      else
      {
         state.getLogger().info("Creating new door panel scene object");

         BehaviorTreeSceneObjectDefinitionMessage message = new BehaviorTreeSceneObjectDefinitionMessage();
         definition.getSceneObjectDefinition().toMessage(message);
         targetSceneObject = (BehaviorTreeSceneDoorPanelExecutor) scene.createObject(message);
         targetSceneObject.setPersistentDetection(openingMechanismDetection);
         targetSceneObject.setDoorPanelPersistentDetection(doorPanelDetection);
         targetSceneObject.update();
         scene.addObject(targetSceneObject);
      }

      return true;
   }

   private boolean setupDoorFrameDetection()
   {
      // First, find a stable door panel
      BehaviorTreeSceneDoorPanelExecutor doorPanelSceneObject = null;
      for (BehaviorTreeSceneObjectState object : scene.getObjects())
      {
         if (object instanceof BehaviorTreeSceneDoorPanelExecutor doorPanelExecutor && doorPanelExecutor.isStable())
         {
            doorPanelSceneObject = doorPanelExecutor;
            break;
         }
      }

      if (doorPanelSceneObject == null)
      {
         if (printDebug)
            state.getLogger().warn("No suitable door panel scene object found.");
         return false;
      }

      state.getLogger().info("Found door panel scene object: {} ({})", doorPanelSceneObject.getName(), doorPanelSceneObject.getID());

      // Check if a frame scene object already exists
      BehaviorTreeSceneDoorFrameExecutor frameSceneObject = null;
      for (BehaviorTreeSceneObjectState object : scene.getObjects())
      {
         if (object instanceof BehaviorTreeSceneDoorFrameExecutor doorPanelExecutor)
         {
            frameSceneObject = doorPanelExecutor;
            break;
         }
      }

      // add object
      if (frameSceneObject != null)
      {
         state.getLogger().info("Updating existing door frame scene object");
         frameSceneObject.setPersistentDetection(doorPanelSceneObject.getDoorPanelPersistentDetection());
         frameSceneObject.setMinPostPoints(definition.getSceneObjectDefinition().getMinPostPoints());
         frameSceneObject.setMinRecessPoints(definition.getSceneObjectDefinition().getMinRecessPoints());
         frameSceneObject.unfreeze();
      }
      else
      {
         state.getLogger().info("Creating new door frame scene object");

         BehaviorTreeSceneObjectDefinitionMessage message = new BehaviorTreeSceneObjectDefinitionMessage();
         definition.getSceneObjectDefinition().toMessage(message);
         frameSceneObject = (BehaviorTreeSceneDoorFrameExecutor) scene.createObject(message);
         frameSceneObject.setDoorPanel(doorPanelSceneObject);
         frameSceneObject.update();
         scene.addObject(frameSceneObject);
      }

      return true;
   }

   private boolean setupSinglePersistentDetection()
   {
      // Find a close stable detection
      PersistentDetection bestDetection = null;
      double closestDistanceSquared = Double.MAX_VALUE;

      for (PersistentDetection detection : scene.getPersistentDetections())
      {
         if (detection.getMostRecentDetection() instanceof IsaacROSFoundationPoseInstantDetection fpDetection)
         {
            if (!(definition.getSceneObjectDefinition().getObjectType() == BehaviorTreeSceneObjectType.FOUNDATION_POSE))
            {
               if (printDebug)
                  state.getLogger().warn("Need %s but found FoundationPose name: %s".formatted(definition.getSceneObjectDefinition().getObjectType().name(),
                                                                                               fpDetection.getObject().name()));

               continue;
            }

            IsaacROSFoundationPoseObject desiredFPType = definition.getSceneObjectDefinition().getFoundationPoseObjectType();
            if (fpDetection.getObject() != desiredFPType)
            {
               if (printDebug)
                  state.getLogger().warn("Need FP object class type {} but found {}", desiredFPType.name(), fpDetection.getObject().name());

               continue;
            }
         }
         else if (detection.getMostRecentDetection() instanceof YOLOv8InstantDetection yoloDetection)
         {
            if (!(definition.getSceneObjectDefinition().getObjectType() == BehaviorTreeSceneObjectType.YOLO_ONLY))
            {
               if (printDebug)
                  state.getLogger().warn("Need %s but found YOLOv8 name: %s"
                                               .formatted(definition.getSceneObjectDefinition().getObjectType().name(),
                                                          detection.getMostRecentDetection().getDetectedObjectClass()));

               continue;
            }

            String desiredYOLOClass = definition.getSceneObjectDefinition().getYoloClassName();
            if (!yoloDetection.getDetectedObjectClass().equals(desiredYOLOClass))
            {
               if (printDebug)
                  state.getLogger().warn("Need YOLO class type {} but found {}", desiredYOLOClass, yoloDetection.getDetectedObjectClass());

               continue;
            }
         }

         int minimumHistorySize = definition.getMinimumHistorySize();
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
         return false;
      }

      state.getLogger().info("Found persistent detection with history size: {}", bestDetection.getHistorySize());

      // Check if a scene object of this type already exists
      BehaviorTreeSceneObjectExecutor targetSceneObject = null;
      for (BehaviorTreeSceneObjectState object : scene.getObjects())
      {
         if (object.getObjectType() == definition.getSceneObjectDefinition().getObjectType())
         {
            boolean match = definition.getSceneObjectDefinition().getObjectType() == BehaviorTreeSceneObjectType.YOLO_ONLY
                            && object.getYoloClassName().equals(definition.getSceneObjectDefinition().getYoloClassName());
            match |= definition.getSceneObjectDefinition().getObjectType() == BehaviorTreeSceneObjectType.FOUNDATION_POSE
                     && object.getFoundationPoseObjectType() == definition.getSceneObjectDefinition().getFoundationPoseObjectType();
            if (match)
            {
               targetSceneObject = (BehaviorTreeSceneObjectExecutor) object;
               break;
            }
         }
      }

      if (targetSceneObject != null)
      {
         state.getLogger().info("Updating existing scene object for type: {}", definition.getSceneObjectDefinition().getName());
         targetSceneObject.unfreeze();
         targetSceneObject.setPersistentDetection(bestDetection);
      }
      else
      {
         state.getLogger().info("Creating new scene object for type: {}", definition.getSceneObjectDefinition().getName());

         BehaviorTreeSceneObjectDefinitionMessage message = new BehaviorTreeSceneObjectDefinitionMessage();
         definition.getSceneObjectDefinition().toMessage(message);
         targetSceneObject = (BehaviorTreeSceneObjectExecutor) scene.createObject(message);
         targetSceneObject.setPersistentDetection(bestDetection);
         targetSceneObject.update();
         scene.addObject(targetSceneObject);
      }

      return true;
   }
}
