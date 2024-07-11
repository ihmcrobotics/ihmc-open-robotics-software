package us.ihmc.perception.sceneGraph.rigidBody.doors;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.YOLOv8.YOLOv8InstantDetection;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeAddition;
import us.ihmc.perception.sceneGraph.rigidBody.PredefinedRigidBodySceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.doors.components.DoorOpeningMechanism;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.perception.sceneGraph.rigidBody.doors.DoorSceneNodeDefinitions.*;

public final class DoorNodeTools
{
   public static final String DOOR_HELPER_NODE_NAME_PREFIX = "doorStaticHandle";

   private static final List<String> doorOpeningMechanismDetectionNames = new ArrayList<>();
   private static final String doorPanelDetectionName = "YOLODoorPanel";
   private static final List<String> doorComponentDetectionNames = new ArrayList<>();

   static
   {
      doorOpeningMechanismDetectionNames.add("YOLODoorLever");
      doorOpeningMechanismDetectionNames.add("YOLODoorKnob");
      doorOpeningMechanismDetectionNames.add("YOLOPushBar");
      doorOpeningMechanismDetectionNames.add("YOLOPullHandle");

      doorComponentDetectionNames.addAll(doorOpeningMechanismDetectionNames);
      doorComponentDetectionNames.add(doorPanelDetectionName);
   }

   public static boolean detectionIsDoorComponent(PersistentDetection detection)
   {
      Class<?> detectionClass = detection.getInstantDetectionClass();
      if (detectionClass.equals(YOLOv8InstantDetection.class))
      {
         return doorComponentDetectionNames.contains(detection.getDetectedObjectName());
      }
      return false;
   }

   public static boolean detectionIsDoorOpeningMechanism(PersistentDetection detection)
   {
      Class<?> detectionClass = detection.getInstantDetectionClass();
      if (detectionClass.equals(YOLOv8InstantDetection.class))
      {
         return doorOpeningMechanismDetectionNames.contains(detection.getDetectedObjectName());
      }
      return false;
   }

   public static boolean detectionIsDoorPanel(PersistentDetection detection)
   {
      Class<?> detectionClass = detection.getInstantDetectionClass();
      if (detectionClass.equals(YOLOv8InstantDetection.class))
      {
         return doorPanelDetectionName.contains(detection.getDetectedObjectName());
      }
      return false;
   }

   public static SceneNode addOpeningMechanismHelperNode(DoorNode doorNode,
                                                         DoorOpeningMechanism openingMechanism,
                                                         SceneGraph sceneGraph,
                                                         SceneGraphModificationQueue modificationQueue)
   {
      String visualModelPath = DOOR_LEVER_HANDLE_VISUAL_MODEL_FILE_PATH;
      RigidBodyTransform modelToPoseTransform = new RigidBodyTransform();
      switch (openingMechanism.getType())
      {
         case LEVER_HANDLE, UNKNOWN ->
         {
            visualModelPath = DOOR_LEVER_HANDLE_VISUAL_MODEL_FILE_PATH;
            modelToPoseTransform.set(DOOR_HANDLE_TO_YOLO_VISUAL_MODEL_TRANSFORM);
         }
         case KNOB ->
         {
            visualModelPath = DOOR_KNOB_VISUAL_MODEL_FILE_PATH;
            modelToPoseTransform.set(DOOR_KNOB_TO_YOLO_VISUAL_MODEL_TRANSFORM);
         }
         case PUSH_BAR ->
         {
            visualModelPath = DOOR_EMERGENCY_BAR_VISUAL_MODEL_FILE_PATH;
            modelToPoseTransform.set(DOOR_PUSH_BAR_TO_YOLO_VISUAL_MODEL_TRANSFORM);
         }
         case PULL_HANDLE ->
         {
            visualModelPath = DOOR_PULL_HANDLE_VISUAL_MODEL_FILE_PATH;
            modelToPoseTransform.set(DOOR_HANDLE_TO_YOLO_VISUAL_MODEL_TRANSFORM);
         }
      }

      String nodeName = DOOR_HELPER_NODE_NAME_PREFIX + openingMechanism.getColloquialName();
      SceneNode doorHelperNode = new PredefinedRigidBodySceneNode(sceneGraph.getNextID().getAndIncrement(),
                                                                  nodeName,
                                                                  sceneGraph.getIDToNodeMap(),
                                                                  doorNode.getID(),
                                                                  new RigidBodyTransform(),
                                                                  visualModelPath,
                                                                  modelToPoseTransform,
                                                                  sceneGraph.getCRDTInfo());
      modificationQueue.accept(new SceneGraphNodeAddition(doorHelperNode, sceneGraph.getRootNode()));
      return doorHelperNode;
   }
}
