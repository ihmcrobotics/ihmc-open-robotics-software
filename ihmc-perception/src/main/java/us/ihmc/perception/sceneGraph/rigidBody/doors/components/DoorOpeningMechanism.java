package us.ihmc.perception.sceneGraph.rigidBody.doors.components;

import com.google.common.base.CaseFormat;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.YOLOv8.YOLOv8DetectionClass;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorNode.DoorSide;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorSceneNodeDefinitions;

import javax.annotation.Nullable;

/**
 * Represents hardware on a door which you have to interact with to open it, could be a knob, lever, deadbolt lock, etc...
 */
public class DoorOpeningMechanism
{
   private final DoorOpeningMechanismType type;
   private final Pose3D graspPose = new Pose3D();
   private final DoorSide doorSide;

   public DoorOpeningMechanism(DoorSide doorSide, DoorOpeningMechanismType type)
   {
      this.type = type;
      this.doorSide = doorSide;
   }

   public DoorOpeningMechanism(DoorSide doorSide, YOLOv8DetectionClass yolOv8DetectionClass)
   {
      switch (yolOv8DetectionClass)
      {
         case DOOR_LEVER -> type = DoorOpeningMechanismType.LEVER_HANDLE;
         case DOOR_KNOB -> type = DoorOpeningMechanismType.KNOB;
         case DOOR_PULL_HANDLE -> type = DoorOpeningMechanismType.PULL_HANDLE;
         case DOOR_PUSH_BAR -> type = DoorOpeningMechanismType.PUSH_BAR;
         default -> type = DoorOpeningMechanismType.UNKNOWN;
      }
      this.doorSide = doorSide;
   }

   public DoorOpeningMechanismType getType()
   {
      return type;
   }

   public Pose3D getGraspPose()
   {
      return graspPose;
   }

   public DoorSide getDoorSide()
   {
      return doorSide;
   }

   public String getColloquialName()
   {
      // E.g. pushLeverHandle, pullKnob, pushPushBar?, pullPullHandle?
      return doorSide.name().toLowerCase() + CaseFormat.UPPER_UNDERSCORE.to(CaseFormat.UPPER_CAMEL, type.name());
   }

   @Nullable
   public String getVisualModelPath()
   {
      switch (type)
      {
         case LEVER_HANDLE ->
         {
            return DoorSceneNodeDefinitions.DOOR_LEVER_HANDLE_VISUAL_MODEL_FILE_PATH;
         }
         case KNOB ->
         {
            return DoorSceneNodeDefinitions.DOOR_KNOB_VISUAL_MODEL_FILE_PATH;
         }
         case PUSH_BAR ->
         {
            return DoorSceneNodeDefinitions.DOOR_EMERGENCY_BAR_VISUAL_MODEL_FILE_PATH;
         }
         case PULL_HANDLE ->
         {
            return DoorSceneNodeDefinitions.DOOR_PULL_HANDLE_VISUAL_MODEL_FILE_PATH;
         }
      }
      return null;
   }

   public RigidBodyTransform getVisualModelTransform()
   {
      switch (type)
      {
         case LEVER_HANDLE ->
         {
            // TODO: DOORNODES, check this
            if (doorSide == DoorSide.PUSH)
            {
               return DoorSceneNodeDefinitions.LEFT_DOOR_LEVER_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM;
            }
            else
            {
               return DoorSceneNodeDefinitions.RIGHT_DOOR_LEVER_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM;
            }
         }
         case KNOB ->
         {
            return DoorSceneNodeDefinitions.DOOR_KNOB_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM;
         }
         case PUSH_BAR ->
         {
            if (doorSide == DoorSide.PUSH)
            {
               return DoorSceneNodeDefinitions.LEFT_DOOR_EMERGENCY_BAR_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM;
            }
            else
            {
               return DoorSceneNodeDefinitions.RIGHT_DOOR_EMERGENCY_BAR_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM;
            }
         }
         case PULL_HANDLE ->
         {
            return DoorSceneNodeDefinitions.DOOR_PULL_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM;
         }
      }
      return new RigidBodyTransform();
   }

   public enum DoorOpeningMechanismType
   {
      UNKNOWN, LEVER_HANDLE, KNOB, PUSH_BAR, PULL_HANDLE;

      public static DoorOpeningMechanismType fromByte(byte b)
      {
         return DoorOpeningMechanismType.values()[b];
      }
   }
}
