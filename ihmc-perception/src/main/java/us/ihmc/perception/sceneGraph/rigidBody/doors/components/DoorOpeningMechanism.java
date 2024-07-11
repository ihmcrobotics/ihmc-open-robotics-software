package us.ihmc.perception.sceneGraph.rigidBody.doors.components;

import com.google.common.base.CaseFormat;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.YOLOv8.YOLOv8DetectionClass;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorNode.DoorSide;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorSceneNodeDefinitions;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;

import javax.annotation.Nullable;
import java.util.UUID;

/**
 * Represents hardware on a door which you have to interact with to open it, could be a knob, lever, deadbolt lock, etc...
 */
public class DoorOpeningMechanism
{
   // Shared over ROS2
   private final DoorOpeningMechanismType type;
   private final DoorSide doorSide;
   private final MutableReferenceFrame mechanismFrame = new MutableReferenceFrame();
   private final UUID detectionID; // syncing the detection ID instead of full detection

   // Not shared over ROS2
   @Nullable
   private PersistentDetection detection = null;

   public DoorOpeningMechanism(DoorSide doorSide, YOLOv8DetectionClass yolOv8DetectionClass, UUID detectionID)
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
      this.detectionID = detectionID;
   }

   public DoorOpeningMechanism(DoorSide doorSide, DoorOpeningMechanismType type, UUID detectionID)
   {
      this.type = type;
      this.doorSide = doorSide;
      this.detectionID = detectionID;
   }

   public void update()
   {
      if (detection != null)
         mechanismFrame.update(transformToWorld -> transformToWorld.getTranslation()
                                                                   .set(detection.getFilteredDetectionFrame().getTransformToWorldFrame().getTranslation()));
   }

   public UUID getDetectionID()
   {
      return detectionID;
   }

   public void setDetection(PersistentDetection detection)
   {
      this.detection = detection;
   }

   @Nullable
   public PersistentDetection getDetection()
   {
      return detection;
   }

   public boolean hasDetection()
   {
      return detection != null;
   }

   public DoorOpeningMechanismType getType()
   {
      return type;
   }

   public DoorSide getDoorSide()
   {
      return doorSide;
   }

   public ReferenceFrame getMechanismFrame()
   {
      return mechanismFrame.getReferenceFrame();
   }

   public void updateMechanismFrame(RigidBodyTransformReadOnly newTransformToWorld)
   {
      mechanismFrame.update(transformToWorld -> transformToWorld.set(newTransformToWorld));
   }

   public InstantDetection getLastDetection()
   {
      if (detection != null)
         return detection.getMostRecentDetection();

      return null;
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

   public void destroy()
   {
      if (detection != null)
         detection.markForDeletion();
   }

   public enum DoorOpeningMechanismType
   {
      UNKNOWN((byte) 0), LEVER_HANDLE((byte) 1), KNOB((byte) 2), PUSH_BAR((byte) 3), PULL_HANDLE((byte) 4);

      final byte byteValue;

      DoorOpeningMechanismType(byte byteValue)
      {
         this.byteValue = byteValue;
      }

      public byte getByteValue()
      {
         return byteValue;
      }

      public static DoorOpeningMechanismType fromByte(byte b)
      {
         for (DoorOpeningMechanismType value : values())
         {
            if (value.getByteValue() == b)
               return value;
         }
         return null;
      }
   }
}
