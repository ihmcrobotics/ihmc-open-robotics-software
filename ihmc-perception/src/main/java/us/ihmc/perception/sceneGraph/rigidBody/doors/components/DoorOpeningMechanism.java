package us.ihmc.perception.sceneGraph.rigidBody.doors.components;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.perception.YOLOv8.YOLOv8DetectionClass;

/**
 * Represents hardware on a door which you have to interact with to open it, could be a knob, lever, deadbolt lock, etc...
 */
public class DoorOpeningMechanism
{
   private final DoorOpeningMechanismType type;
   private final Pose3D graspPose = new Pose3D();

   public DoorOpeningMechanism(DoorOpeningMechanismType type)
   {
      this.type = type;
   }

   public DoorOpeningMechanism(YOLOv8DetectionClass yolOv8DetectionClass)
   {
      switch (yolOv8DetectionClass)
      {
         case DOOR_LEVER -> type = DoorOpeningMechanismType.LEVER_HANDLE;
         case DOOR_KNOB -> type = DoorOpeningMechanismType.KNOB;
         case DOOR_PULL_HANDLE -> type = DoorOpeningMechanismType.PULL_HANDLE;
         case DOOR_PUSH_BAR -> type = DoorOpeningMechanismType.PUSH_BAR;
         default -> type = DoorOpeningMechanismType.UNKNOWN;
      }
   }

   public DoorOpeningMechanismType getType()
   {
      return type;
   }

   public Pose3D getGraspPose()
   {
      return graspPose;
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
