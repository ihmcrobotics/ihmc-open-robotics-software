package us.ihmc.perception.YOLOv8;

import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;

import javax.annotation.Nullable;

/**
 * TODO: replace this with some json that gets downloaded at runtime and cached locally
 */
public enum YOLOv8DetectionClass
{
   DOOR_PANEL(0, "YOLODoorPanel", null, null),
   DOOR_LEVER(1, "YOLODoorLever", null, null),
   DOOR_KNOB(2, "YOLODoorKnob", null, null),
   DOOR_PUSH_BAR(3, "YOLODoorPushBar", null, null),
   DOOR_PULL_HANDLE(4, "YOLODoorPullHandle", null, null),
   TRASH_CAN(5, "YOLOTrashCan", null, null),
   WOOD_BAR(6, "YOLOWoodBar", null, null),
   TOM(7, "YOLOTom", null, null),
   COUCH(8, "YOLOCouch", null, null);

   private final int classID;
   private final String defaultNodeName;
   @Nullable
   private final PrimitiveRigidBodyShape primitiveApproximation;
   @Nullable
   private final String pointCloudFileName;

   YOLOv8DetectionClass(int classID, String defaultNodeName, @Nullable PrimitiveRigidBodyShape primitiveApproximation, @Nullable String pointCloudFileName)
   {
      this.classID = classID;
      this.defaultNodeName = defaultNodeName;
      this.primitiveApproximation = primitiveApproximation;
      this.pointCloudFileName = pointCloudFileName;
   }

   /**
    * The corresponding shape is the best primitive approximation of the object determined by Tomasz.
    * For example, a bird is just an ellipsoid, a train is a long box, and a carrot is just a cone.
    * If you disagree, feel free to change the corresponding shape (although you're clearly wrong).
    *
    * @return the objectively correct approximation of the object as a primitive shape. Null if no approximation is good.
    */
   public PrimitiveRigidBodyShape getPrimitiveApproximation()
   {
      return primitiveApproximation;
   }

   public String getPointCloudFileName()
   {
      return pointCloudFileName;
   }

   public int getClassID()
   {
      return classID;
   }

   public String getDefaultNodeName()
   {
      return defaultNodeName;
   }

   public byte toByte()
   {
      return (byte) this.ordinal();
   }

   public static YOLOv8DetectionClass fromByte(byte enumAsByte)
   {
      return values()[enumAsByte];
   }

   public static YOLOv8DetectionClass fromClassID(int classID)
   {
      for (YOLOv8DetectionClass detectionClass : values())
      {
         if (detectionClass.getClassID() == classID)
            return detectionClass;
      }

      return null;
   }

   @Override
   public String toString()
   {
      return name().toLowerCase().replaceAll("_", " ");
   }
}