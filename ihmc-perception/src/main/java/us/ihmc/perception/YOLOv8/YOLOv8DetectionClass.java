package us.ihmc.perception.YOLOv8;

import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;

import javax.annotation.Nullable;

public enum YOLOv8DetectionClass
{
   DRILL(null, null),
   DOOR_LEVER(null, null),
   DOOR_PULL_HANDLE(null, null),
   DOOR_PUSH_BAR(null, null);

   @Nullable
   private PrimitiveRigidBodyShape primitiveApproximation;

   private String pointCloudFileName;

   YOLOv8DetectionClass(PrimitiveRigidBodyShape primitiveApproximation, String pointCloudFileName)
   {
      this.primitiveApproximation = primitiveApproximation;
      this.pointCloudFileName = pointCloudFileName;
   }

   /**
    * The corresponding shape is the best primitive approximation of the object determined by Tomasz.
    * For example, a bird is just an ellipsoid, a train is a long box, and a carrot is just a cone.
    * If you disagree, feel free to change the corresponding shape (although you're clearly wrong).
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

   public int getClassId()
   {
      return ordinal();
   }

   public static YOLOv8DetectionClass fromByte(byte enumAsByte)
   {
      return values()[enumAsByte];
   }

   public byte toByte()
   {
      return (byte) this.ordinal();
   }

   @Override
   public String toString()
   {
      return name().toLowerCase().replaceAll("_", " ");
   }
}