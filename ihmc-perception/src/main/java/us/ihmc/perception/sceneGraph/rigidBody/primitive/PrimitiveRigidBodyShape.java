package us.ihmc.perception.sceneGraph.rigidBody.primitive;

public enum PrimitiveRigidBodyShape
{
   BOX("Box"), PRISM("Prism"), ELLIPSOID("Ellipsoid"), CYLINDER("Cylinder"), CONE("Cone");

   private final String capitalizedName;

   PrimitiveRigidBodyShape(String capitalizedName)
   {
      this.capitalizedName = capitalizedName;
   }

   public String getCapitalizedName()
   {
      return capitalizedName;
   }

   public static PrimitiveRigidBodyShape fromString(String shapeString)
   {
      return PrimitiveRigidBodyShape.valueOf(shapeString.toUpperCase());
   }
   public static PrimitiveRigidBodyShape fromByte(byte enumAsByte)
   {
      return values()[enumAsByte];
   }
   public byte toByte()
   {
      return (byte) this.ordinal();
   }
}
