package us.ihmc.perception.sceneGraph.rigidBody.primitive;

public enum PrimitiveRigidBodyShape
{
   BOX, PRISM, ELLIPSOID, CYLINDER, CONE;

   public static PrimitiveRigidBodyShape fromString(String shapeString)
   {
      return PrimitiveRigidBodyShape.valueOf(shapeString.toUpperCase());
   }
}
