package us.ihmc.perception.sceneGraph.rigidBodies;

import java.util.ArrayList;
import java.util.List;

public enum PrimitiveRigidBodyShape
{
   BOX, PRISM, ELLIPSOID, CYLINDER, CONE;

   public static List<PrimitiveRigidBodyShape> getAvailableShapes()
   {
      List<PrimitiveRigidBodyShape> availableShapes = new ArrayList<>();
      availableShapes.add(BOX);
      availableShapes.add(PRISM);
      availableShapes.add(ELLIPSOID);
      availableShapes.add(CYLINDER);
      availableShapes.add(CONE);
      return availableShapes;
   }
}
