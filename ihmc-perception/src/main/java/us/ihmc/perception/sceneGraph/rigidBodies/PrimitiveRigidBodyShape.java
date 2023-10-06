package us.ihmc.perception.sceneGraph.rigidBodies;

import java.util.ArrayList;
import java.util.List;

public enum PrimitiveRigidBodyShape
{
   BOX, PYRAMID, CYLINDER, SPHERE, ELLIPSOID, TORUS;

   public static List<PrimitiveRigidBodyShape> getAvailableShapes()
   {
      List<PrimitiveRigidBodyShape> availableShapes = new ArrayList<>();
      availableShapes.add(BOX);
      availableShapes.add(PYRAMID);
      availableShapes.add(CYLINDER);
      availableShapes.add(SPHERE);
      availableShapes.add(ELLIPSOID);
      availableShapes.add(TORUS);
      return availableShapes;
   }
}
