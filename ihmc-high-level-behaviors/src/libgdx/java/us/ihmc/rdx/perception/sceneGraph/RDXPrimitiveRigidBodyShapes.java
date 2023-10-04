package us.ihmc.rdx.perception.sceneGraph;

import java.util.ArrayList;
import java.util.List;

public enum RDXPrimitiveRigidBodyShapes
{
   BOX, PYRAMID, CYLINDER, SPHERE, ELLIPSOID, TORUS;

   public static List<RDXPrimitiveRigidBodyShapes> getAvailableShapes()
   {
      List<RDXPrimitiveRigidBodyShapes> availableShapes = new ArrayList<>();
      availableShapes.add(BOX);
      availableShapes.add(PYRAMID);
      availableShapes.add(CYLINDER);
      availableShapes.add(SPHERE);
      availableShapes.add(ELLIPSOID);
      availableShapes.add(TORUS);
      return availableShapes;
   }
}
