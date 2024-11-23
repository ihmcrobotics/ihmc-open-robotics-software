package us.ihmc.javafx;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.javaFXToolkit.JavaFXGraphicTools;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.robotics.geometry.PlanarRegion;

public class JavaFXGraphicMissingTools
{
   public static void drawPlanarRegion(JavaFXMeshBuilder meshBuilder, PlanarRegion planarRegion, double lineWidth)
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      planarRegion.getTransformToWorld(transformToWorld);

      JavaFXGraphicTools.drawPlanarRegion(meshBuilder, transformToWorld, planarRegion.getConcaveHull(), planarRegion.getConvexPolygons(), lineWidth);
   }
}