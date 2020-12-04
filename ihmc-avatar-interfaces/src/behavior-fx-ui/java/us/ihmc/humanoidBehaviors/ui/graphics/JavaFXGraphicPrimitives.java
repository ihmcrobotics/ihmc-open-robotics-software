package us.ihmc.humanoidBehaviors.ui.graphics;

import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.Sphere;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.javaFXVisualizers.JavaFXGraphicTools;
import us.ihmc.robotics.geometry.GeometryTools;

public class JavaFXGraphicPrimitives
{
   public static Node createBoundingBox3D(BoundingBox3D boundingBox, Color color, double lineWidth)
   {
      Box3D box = GeometryTools.convertBoundingBox3DToBox3D(boundingBox);

      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();

      JavaFXGraphicTools.drawBoxEdges(meshBuilder, box, lineWidth);
      MeshView boundingBoxMeshView = new MeshView(meshBuilder.generateMesh());
      boundingBoxMeshView.setMaterial(new PhongMaterial(color));
      return boundingBoxMeshView;
   }

   public static Node createSphere3D(Point3D point, Color color, double radius)
   {
      Sphere sphere = new Sphere(radius);
      PhongMaterial material = new PhongMaterial(color);
      sphere.setMaterial(material);
      JavaFXGraphicTools.setNodePosition(sphere, point);
      return sphere;
   }
}
