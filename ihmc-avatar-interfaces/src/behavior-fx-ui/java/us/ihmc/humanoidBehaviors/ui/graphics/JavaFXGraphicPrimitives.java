package us.ihmc.humanoidBehaviors.ui.graphics;

import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.Sphere;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.javaFXVisualizers.JavaFXGraphicTools;
import us.ihmc.robotics.geometry.GeometryTools;

import java.util.List;

public class JavaFXGraphicPrimitives
{
   public static MeshView createBoundingBox3D(BoundingBox3D boundingBox, Color color, double lineWidth)
   {
      Box3D box = GeometryTools.convertBoundingBox3DToBox3D(boundingBox);

      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();

      JavaFXGraphicTools.drawBoxEdges(meshBuilder, box, lineWidth);
      MeshView boundingBoxMeshView = new MeshView(meshBuilder.generateMesh());
      boundingBoxMeshView.setMaterial(new PhongMaterial(color));
      return boundingBoxMeshView;
   }

   public static Sphere createSphere3D(Point3D point, Color color, double radius)
   {
      Sphere sphere = new Sphere(radius);
      PhongMaterial material = new PhongMaterial(color);
      sphere.setMaterial(material);
      JavaFXGraphicTools.setNodePosition(sphere, point);
      return sphere;
   }

   public static MeshView createPath(List<? extends Pose3DReadOnly> path, Color color)
   {
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();
      for (int segmentIndex = 0; segmentIndex < path.size() - 1; segmentIndex++)
      {
         Point3DReadOnly lineStart = path.get(segmentIndex).getPosition();
         Point3DReadOnly lineEnd = path.get(segmentIndex + 1).getPosition();
         double lineThickness = 0.025;
         meshBuilder.addLine(lineStart, lineEnd, lineThickness);
         // TODO: Draw orientation somehow
      }

      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(new PhongMaterial(color));

      return meshView;
   }
}
