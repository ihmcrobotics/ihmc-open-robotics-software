package us.ihmc.humanoidBehaviors.ui.graphics;

import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.javaFXVisualizers.JavaFXGraphicTools;
import us.ihmc.robotics.geometry.GeometryTools;

public class BoundingBox3DGraphic
{
   private final MeshView boundingBoxMeshView;

   public BoundingBox3DGraphic(BoundingBox3D boundingBox, Color color, double lineWidth)
   {
      Box3D box = GeometryTools.convertBoundingBox3DToBox3D(boundingBox);

      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();

      JavaFXGraphicTools.drawBoxEdges(meshBuilder, box, lineWidth);
      boundingBoxMeshView = new MeshView(meshBuilder.generateMesh());
      boundingBoxMeshView.setMaterial(new PhongMaterial(color));
   }

   public Node getNode()
   {
      return boundingBoxMeshView;
   }
}
