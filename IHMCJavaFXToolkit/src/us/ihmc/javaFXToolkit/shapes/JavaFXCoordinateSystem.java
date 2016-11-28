package us.ihmc.javaFXToolkit.shapes;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;

import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;

public class JavaFXCoordinateSystem extends Group
{
   public JavaFXCoordinateSystem(double length)
   {
      TextureColorPalette1D colorPalette = new TextureColorPalette1D();
      colorPalette.setHueBased(1.0, 1.0);
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette);
      double radius = 0.02 * length;
      double coneHeight = 0.10 * length;
      double coneRadius = 0.05 * length;
      meshBuilder.addCylinder(length, radius, new Point3d(), new AxisAngle4d(0.0, 1.0, 0.0, Math.PI / 2.0), Color.RED);
      meshBuilder.addCone(coneHeight, coneRadius, new Point3d(length, 0.0, 0.0), new AxisAngle4d(0.0, 1.0, 0.0, Math.PI / 2.0), Color.RED);
      meshBuilder.addCylinder(length, radius, new Point3d(), new AxisAngle4d(1.0, 0.0, 0.0, -Math.PI / 2.0), Color.GREEN);
      meshBuilder.addCone(coneHeight, coneRadius, new Point3d(0.0, length, 0.0), new AxisAngle4d(1.0, 0.0, 0.0, -Math.PI / 2.0), Color.GREEN);
      meshBuilder.addCylinder(length, radius, new Point3d(), Color.BLUE);
      meshBuilder.addCone(coneHeight, coneRadius, new Point3d(0.0, 0.0, length), Color.BLUE);

      MeshView coordinateSystem = new MeshView(meshBuilder.generateMesh());
      coordinateSystem.setMaterial(meshBuilder.generateMaterial());
      getChildren().addAll(coordinateSystem);
   }
}
