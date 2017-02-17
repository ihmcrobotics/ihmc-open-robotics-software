package us.ihmc.javaFXToolkit.shapes;

import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple3D.Point3D;

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
      meshBuilder.addCylinder(length, radius, new Point3D(), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), Color.RED);
      meshBuilder.addCone(coneHeight, coneRadius, new Point3D(length, 0.0, 0.0), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), Color.RED);
      meshBuilder.addCylinder(length, radius, new Point3D(), new AxisAngle(1.0, 0.0, 0.0, -Math.PI / 2.0), Color.GREEN);
      meshBuilder.addCone(coneHeight, coneRadius, new Point3D(0.0, length, 0.0), new AxisAngle(1.0, 0.0, 0.0, -Math.PI / 2.0), Color.GREEN);
      meshBuilder.addCylinder(length, radius, new Point3D(), Color.BLUE);
      meshBuilder.addCone(coneHeight, coneRadius, new Point3D(0.0, 0.0, length), Color.BLUE);

      MeshView coordinateSystem = new MeshView(meshBuilder.generateMesh());
      coordinateSystem.setMaterial(meshBuilder.generateMaterial());
      getChildren().addAll(coordinateSystem);
   }
}
