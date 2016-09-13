package us.ihmc.javaFXToolkit.shapes;

import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Cylinder;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;

public class JavaFXCoordinateSystem extends Group
{
   public JavaFXCoordinateSystem(double length)
   {
      Cylinder xAxis = addCylinder(length, Color.RED);
      xAxis.getTransforms().add(new Rotate(90, Rotate.Z_AXIS));
      xAxis.getTransforms().add(new Translate(0.0, -length / 2.0, 0.0));
      Cylinder yAxis = addCylinder(length, Color.WHITE);
      yAxis.getTransforms().add(new Rotate(0, Rotate.Z_AXIS));
      yAxis.getTransforms().add(new Rotate(180, Rotate.X_AXIS));
      yAxis.getTransforms().add(new Translate(0.0, -length / 2.0, 0.0));
      Cylinder zAxis = addCylinder(length, Color.BLUE);
      zAxis.getTransforms().add(new Rotate(-90, Rotate.X_AXIS));
      zAxis.getTransforms().add(new Translate(0.0, -length / 2.0, 0.0));

      getChildren().addAll(xAxis, yAxis, zAxis);
   }

   public Cylinder addCylinder(double length, Color color)
   {
      double radius = 0.02 * length;

      Cylinder cylinder = new Cylinder(radius, length);

      PhongMaterial material = new PhongMaterial();
      material.setDiffuseColor(color);
      material.setSpecularColor(color.brighter());
      cylinder.setMaterial(material);

      return cylinder;
   }
}
