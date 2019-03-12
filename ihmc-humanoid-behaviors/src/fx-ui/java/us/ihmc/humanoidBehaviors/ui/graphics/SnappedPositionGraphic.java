package us.ihmc.humanoidBehaviors.ui.graphics;

import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Sphere;
import us.ihmc.euclid.tuple3D.Point3D;

public class SnappedPositionGraphic
{
   private final Sphere sphere;
   private final PhongMaterial material;

   public SnappedPositionGraphic(Color color)
   {
      sphere = new Sphere(0.05);
      material = new PhongMaterial(color);
      sphere.setMaterial(material);
   }

   public void setPosition(Point3D position)
   {
      sphere.setTranslateX(position.getX());
      sphere.setTranslateY(position.getY());
      sphere.setTranslateZ(position.getZ());
   }

   public Point3D getPosition()
   {
      return new Point3D(sphere.getTranslateX(), sphere.getTranslateY(), sphere.getTranslateZ());
   }

   public Sphere getSphere()
   {
      return sphere;
   }
}
