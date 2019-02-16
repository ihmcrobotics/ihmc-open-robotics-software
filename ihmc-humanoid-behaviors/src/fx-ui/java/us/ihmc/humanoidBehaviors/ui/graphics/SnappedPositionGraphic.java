package us.ihmc.humanoidBehaviors.ui.graphics;

import javafx.animation.AnimationTimer;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Sphere;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;

import java.util.concurrent.atomic.AtomicReference;

public class SnappedPositionGraphic extends FXUIGraphic
{
   private final Messager messager;

   public final Sphere sphere;
   public final PhongMaterial material;

   private AtomicReference<Point3D> position;

   public SnappedPositionGraphic(Messager messager, Color color, Topic<Point3D> positionTopic)
   {
      this.messager = messager;

      sphere = new Sphere(0.05);
      material = new PhongMaterial(color);

      position = messager.createInput(positionTopic, new Point3D());

      rootChildren.add(sphere);
   }

   @Override
   public void handle(long now)
   {
      Point3D position = this.position.get();
      sphere.setTranslateX(position.getX());
      sphere.setTranslateX(position.getY());
      sphere.setTranslateX(position.getZ());
   }
}
