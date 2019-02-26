package us.ihmc.humanoidBehaviors.ui.graphics;

import javafx.scene.input.MouseEvent;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Sphere;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.ui.references.ActivationReference;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.references.ChangingReference;
import us.ihmc.humanoidBehaviors.ui.editors.SnappedPositionEditor;
import us.ihmc.humanoidBehaviors.ui.model.FXUIGraphic;
import us.ihmc.messager.Messager;

public class SnappedPositionGraphic extends FXUIGraphic
{
   public final Sphere sphere;
   public final PhongMaterial material;

   private final ActivationReference<FXUIGraphic> selectedReference;
   private final ChangingReference<Point3D> positionReference;

   public SnappedPositionGraphic(Messager messager, Color color)
   {
      sphere = new Sphere(0.05);
      material = new PhongMaterial(color);
      sphere.setMaterial(material);

      positionReference = new ChangingReference<>(messager.createInput(SnappedPositionEditor.API.SelectedPosition, new Point3D()));
      selectedReference = new ActivationReference<>(messager.createInput(BehaviorUI.API.SelectedGraphic, FXUIGraphic.NONE), this);

      rootChildren.add(sphere);
   }

   @Override
   public void handle(long now)
   {
      if (selectedReference.pollActivated())
      {
         if (selectedReference.activationChanged())
         {
            sphere.setMouseTransparent(true);
         }

         Point3D position = positionReference.poll();
         if (positionReference.hasChanged())
         {
            sphere.setTranslateX(position.getX());
            sphere.setTranslateY(position.getY());
            sphere.setTranslateZ(position.getZ());
         }
      }
      else if (selectedReference.activationChanged())
      {
         sphere.setMouseTransparent(false);
      }
   }
}
