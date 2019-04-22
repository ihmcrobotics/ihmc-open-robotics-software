package us.ihmc.humanoidBehaviors.ui.graphics;

import javafx.scene.paint.Color;
import javafx.scene.shape.DrawMode;
import org.fxyz3d.shapes.primitives.Text3DMesh;
import us.ihmc.euclid.tuple3D.Point3D;

public class LabelGraphic
{
   private final Text3DMesh textMesh;

   public LabelGraphic(String text, double height, Color color)
   {
      textMesh = new Text3DMesh(text, height);
      textMesh.setTextureModeNone(color);
      textMesh.setDrawMode(DrawMode.FILL);
      double value = 0.1;
      textMesh.setScaleX(value);
      textMesh.setScaleY(value);
      textMesh.setScaleZ(value);

   }

   public void setPosition(Point3D position)
   {
      textMesh.setTranslateX(position.getX());
      textMesh.setTranslateY(position.getY());
      textMesh.setTranslateZ(position.getZ());
   }

   public Text3DMesh getMesh()
   {
      return textMesh;
   }
}
