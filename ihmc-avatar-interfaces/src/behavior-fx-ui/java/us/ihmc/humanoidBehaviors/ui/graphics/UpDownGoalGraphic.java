package us.ihmc.humanoidBehaviors.ui.graphics;

import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Sphere;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.humanoidBehaviors.ui.tools.JavaFXGraphicTools;

import java.util.ArrayList;
import java.util.List;

public class UpDownGoalGraphic
{
   private final List<Sphere> spheres = new ArrayList<>();
   private final List<FramePose3D> poses = new ArrayList<>();

   public UpDownGoalGraphic()
   {
      for (int i = 0; i < 6; i++)
      {
         spheres.add(new Sphere(0.015));
         poses.add(new FramePose3D());
      }

      setValid(false);
   }

   public List<FramePose3D> getPoses()
   {
      return poses;
   }

   public void update()
   {
      for (int i = 0; i < spheres.size(); i++)
      {
         JavaFXGraphicTools.setNodeTransformFromPose(spheres.get(i), poses.get(i));
      }
   }

   public void setValid(boolean valid)
   {
      for (Sphere sphere : spheres)
      {
         sphere.setMaterial(new PhongMaterial(valid ? Color.GREEN : Color.RED));
      }
   }

   public List<Sphere> getNodes()
   {
      return spheres;
   }
}
