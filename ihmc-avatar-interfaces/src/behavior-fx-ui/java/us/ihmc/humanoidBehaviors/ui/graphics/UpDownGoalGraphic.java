package us.ihmc.humanoidBehaviors.ui.graphics;

import javafx.collections.ObservableList;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Sphere;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.humanoidBehaviors.ui.tools.JavaFXGraphicTools;
import us.ihmc.humanoidBehaviors.upDownExploration.UpDownFlatAreaFinder;
import us.ihmc.humanoidBehaviors.upDownExploration.UpDownResult;
import us.ihmc.log.LogTools;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class UpDownGoalGraphic
{
   public static final Color VALID_COLOR = Color.BLUE; // for color blind people :)
   public static final Color INVALID_COLOR = Color.ORANGE;

   private final List<Sphere> spheres = new ArrayList<>();
   private final List<FramePose3D> poses = new ArrayList<>();

   public UpDownGoalGraphic()
   {
      for (int i = 0; i < UpDownFlatAreaFinder.NUMBER_OF_VERTICES + 1; i++)
      {
         spheres.add(new Sphere(0.015));
         spheres.get(i).setMaterial(new PhongMaterial(INVALID_COLOR));
         poses.add(new FramePose3D());
      }

      setValid(false);
   }

   public List<FramePose3D> getPoses()
   {
      return poses;
   }

   public void setResult(UpDownResult result)
   {
      setPoses(result.getPoints());
      setValid(result.isValid());
   }

   public void setPoses(List<Pose3D> newPoses)
   {
      for (int i = 0; i < poses.size(); i++)
      {
         poses.get(i).set(newPoses.get(i));
      }
      update();
   }

   public void update()
   {
      for (int i = 0; i < spheres.size(); i++)
      {
         FramePose3D pose = poses.get(i);
         if (Double.isNaN(pose.getZ()))
         {
            pose.setZ(0.0);
         }

         JavaFXGraphicTools.setNodeTransformFromPose(spheres.get(i), pose);
      }
   }

   public void setValid(boolean valid)
   {
      for (Sphere sphere : spheres)
      {
         ((PhongMaterial) sphere.getMaterial()).setDiffuseColor(valid ? VALID_COLOR : INVALID_COLOR);
      }
   }

   public void callOnNodes(Consumer<Node> nodeConsumer)
   {
      spheres.forEach(sphere -> nodeConsumer.accept(sphere));
   }

   public List<Sphere> getNodes()
   {
      return spheres;
   }
}
