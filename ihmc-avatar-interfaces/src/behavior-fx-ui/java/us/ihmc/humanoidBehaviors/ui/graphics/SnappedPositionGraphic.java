package us.ihmc.humanoidBehaviors.ui.graphics;

import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Sphere;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.humanoidBehaviors.ui.tools.JavaFXGraphicTools;

public class SnappedPositionGraphic
{
   private final Sphere sphere;
   private final PhongMaterial material;
   private final FramePose3D pose = new FramePose3D();

   public SnappedPositionGraphic(Color color)
   {
      sphere = new Sphere(0.05);
      material = new PhongMaterial(color);
      sphere.setMaterial(material);
   }

   public FramePose3DBasics getPose()
   {
      return pose;
   }

   public void update()
   {
      JavaFXGraphicTools.setNodeTransformFromPose(sphere, pose);
   }

   public Node getNode()
   {
      return sphere;
   }
}
