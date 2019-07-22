package us.ihmc.humanoidBehaviors.ui.graphics;

import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Sphere;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidBehaviors.ui.model.interfaces.PositionEditable;

public class PositionGraphic implements PositionEditable
{
   private final Sphere sphere;
   private final PhongMaterial material;
   private final FramePose3D pose = new FramePose3D();

   public PositionGraphic(Color color, double radius)
   {
      sphere = new Sphere(radius);
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

   public void clear()
   {
      pose.getPosition().setToNaN();
      update();
   }

   @Override
   public void setPosition(Point3DReadOnly position)
   {
      pose.setPosition(position);
      update();
   }

   @Override
   public void setMouseTransparent(boolean transparent)
   {
      sphere.setMouseTransparent(transparent);
   }
}
