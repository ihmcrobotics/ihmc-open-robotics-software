package us.ihmc.humanoidBehaviors.ui.graphics;

import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidBehaviors.ui.model.interfaces.PositionEditable;
import us.ihmc.javaFXVisualizers.JavaFXGraphicTools;

public class DoorGraphic implements PositionEditable
{
   private final Box box;
   private final PhongMaterial material;
   private final FramePose3D pose = new FramePose3D();

   public DoorGraphic(Color color)
   {
      box = new Box(0.914, 0.0345, 2.034);
      material = new PhongMaterial(color);
      box.setMaterial(material);
   }

   public FramePose3DBasics getPose()
   {
      return pose;
   }

   public void update()
   {
      JavaFXGraphicTools.setNodeTransformFromPose(box, pose);
   }

   public Node getNode()
   {
      return box;
   }

   public void clear()
   {
      pose.getPosition().setToNaN();
      update();
   }

   public void setTransparency(double alpha)
   {

   }

   public void setVisible(boolean visible)
   {
      box.setVisible(visible);
   }

   @Override
   public void setPosition(Point3DReadOnly position)
   {
      pose.getPosition().set(position);
      update();
   }

   @Override
   public void setMouseTransparent(boolean transparent)
   {
      box.setMouseTransparent(transparent);
   }
}
