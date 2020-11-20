package us.ihmc.humanoidBehaviors.ui.graphics;

import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidBehaviors.ui.model.interfaces.PositionEditable;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javaFXVisualizers.JavaFXGraphicTools;

public class DoorGraphic implements PositionEditable
{
   private final MeshView box = new MeshView();
   private final FramePose3D pose = new FramePose3D();

   public DoorGraphic(Color color)
   {
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(1024, false));
      float lx = 0.914f;
      float ly = 0.0345f;
      float lz = 2.034f;
      meshBuilder.addBox(lx, ly, lz, new Vector3D(lx/ 2.0, ly / 2.0, lz / 2.0), color);
      box.setMesh(meshBuilder.generateMesh());
      box.setMaterial(meshBuilder.generateMaterial());
   }

   public void setPose(Pose3DReadOnly pose)
   {
      this.pose.set(pose);
      update();
   }

   public FramePose3DReadOnly getPose()
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
