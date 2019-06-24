package us.ihmc.humanoidBehaviors.ui.graphics;

import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidBehaviors.ui.tools.JavaFXGraphicTools;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;

public class OrientationGraphic
{
   private final MeshView arrow;
   private final double cylinderLength;
   private final FramePose3D pose = new FramePose3D();

   public OrientationGraphic()
   {
      cylinderLength = 0.25;
      double radius = 0.01;
      Color color = Color.GREEN;

      TextureColorPalette1D colorPalette = new TextureColorPalette1D();
      colorPalette.setHueBased(1.0, 1.0);
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette);

      double coneHeight = 0.10 * cylinderLength;
      double coneRadius = 1.5 * radius;

      meshBuilder.addCylinder(cylinderLength, radius, new Point3D(), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), color);
      meshBuilder.addCone(coneHeight, coneRadius, new Point3D(cylinderLength, 0.0, 0.0), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), color);

      arrow = new MeshView(meshBuilder.generateMesh());
      arrow.setMaterial(meshBuilder.generateMaterial());
      arrow.setVisible(false);
   }

   public FramePose3DBasics getPose()
   {
      return pose;
   }

   public void update()
   {
      JavaFXGraphicTools.setNodeTransformFromPose(arrow, pose);
   }

   public Node getNode()
   {
      return arrow;
   }

   public double getYaw()
   {
      return pose.getYaw();
   }
}
