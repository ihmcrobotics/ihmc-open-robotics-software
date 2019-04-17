package us.ihmc.humanoidBehaviors.ui.graphics;

import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;

public class OrientationGraphic
{
   private final SnappedPositionGraphic positionGraphic;

   private final MeshView arrow;

   private final double cylinderLength;

   public OrientationGraphic(SnappedPositionGraphic positionGraphic)
   {
      this.positionGraphic = positionGraphic;

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

   public MeshView getArrow()
   {
      return arrow;
   }

   public void setPosition(Point3D position)
   {
      double orientationRadians = Math.toRadians(arrow.getRotate());
      arrow.setTranslateX(position.getX() + 0.5 * cylinderLength * (Math.cos(orientationRadians) - 1.0));
      arrow.setTranslateY(position.getY() + 0.5 * cylinderLength * Math.sin(orientationRadians));
      arrow.setTranslateZ(position.getZ());
   }

   public void setOrientation(Point3D orientationPoint)
   {
      Vector3D difference = new Vector3D();
      difference.sub(orientationPoint, positionGraphic.getPosition());
      double startYaw = Math.atan2(difference.getY(), difference.getX());
      arrow.setTranslateX(positionGraphic.getSphere().getTranslateX() + 0.5 * cylinderLength * (Math.cos(startYaw) - 1.0));
      arrow.setTranslateY(positionGraphic.getSphere().getTranslateY() + 0.5 * cylinderLength * Math.sin(startYaw));
      arrow.setTranslateZ(positionGraphic.getSphere().getTranslateZ());
      arrow.setRotate(Math.toDegrees(startYaw));
   }
}
