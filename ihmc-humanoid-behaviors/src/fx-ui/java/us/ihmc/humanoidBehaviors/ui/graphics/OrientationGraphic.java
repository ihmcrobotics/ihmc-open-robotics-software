package us.ihmc.humanoidBehaviors.ui.graphics;

import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.editors.OrientationYawEditor;
import us.ihmc.humanoidBehaviors.ui.editors.SnappedPositionEditor;
import us.ihmc.humanoidBehaviors.ui.model.FXUIGraphic;
import us.ihmc.humanoidBehaviors.ui.references.ActivationReference;
import us.ihmc.humanoidBehaviors.ui.references.ChangingReference;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;

public class OrientationGraphic extends FXUIGraphic
{
   private final SnappedPositionGraphic positionGraphic;

   private final MeshView arrow;

   private final ActivationReference<Node> selectedReference;
   private final ActivationReference<Node> positionSelectedReference;
   private final ChangingReference<Point3D> positionReference;
   private final ChangingReference<Point3D> orientationPointReference;
   private final double cylinderLength;

   public OrientationGraphic(Messager messager, SnappedPositionGraphic positionGraphic)
   {
      super(messager);

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
      rootChildren.add(arrow);

      selectedReference = new ActivationReference<>(messager.createInput(BehaviorUI.API.SelectedGraphic, null), arrow);
      positionSelectedReference = new ActivationReference<>(messager.createInput(BehaviorUI.API.SelectedGraphic, null), positionGraphic.getSphere());
      positionReference = new ChangingReference<>(messager.createInput(SnappedPositionEditor.API.SelectedPosition, new Point3D()));
      orientationPointReference = new ChangingReference<>(messager.createInput(OrientationYawEditor.API.SelectedOrientation, new Point3D()));
   }

   @Override
   public void handle(long now)
   {
      if (positionSelectedReference.pollActivated())
      {
         if (positionSelectedReference.activationChanged())
         {
            arrow.setMouseTransparent(true);
         }

         Point3D position = positionReference.poll();
         if (positionReference.hasChanged())
         {
            double orientationRadians = Math.toRadians(arrow.getRotate());
            arrow.setTranslateX(position.getX() + 0.5 * cylinderLength * (Math.cos(orientationRadians) - 1.0));
            arrow.setTranslateY(position.getY() + 0.5 * cylinderLength * Math.sin(orientationRadians));
            arrow.setTranslateZ(position.getZ());
         }
      }
      else if (positionSelectedReference.activationChanged())
      {
         arrow.setMouseTransparent(false);
      }

      if (selectedReference.pollActivated())
      {
         if (selectedReference.activationChanged())
         {
            LogTools.debug("Arrow graphic selected");
            arrow.setMouseTransparent(true);
         }

         Point3D orientationPoint = orientationPointReference.poll();
         if (orientationPointReference.hasChanged())
         {
            Vector3D difference = new Vector3D();
            difference.setX(orientationPoint.getX() - positionGraphic.getSphere().getTranslateX());
            difference.setY(orientationPoint.getY() - positionGraphic.getSphere().getTranslateY());
            difference.setZ(orientationPoint.getZ() - positionGraphic.getSphere().getTranslateZ());
            double startYaw = Math.atan2(difference.getY(), difference.getX());
            arrow.setTranslateX(positionGraphic.getSphere().getTranslateX() + 0.5 * cylinderLength * (Math.cos(startYaw) - 1.0));
            arrow.setTranslateY(positionGraphic.getSphere().getTranslateY() + 0.5 * cylinderLength * Math.sin(startYaw));
            arrow.setTranslateZ(positionGraphic.getSphere().getTranslateZ());
            arrow.setRotate(Math.toDegrees(startYaw));
         }
      }
      else if (selectedReference.activationChanged())
      {
         LogTools.debug("Arrow graphic deselected");
         arrow.setMouseTransparent(false);
      }
   }

   public MeshView getArrow()
   {
      return arrow;
   }
}
