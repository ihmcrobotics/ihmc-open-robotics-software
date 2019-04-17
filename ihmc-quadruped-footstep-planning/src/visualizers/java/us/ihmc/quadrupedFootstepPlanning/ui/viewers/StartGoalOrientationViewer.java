package us.ihmc.quadrupedFootstepPlanning.ui.viewers;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.StartGoalPositionViewer.*;

public class StartGoalOrientationViewer extends AnimationTimer
{
   private final Group root = new Group();

   private static final double cylinderLength = 5.0 * RADIUS;
   private final ArrowGraphic startArrow = new ArrowGraphic(0.2 * RADIUS, cylinderLength, Color.GREEN);
   private final ArrowGraphic lowLevelGoalArrow = new ArrowGraphic(0.2 * RADIUS, cylinderLength, Color.DARKRED);
   private final ArrowGraphic goalArrow = new ArrowGraphic(0.2 * RADIUS, cylinderLength, Color.RED);

   private final AtomicReference<Boolean> startRotationEditModeEnabled;
   private final AtomicReference<Boolean> goalRotationEditModeEnabled;

   private final AtomicReference<Point3D> startPositionReference;
   private final AtomicReference<Point3D> lowLevelGoalPositionReference;
   private final AtomicReference<Point3D> goalPositionReference;

   private final AtomicReference<Quaternion> startQuaternionReference;
   private final AtomicReference<Quaternion> lowLevelGoalQuaternionReference;
   private final AtomicReference<Quaternion> goalQuaternionReference;

   public StartGoalOrientationViewer(Messager messager, Topic<Boolean> startOrientationEditModeEnabledTopic, Topic<Boolean> goalOrientationEditModeEnabledTopic,
                                     Topic<Point3D> startPositionTopic, Topic<Quaternion> startOrientationTopic, Topic<Point3D> lowLevelGoalPositionTopic,
                                     Topic<Quaternion> lowLevelGoalOrientationTopic, Topic<Point3D> goalPositionTopic, Topic<Quaternion> goalOrientationTopic)
   {
      startArrow.setMouseTransparent(true);
      lowLevelGoalArrow.setMouseTransparent(true);
      goalArrow.setMouseTransparent(true);

      root.getChildren().add(startArrow);
      root.getChildren().add(lowLevelGoalArrow);
      root.getChildren().add(goalArrow);

      startRotationEditModeEnabled = messager.createInput(startOrientationEditModeEnabledTopic, false);
      goalRotationEditModeEnabled = messager.createInput(goalOrientationEditModeEnabledTopic, false);

      startPositionReference = messager.createInput(startPositionTopic, new Point3D());
      lowLevelGoalPositionReference = messager.createInput(lowLevelGoalPositionTopic, new Point3D());
      goalPositionReference = messager.createInput(goalPositionTopic, new Point3D());

      startQuaternionReference = messager.createInput(startOrientationTopic, new Quaternion());
      lowLevelGoalQuaternionReference = messager.createInput(lowLevelGoalOrientationTopic, new Quaternion());
      goalQuaternionReference = messager.createInput(goalOrientationTopic, new Quaternion());
   }

   @Override
   public void handle(long now)
   {
      if (startRotationEditModeEnabled.get())
         startArrow.setMaterial(startTransparentMaterial);
      else
         startArrow.setMaterial(startOpaqueMaterial);

      Point3D startPosition = startPositionReference.get();
      if (startPosition != null)
      {
         setArrowPose(startArrow, startPosition, startQuaternionReference.get().getYaw());
      }

      if (goalRotationEditModeEnabled.get())
         goalArrow.setMaterial(goalTransparentMaterial);
      else
         goalArrow.setMaterial(goalOpaqueMaterial);

      Point3D goalPosition = goalPositionReference.get();
      if (goalPosition != null)
      {
         setArrowPose(goalArrow, goalPosition, goalQuaternionReference.get().getYaw());
      }

      Point3D lowLevelGoalPosition = lowLevelGoalPositionReference.get();
      if (goalPosition != null)
      {
         setArrowPose(lowLevelGoalArrow, lowLevelGoalPosition, lowLevelGoalQuaternionReference.get().getYaw());
      }
   }

   private static void setArrowPose(ArrowGraphic arrow, Point3D position, double orientationRadians)
   {
      arrow.setTranslateX(position.getX() + 0.5 * cylinderLength * (Math.cos(orientationRadians) - 1.0));
      arrow.setTranslateY(position.getY() + 0.5 * cylinderLength * Math.sin(orientationRadians));
      arrow.setTranslateZ(position.getZ());
      arrow.setRotate(Math.toDegrees(orientationRadians));
   }

   private class ArrowGraphic extends Group
   {
      private final MeshView arrow;

      public ArrowGraphic(double radius, double length, Color color)
      {
         TextureColorPalette1D colorPalette = new TextureColorPalette1D();
         colorPalette.setHueBased(1.0, 1.0);
         JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette);

         double coneHeight = 0.10 * length;
         double coneRadius = 1.5 * radius;

         meshBuilder.addCylinder(length, radius, new Point3D(), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), color);
         meshBuilder.addCone(coneHeight, coneRadius, new Point3D(length, 0.0, 0.0), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), color);

         this.arrow = new MeshView(meshBuilder.generateMesh());
         arrow.setMaterial(meshBuilder.generateMaterial());
         getChildren().add(arrow);
      }

      public void setMaterial(PhongMaterial material)
      {
         arrow.setMaterial(material);
      }
   }

   public Node getRoot()
   {
      return root;
   }
}
