package us.ihmc.footstepPlanning.ui;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.shape.Cylinder;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.footstepPlanning.ui.FootstepPlannerUserInterfaceAPI.*;

import static us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.StartGoalPositionViewer.RADIUS;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.StartGoalPositionViewer.startOpaqueMaterial;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.StartGoalPositionViewer.startTransparentMaterial;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.StartGoalPositionViewer.goalOpaqueMaterial;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.StartGoalPositionViewer.goalTransparentMaterial;

public class StartGoalOrientationViewer extends AnimationTimer
{
   private final Group root = new Group();

   private static final double cylinderLength = 5.0 * RADIUS;
   private final Cylinder startArrow = new Cylinder(0.2 * RADIUS, cylinderLength);
   private final Cylinder goalArrow = new Cylinder(0.2 * RADIUS, cylinderLength);

   private final AtomicReference<Boolean> startRotationEditModeEnabled;
   private final AtomicReference<Boolean> goalRotationEditModeEnabled;

   private final AtomicReference<Point3D> startPositionReference;
   private final AtomicReference<Point3D> goalPositionReference;

   private final AtomicReference<Double> startRotationReference;
   private final AtomicReference<Double> goalRotationReference;

   public StartGoalOrientationViewer(REAMessager messager)
   {
      startArrow.setMouseTransparent(true);
      goalArrow.setMouseTransparent(true);

      root.getChildren().add(startArrow);
      root.getChildren().add(goalArrow);

      startRotationEditModeEnabled = messager.createInput(StartOrientationEditModeEnabledTopic, false);
      goalRotationEditModeEnabled = messager.createInput(GoalOrientationEditModeEnabledTopic, false);

      startPositionReference = messager.createInput(StartPositionTopic, new Point3D());
      goalPositionReference = messager.createInput(GoalPositionTopic, new Point3D());

      startRotationReference = messager.createInput(StartOrientationTopic, 0.0);
      goalRotationReference = messager.createInput(GoalOrientationTopic, 0.0);
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
         setArrowPose(startArrow, startPosition, startRotationReference.get());
      }

      if (goalRotationEditModeEnabled.get())
         goalArrow.setMaterial(goalTransparentMaterial);
      else
         goalArrow.setMaterial(goalOpaqueMaterial);

      Point3D goalPosition = goalPositionReference.get();
      if (goalPosition != null)
      {
         setArrowPose(goalArrow, goalPosition, goalRotationReference.get());
      }
   }

   private static void setArrowPose(Cylinder arrow, Point3D position, double orientationRadians)
   {
      arrow.setTranslateX(position.getX() + 0.5 * cylinderLength * Math.cos(orientationRadians));
      arrow.setTranslateY(position.getY() + 0.5 * cylinderLength * Math.sin(orientationRadians));
      arrow.setTranslateZ(position.getZ());
      arrow.setRotate(Math.toDegrees(0.5 * Math.PI + orientationRadians));
   }

   public Node getRoot()
   {
      return root;
   }
}
