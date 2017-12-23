package us.ihmc.pathPlanning.visibilityGraphs.ui.viewers;

import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Sphere;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotEnvironmentAwareness.communication.APIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;

public class StartGoalPositionViewer extends AnimationTimer
{
   public static final double RADIUS = 0.05;

   private final Group root = new Group();
   private final ObservableList<Node> rootChildren = root.getChildren();

   private final Sphere startSphere = new Sphere(RADIUS);
   private final Sphere goalSphere = new Sphere(RADIUS);
   private boolean isStartCurrentlyShown;
   private boolean isGoalCurrentlyShown;

   public enum StartGoalViewMode
   {
      TRANSLUCENT, OPAQUE
   };

   public static final PhongMaterial startOpaqueMaterial = new PhongMaterial(Color.GREEN);
   public static final PhongMaterial startTransparentMaterial = new PhongMaterial(toTransparentColor(Color.ORANGE, 0.7));
   public static final PhongMaterial goalOpaqueMaterial = new PhongMaterial(Color.RED);
   public static final PhongMaterial goalTransparentMaterial = new PhongMaterial(toTransparentColor(Color.ORANGE, 0.7));

   private AtomicReference<Boolean> showStartPosition = null;
   private AtomicReference<Boolean> showGoalPosition = null;
   private AtomicReference<Boolean> startEditModeEnabled = null;
   private AtomicReference<Boolean> goalEditModeEnabled = null;
   private AtomicReference<Point3D> startPositionReference = null;
   private AtomicReference<Point3D> goalPositionReference = null;

   private final REAMessager messager;

   public StartGoalPositionViewer(REAMessager messager)
   {
      this.messager = messager;
      startSphere.setMouseTransparent(true);
      goalSphere.setMouseTransparent(true);

      showStart(true);
      showGoal(true);
   }

   public StartGoalPositionViewer(REAMessager messager, Topic<Boolean> startEditModeEnabledTopic, Topic<Boolean> goalEditModeEnabledTopic,
                                  Topic<Point3D> startPositionTopic, Topic<Point3D> goalPositionTopic)
   {
      this(messager);
      setEditStartGoalTopics(startEditModeEnabledTopic, goalEditModeEnabledTopic);
      setPositionStartGoalTopics(startPositionTopic, goalPositionTopic);
   }

   public void setPositionStartGoalTopics(Topic<Point3D> startPositionTopic, Topic<Point3D> goalPositionTopic)
   {
      startPositionReference = messager.createInput(startPositionTopic, new Point3D());
      goalPositionReference = messager.createInput(goalPositionTopic, new Point3D());
   }

   public void setEditStartGoalTopics(Topic<Boolean> startEditModeEnabledTopic, Topic<Boolean> goalEditModeEnabledTopic)
   {
      startEditModeEnabled = messager.createInput(startEditModeEnabledTopic, false);
      goalEditModeEnabled = messager.createInput(goalEditModeEnabledTopic, false);
   }

   public void setShowStartGoalTopics(Topic<Boolean> showStartTopic, Topic<Boolean> showGoalTopic)
   {
      showStartPosition = messager.createInput(showStartTopic, true);
      showGoalPosition = messager.createInput(showGoalTopic, true);
   }

   @Override
   public void handle(long now)
   {
      if (showStartPosition != null)
         showStart(showStartPosition.get());

      if (showGoalPosition != null)
         showGoal(showGoalPosition.get());

      if (startEditModeEnabled != null && startEditModeEnabled.get())
         startSphere.setMaterial(startTransparentMaterial);
      else
         startSphere.setMaterial(startOpaqueMaterial);

      if (startPositionReference != null)
      {
         Point3D startPosition = startPositionReference.get();
         if (startPosition != null)
         {
            setStartPosition(startPosition);
         }
      }

      if (goalEditModeEnabled != null && goalEditModeEnabled.get())
         goalSphere.setMaterial(goalTransparentMaterial);
      else
         goalSphere.setMaterial(goalOpaqueMaterial);

      if (goalPositionReference != null)
      {
         Point3D goalPosition = goalPositionReference.get();
         if (goalPosition != null)
         {
            setGoalPosition(goalPosition);
         }
      }
   }

   private void setStartPosition(Point3D position)
   {
      startSphere.setTranslateX(position.getX());
      startSphere.setTranslateY(position.getY());
      startSphere.setTranslateZ(position.getZ());
   }

   private void setGoalPosition(Point3D position)
   {
      goalSphere.setTranslateX(position.getX());
      goalSphere.setTranslateY(position.getY());
      goalSphere.setTranslateZ(position.getZ());
   }

   private void showStart(boolean show)
   {
      if (show)
      {
         if (!isStartCurrentlyShown)
            rootChildren.add(startSphere);
      }
      else
      {
         if (isStartCurrentlyShown)
            rootChildren.remove(startSphere);
      }
      isStartCurrentlyShown = show;
   }

   private void showGoal(boolean show)
   {
      if (show)
      {
         if (!isGoalCurrentlyShown)
            rootChildren.add(goalSphere);
      }
      else
      {
         if (isGoalCurrentlyShown)
            rootChildren.remove(goalSphere);
      }
      isGoalCurrentlyShown = show;
   }

   public static Color toTransparentColor(Color opaqueColor, double opacity)
   {
      double red = opaqueColor.getRed();
      double green = opaqueColor.getGreen();
      double blue = opaqueColor.getBlue();
      return new Color(red, green, blue, opacity);
   }

   public Node getRoot()
   {
      return root;
   }
}
