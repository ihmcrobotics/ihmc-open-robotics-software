package us.ihmc.pathPlanning.visibilityGraphs.ui;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.event.EventHandler;
import javafx.scene.Node;
import javafx.scene.SubScene;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.PickResult;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;

public class VisibilityGraphStartGoalEditor extends AnimationTimer
{
   private static final boolean VERBOSE = true;

   private final EventHandler<MouseEvent> rayCastInterceptor;
   private boolean isRayCastInterceptorAttached = false;
   private final AtomicReference<Point3D> latestInterception = new AtomicReference<>(null);

   private final EventHandler<MouseEvent> leftClickInterceptor;
   private boolean isLeftClickInterceptorAttached = false;
   private final AtomicBoolean positionValidated = new AtomicBoolean(false);

   private final REAMessager messager;
   private final Node sceneNode;

   private final AtomicReference<Boolean> startEditModeEnabled;
   private final AtomicReference<Boolean> goalEditModeEnabled;

   public VisibilityGraphStartGoalEditor(REAMessager messager, Node sceneNode)
   {
      this.messager = messager;
      this.sceneNode = sceneNode;

      startEditModeEnabled = messager.createInput(UIVisibilityGraphsTopics.StartEditModeEnabled, false);
      goalEditModeEnabled = messager.createInput(UIVisibilityGraphsTopics.GoalEditModeEnabled, false);

      rayCastInterceptor = new EventHandler<MouseEvent>()
      {
         @Override
         public void handle(MouseEvent event)
         {
            PickResult pickResult = event.getPickResult();
            Node intersectedNode = pickResult.getIntersectedNode();
            if (intersectedNode == null || intersectedNode instanceof SubScene)
               return;
            javafx.geometry.Point3D localPoint = pickResult.getIntersectedPoint();
            javafx.geometry.Point3D scenePoint = intersectedNode.getLocalToSceneTransform().transform(localPoint);

            Point3D interception = new Point3D();
            interception.setX(scenePoint.getX());
            interception.setY(scenePoint.getY());
            interception.setZ(scenePoint.getZ());

            latestInterception.set(interception);
         }
      };

      leftClickInterceptor = new EventHandler<MouseEvent>()
      {
         @Override
         public void handle(MouseEvent event)
         {
            if (event.getButton() != MouseButton.PRIMARY)
               return;

            if (event.isStillSincePress() && event.getEventType() == MouseEvent.MOUSE_CLICKED)
               positionValidated.set(true);
         }
      };
   }

   @Override
   public void handle(long now)
   {
      if (startEditModeEnabled.get() && goalEditModeEnabled.get())
         throw new RuntimeException("Cannot edit start AND goal together.");

      if (startEditModeEnabled.get() || goalEditModeEnabled.get())
      {
         attachEvenHandlers();
      }
      else
      {
         removeEventHandlers();
         return;
      }

      if (startEditModeEnabled.get())
      {
         Point3D interception = latestInterception.getAndSet(null);
         if (interception != null)
         {
            messager.submitMessage(UIVisibilityGraphsTopics.StartPosition, interception);
         }

         if (positionValidated.getAndSet(false))
         {
            if (VERBOSE)
               PrintTools.info(this, "Start position is validated: " + interception);
            messager.submitMessage(UIVisibilityGraphsTopics.StartEditModeEnabled, false);
         }
         return;
      }

      if (goalEditModeEnabled.get())
      {
         Point3D interception = latestInterception.getAndSet(null);
         if (interception != null)
         {
            messager.submitMessage(UIVisibilityGraphsTopics.GoalPosition, interception);
         }

         if (positionValidated.getAndSet(false))
         {
            if (VERBOSE)
               PrintTools.info(this, "Goal position is validated: " + interception);
            messager.submitMessage(UIVisibilityGraphsTopics.GoalEditModeEnabled, false);
         }
      }
   }

   private void attachEvenHandlers()
   {
      if (!isRayCastInterceptorAttached)
      {
         if (VERBOSE)
            PrintTools.info(this, "Attaching ray cast event handler.");
         sceneNode.addEventHandler(MouseEvent.ANY, rayCastInterceptor);
         isRayCastInterceptorAttached = true;
      }
      if (!isLeftClickInterceptorAttached)
      {
         if (VERBOSE)
            PrintTools.info(this, "Attaching left click event handler.");
         sceneNode.addEventHandler(MouseEvent.ANY, leftClickInterceptor);
         isLeftClickInterceptorAttached = true;
      }
   }

   private void removeEventHandlers()
   {
      if (isRayCastInterceptorAttached)
      {
         sceneNode.removeEventHandler(MouseEvent.ANY, rayCastInterceptor);
         isRayCastInterceptorAttached = false;
      }
      if (isLeftClickInterceptorAttached)
      {
         sceneNode.removeEventHandler(MouseEvent.ANY, leftClickInterceptor);
         isLeftClickInterceptorAttached = false;
      }
   }
}
