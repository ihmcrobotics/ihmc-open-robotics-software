package us.ihmc.footstepPlanning.ui;

import javafx.animation.AnimationTimer;
import javafx.event.EventHandler;
import javafx.scene.Node;
import javafx.scene.SubScene;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.PickResult;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class StartGoalOrientationEditor extends AnimationTimer
{
   private static final boolean VERBOSE = true;

   private final EventHandler<MouseEvent> rayCastInterceptor;
   private final EventHandler<MouseEvent> leftClickInterceptor;

   private boolean isRayCastInterceptorAttached = false;
   private boolean isLeftClickInterceptorAttached = false;

   private final AtomicReference<Point3D> latestInterception = new AtomicReference<>(null);
   private final AtomicReference<Point3D> startPositionReference;
   private final AtomicReference<Point3D> goalPositionReference;

   private final AtomicBoolean orientationValidated = new AtomicBoolean(false);

   private final REAMessager messager;
   private final SubScene subScene;

   private final AtomicReference<Boolean> startEditModeEnabled;
   private final AtomicReference<Boolean> goalEditModeEnabled;

   public StartGoalOrientationEditor(REAMessager messager, SubScene subScene)
   {
      this.messager = messager;
      this.subScene = subScene;

      startEditModeEnabled = messager.createInput(FootstepPlannerUserInterfaceAPI.StartOrientationEditModeEnabledTopic, false);
      goalEditModeEnabled = messager.createInput(FootstepPlannerUserInterfaceAPI.GoalOrientationEditModeEnabledTopic, false);

      startPositionReference = messager.createInput(FootstepPlannerUserInterfaceAPI.StartPositionTopic);
      goalPositionReference = messager.createInput(FootstepPlannerUserInterfaceAPI.GoalPositionTopic);

      rayCastInterceptor = (event) ->
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
      };

      leftClickInterceptor = (event) ->
      {
         if (event.getButton() != MouseButton.PRIMARY)
            return;

         if (event.isStillSincePress() && event.getEventType() == MouseEvent.MOUSE_CLICKED)
            orientationValidated.set(true);
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

      if(startEditModeEnabled.get())
      {
         Point3D startPosition = startPositionReference.get();
         Point3D interception = latestInterception.getAndSet(null);

         if(startPosition != null && interception != null)
         {
            Vector3D difference = new Vector3D();
            difference.sub(interception, startPosition);
            double startYaw = Math.atan2(difference.getY(), difference.getX());

            messager.submitMessage(FootstepPlannerUserInterfaceAPI.StartOrientationTopic, startYaw);
         }

         if(orientationValidated.getAndSet(false))
         {
            messager.submitMessage(FootstepPlannerUserInterfaceAPI.StartOrientationEditModeEnabledTopic, false);
         }
      }

      if(goalEditModeEnabled.get())
      {
         Point3D goalPosition = goalPositionReference.get();
         Point3D interception = latestInterception.getAndSet(null);

         if(goalPosition != null && interception != null)
         {
            Vector3D difference = new Vector3D();
            difference.sub(interception, goalPosition);
            double goalYaw = Math.atan2(difference.getY(), difference.getX());

            messager.submitMessage(FootstepPlannerUserInterfaceAPI.GoalOrientationTopic, goalYaw);
         }

         if(orientationValidated.getAndSet(false))
         {
            messager.submitMessage(FootstepPlannerUserInterfaceAPI.GoalOrientationEditModeEnabledTopic, false);
         }
      }
   }

   private void attachEvenHandlers()
   {
      if (!isRayCastInterceptorAttached)
      {
         if (VERBOSE)
            PrintTools.info(this, "Attaching ray cast event handler.");
         subScene.addEventHandler(MouseEvent.ANY, rayCastInterceptor);
         isRayCastInterceptorAttached = true;
      }
      if (!isLeftClickInterceptorAttached)
      {
         if (VERBOSE)
            PrintTools.info(this, "Attaching left click event handler.");
         subScene.addEventHandler(MouseEvent.ANY, leftClickInterceptor);
         isLeftClickInterceptorAttached = true;
      }
   }

   private void removeEventHandlers()
   {
      if (isRayCastInterceptorAttached)
      {
         subScene.removeEventHandler(MouseEvent.ANY, rayCastInterceptor);
         isRayCastInterceptorAttached = false;
      }
      if (isLeftClickInterceptorAttached)
      {
         subScene.removeEventHandler(MouseEvent.ANY, leftClickInterceptor);
         isLeftClickInterceptorAttached = false;
      }
   }
}
