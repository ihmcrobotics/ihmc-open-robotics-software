package us.ihmc.footstepPlanning.ui.components;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import com.sun.javafx.scene.CameraHelper;

import javafx.animation.AnimationTimer;
import javafx.event.EventHandler;
import javafx.scene.Camera;
import javafx.scene.SubScene;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.eventHandlers.PlaneIntersectionCalculator;
import us.ihmc.robotics.geometry.PlanarRegion;

public class StartGoalOrientationEditor extends AnimationTimer
{
   private static final boolean VERBOSE = false;

   private final PlaneIntersectionCalculator planeIntersectionCalculator;
   private final EventHandler<MouseEvent> leftClickInterceptor;

   private boolean isRayCastInterceptorAttached = false;
   private boolean isLeftClickInterceptorAttached = false;

   private final AtomicReference<Point3D> startPositionReference;
   private final AtomicReference<Point3D> goalPositionReference;

   private final AtomicBoolean orientationValidated = new AtomicBoolean(false);

   private final Messager messager;
   private final SubScene subScene;

   private final AtomicReference<Boolean> startEditModeEnabled;
   private final AtomicReference<Boolean> goalEditModeEnabled;

   public StartGoalOrientationEditor(Messager messager, SubScene subScene)
   {
      this.messager = messager;
      this.subScene = subScene;

      startEditModeEnabled = messager.createInput(FootstepPlannerMessagerAPI.StartOrientationEditModeEnabled, false);
      goalEditModeEnabled = messager.createInput(FootstepPlannerMessagerAPI.GoalOrientationEditModeEnabled, false);

      startPositionReference = messager.createInput(FootstepPlannerMessagerAPI.StartPosition);
      goalPositionReference = messager.createInput(FootstepPlannerMessagerAPI.GoalPosition);

      planeIntersectionCalculator = new PlaneIntersectionCalculator(subScene.getCamera());
      messager.registerTopicListener(FootstepPlannerMessagerAPI.SelectedRegion, planeIntersectionCalculator::setPlanarRegion);

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
         Point3D interception = planeIntersectionCalculator.pollIntersection();

         if(startPosition != null && interception != null)
         {
            Vector3D difference = new Vector3D();
            difference.sub(interception, startPosition);
            double startYaw = Math.atan2(difference.getY(), difference.getX());
            Quaternion orientation = new Quaternion(startYaw, 0.0, 0.0);

            messager.submitMessage(FootstepPlannerMessagerAPI.StartOrientation, orientation);
         }

         if(orientationValidated.getAndSet(false))
         {
            messager.submitMessage(FootstepPlannerMessagerAPI.StartOrientationEditModeEnabled, false);
            messager.submitMessage(FootstepPlannerMessagerAPI.EditModeEnabled, false);
         }
      }

      if(goalEditModeEnabled.get())
      {
         Point3D goalPosition = goalPositionReference.get();
         Point3D interception = planeIntersectionCalculator.pollIntersection();

         if(goalPosition != null && interception != null)
         {
            Vector3D difference = new Vector3D();
            difference.sub(interception, goalPosition);
            double goalYaw = Math.atan2(difference.getY(), difference.getX());
            Quaternion orientation = new Quaternion(goalYaw, 0.0, 0.0);

            messager.submitMessage(FootstepPlannerMessagerAPI.GoalOrientation, orientation);
         }

         if(orientationValidated.getAndSet(false))
         {
            messager.submitMessage(FootstepPlannerMessagerAPI.GoalOrientationEditModeEnabled, false);
            messager.submitMessage(FootstepPlannerMessagerAPI.EditModeEnabled, false);
         }
      }
   }

   private void attachEvenHandlers()
   {
      if (!isRayCastInterceptorAttached)
      {
         if (VERBOSE)
            PrintTools.info(this, "Attaching ray cast event handler.");
         subScene.addEventHandler(MouseEvent.ANY, planeIntersectionCalculator);
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
         subScene.removeEventHandler(MouseEvent.ANY, planeIntersectionCalculator);
         isRayCastInterceptorAttached = false;
      }
      if (isLeftClickInterceptorAttached)
      {
         subScene.removeEventHandler(MouseEvent.ANY, leftClickInterceptor);
         isLeftClickInterceptorAttached = false;
      }
   }
}
