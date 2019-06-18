package us.ihmc.quadrupedFootstepPlanning.ui.components;

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
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class StartGoalOrientationEditor extends AnimationTimer
{
   private static final boolean VERBOSE = false;

   private final EventHandler<MouseEvent> rayCastInterceptor;
   private final EventHandler<MouseEvent> leftClickInterceptor;

   private boolean isRayCastInterceptorAttached = false;
   private boolean isLeftClickInterceptorAttached = false;

   private final AtomicReference<Point3D> latestInterception = new AtomicReference<>(null);
   private final AtomicReference<Point3D> startPositionReference;
   private final AtomicReference<Point3D> goalPositionReference;

   private final AtomicBoolean orientationValidated = new AtomicBoolean(false);

   private final Messager messager;
   private final SubScene subScene;

   private final AtomicReference<Boolean> startEditModeEnabled;
   private final AtomicReference<Boolean> goalEditModeEnabled;

   private final Topic<Boolean> editModeEnabledTopic;
   private final Topic<Boolean> startOrientationEditModeEnabledTopic;
   private final Topic<Boolean> goalOrientationEditModeEnabledTopic;
   private final Topic<Quaternion> startOrientationTopic;
   private final Topic<Quaternion> goalOrientationTopic;

   public StartGoalOrientationEditor(Messager messager, SubScene subScene, Topic<Boolean> editModeEnabledTopic,
                                     Topic<Boolean> startOrientationEditModeEnabledTopic, Topic<Boolean> goalOrientationEditModeEnabledTopic,
                                     Topic<Point3D> startPositionTopic, Topic<Quaternion> startOrientationTopic, Topic<Point3D> goalPositionTopic,
                                     Topic<Quaternion> goalOrientationTopic, Topic<PlanarRegion> selectedRegionTopic)
   {
      this.messager = messager;
      this.subScene = subScene;
      this.editModeEnabledTopic = editModeEnabledTopic;
      this.startOrientationEditModeEnabledTopic = startOrientationEditModeEnabledTopic;
      this.goalOrientationEditModeEnabledTopic = goalOrientationEditModeEnabledTopic;
      this.startOrientationTopic = startOrientationTopic;
      this.goalOrientationTopic = goalOrientationTopic;

      startEditModeEnabled = messager.createInput(startOrientationEditModeEnabledTopic, false);
      goalEditModeEnabled = messager.createInput(goalOrientationEditModeEnabledTopic, false);

      startPositionReference = messager.createInput(startPositionTopic);
      goalPositionReference = messager.createInput(goalPositionTopic);

      AtomicReference<PlanarRegion> selectedRegionReference = messager.createInput(selectedRegionTopic);

      rayCastInterceptor = (event) ->
      {
         latestInterception.set(intersectRayWithPlane(subScene.getCamera(), selectedRegionReference.get(), event));
      };

      leftClickInterceptor = (event) ->
      {
         if (event.getButton() != MouseButton.PRIMARY)
            return;

         if (event.isStillSincePress() && event.getEventType() == MouseEvent.MOUSE_CLICKED)
            orientationValidated.set(true);
      };
   }

   private static Point3D intersectRayWithPlane(Camera camera, PlanarRegion planarRegion, MouseEvent event)
   {
      Line3D line = getPickRay(camera, event);

      RigidBodyTransform regionTransform = new RigidBodyTransform();
      planarRegion.getTransformToWorld(regionTransform);
      Vector3D planeNormal = planarRegion.getNormal();
      Point3D pointOnPlane = new Point3D();
      regionTransform.getTranslation(pointOnPlane);

      return EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(pointOnPlane, planeNormal, line.getPoint(), line.getDirection());
   }

   private static Line3D getPickRay(Camera camera, MouseEvent event)
   {
      Point3D point1 = new Point3D();
      point1.setX(camera.getLocalToSceneTransform().getTx());
      point1.setY(camera.getLocalToSceneTransform().getTy());
      point1.setZ(camera.getLocalToSceneTransform().getTz());

      Point3D point2 = new Point3D();
      javafx.geometry.Point3D pointOnProjectionPlane = CameraHelper.pickProjectPlane(camera, event.getSceneX(), event.getSceneY());
      point2.setX(pointOnProjectionPlane.getX());
      point2.setY(pointOnProjectionPlane.getY());
      point2.setZ(pointOnProjectionPlane.getZ());

      return new Line3D(point1, point2);
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
            Quaternion orientation = new Quaternion(startYaw, 0.0, 0.0);

            messager.submitMessage(startOrientationTopic, orientation);
         }

         if(orientationValidated.getAndSet(false))
         {
            messager.submitMessage(startOrientationEditModeEnabledTopic, false);
            messager.submitMessage(editModeEnabledTopic, false);
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
            Quaternion orientation = new Quaternion(goalYaw, 0.0, 0.0);

            messager.submitMessage(goalOrientationTopic, orientation);
         }

         if(orientationValidated.getAndSet(false))
         {
            messager.submitMessage(goalOrientationEditModeEnabledTopic, false);
            messager.submitMessage(editModeEnabledTopic, false);
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
