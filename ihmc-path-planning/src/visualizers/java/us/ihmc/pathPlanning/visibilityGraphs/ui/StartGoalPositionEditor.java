package us.ihmc.pathPlanning.visibilityGraphs.ui;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.event.EventHandler;
import javafx.scene.Node;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.PickResult;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class StartGoalPositionEditor extends AnimationTimer
{
   private final EventHandler<MouseEvent> rayCastInterceptor;
   private boolean isRayCastInterceptorAttached = false;
   private final AtomicReference<Point3D> latestInterception = new AtomicReference<>(null);
   private final AtomicReference<PlanarRegion> selectedRegion = new AtomicReference<>(null);

   private final EventHandler<MouseEvent> leftClickInterceptor;
   private boolean isLeftClickInterceptorAttached = false;
   private final AtomicBoolean positionValidated = new AtomicBoolean(false);

   private final Messager messager;
   private final Node sceneNode;

   private final AtomicReference<Boolean> startEditModeEnabled;
   private final AtomicReference<Boolean> goalEditModeEnabled;
   private final AtomicReference<PlanarRegionsList> planarRegionsList;

   private final Topic<Boolean> startEditModeEnabledTopic;
   private final Topic<Boolean> goalEditModeEnabledTopic;
   private final Topic<Point3D> startPositionTopic;
   private final Topic<Point3D> goalPositionTopic;
   private final Topic<PlanarRegion> selectedRegionTopic;

   private final Topic<Boolean> startOrientationEditModeEnabledTopic;
   private final Topic<Boolean> goalOrientationEditModeEnabledTopic;

   public StartGoalPositionEditor(Messager messager, Node sceneNode, Topic<Boolean> startEditModeEnabledTopic, Topic<Boolean> goalEditModeEnabledTopic,
                                  Topic<Point3D> startPositionTopic, Topic<Point3D> goalPositionTopic)
   {
      this(messager, sceneNode, startEditModeEnabledTopic, goalEditModeEnabledTopic, startPositionTopic, goalPositionTopic, null, null, null, null);
   }

   public StartGoalPositionEditor(Messager messager, Node sceneNode, Topic<Boolean> startEditModeEnabledTopic, Topic<Boolean> goalEditModeEnabledTopic,
                                  Topic<Point3D> startPositionTopic, Topic<Point3D> goalPositionTopic, Topic<PlanarRegionsList> planarRegionDataTopic,
                                  Topic<PlanarRegion> selectedRegionTopic, Topic<Boolean> startOrientationEditModeEnabledTopic,
                                  Topic<Boolean> goalOrientationEditModeEnabledTopic)
   {
      this.messager = messager;
      this.sceneNode = sceneNode;

      this.startEditModeEnabledTopic = startEditModeEnabledTopic;
      this.goalEditModeEnabledTopic = goalEditModeEnabledTopic;
      this.startPositionTopic = startPositionTopic;
      this.goalPositionTopic = goalPositionTopic;
      this.selectedRegionTopic = selectedRegionTopic;
      this.startOrientationEditModeEnabledTopic = startOrientationEditModeEnabledTopic;
      this.goalOrientationEditModeEnabledTopic = goalOrientationEditModeEnabledTopic;

      startEditModeEnabled = messager.createInput(startEditModeEnabledTopic, false);
      goalEditModeEnabled = messager.createInput(goalEditModeEnabledTopic, false);

      if (planarRegionDataTopic != null)
      {
         planarRegionsList = messager.createInput(planarRegionDataTopic);
      }
      else
      {
         planarRegionsList = null;
      }

      rayCastInterceptor = new EventHandler<MouseEvent>()
      {
         @Override
         public void handle(MouseEvent event)
         {
            PickResult pickResult = event.getPickResult();
            Node intersectedNode = pickResult.getIntersectedNode();
            if (intersectedNode == null || !(intersectedNode instanceof MeshView))
               return;
            javafx.geometry.Point3D localPoint = pickResult.getIntersectedPoint();
            javafx.geometry.Point3D scenePoint = intersectedNode.getLocalToSceneTransform().transform(localPoint);

            Point3D interception = new Point3D();
            interception.setX(scenePoint.getX());
            interception.setY(scenePoint.getY());
            interception.setZ(scenePoint.getZ());

            latestInterception.set(interception);

            if (planarRegionsList != null)
            {
               PlanarRegion region = findRegion(planarRegionsList.get(), interception);
               if (region == null)
                  return;

               selectedRegion.set(region);
            }
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

   private static PlanarRegion findRegion(PlanarRegionsList planarRegionsList, Point3D point)
   {
      for (PlanarRegion region : planarRegionsList.getPlanarRegionsAsList())
      {
         if (PlanarRegionTools.isPointOnRegion(region, point, 1.0e-5))
         {
            return region;
         }
      }
      return null;
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
            if (selectedRegionTopic != null)
            {
               messager.submitMessage(selectedRegionTopic, selectedRegion.get());
            }
            messager.submitMessage(startPositionTopic, interception);
         }

         if (positionValidated.getAndSet(false))
         {
            LogTools.debug("Start position is validated: " + interception, this);
            messager.submitMessage(startEditModeEnabledTopic, false);
            if (startOrientationEditModeEnabledTopic != null)
            {
               LogTools.debug("submitMessage  startOrientationEditModeEnabledTopic");
               messager.submitMessage(startOrientationEditModeEnabledTopic, true);
            }
         }
         return;
      }

      if (goalEditModeEnabled.get())
      {
         Point3D interception = latestInterception.getAndSet(null);
         if (interception != null)
         {
            if (selectedRegionTopic != null)
            {
               messager.submitMessage(selectedRegionTopic, selectedRegion.get());
            }
            messager.submitMessage(goalPositionTopic, interception);
         }

         if (positionValidated.getAndSet(false))
         {
            LogTools.debug("Goal position is validated: " + interception, this);
            messager.submitMessage(goalEditModeEnabledTopic, false);
            if (goalOrientationEditModeEnabledTopic != null)
            {
               messager.submitMessage(goalOrientationEditModeEnabledTopic, true);
            }
         }
      }
   }

   private void attachEvenHandlers()
   {
      if (!isRayCastInterceptorAttached)
      {
         LogTools.debug("Attaching ray cast event handler.", this);
         sceneNode.addEventHandler(MouseEvent.ANY, rayCastInterceptor);
         isRayCastInterceptorAttached = true;
      }
      if (!isLeftClickInterceptorAttached)
      {
         LogTools.debug("Attaching left click event handler.", this);
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
