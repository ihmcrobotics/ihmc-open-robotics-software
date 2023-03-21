package us.ihmc.pathPlanning.visibilityGraphs.ui;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.event.EventHandler;
import javafx.scene.Node;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.pathPlanning.visibilityGraphs.ui.eventHandlers.PlanarRegionSelector;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class StartGoalPositionEditor extends AnimationTimer
{
   private final PlanarRegionSelector planarRegionSelector = new PlanarRegionSelector();
   private boolean isRayCastInterceptorAttached = false;

   private final EventHandler<MouseEvent> leftClickInterceptor;
   private boolean isLeftClickInterceptorAttached = false;
   private final AtomicBoolean positionValidated = new AtomicBoolean(false);

   private final Messager messager;
   private final Node sceneNode;

   private final AtomicReference<Boolean> startEditModeEnabled;
   private final AtomicReference<Boolean> goalEditModeEnabled;

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

      if (startEditModeEnabledTopic != null)
         startEditModeEnabled = messager.createInput(startEditModeEnabledTopic, false);
      else
         startEditModeEnabled = null;
      goalEditModeEnabled = messager.createInput(goalEditModeEnabledTopic, false);

      if (planarRegionDataTopic != null)
      {
         messager.addTopicListener(planarRegionDataTopic, planarRegionSelector::setPlanarRegionsList);
      }

      leftClickInterceptor = mouseEvent ->
      {
         if (mouseEvent.getButton() != MouseButton.PRIMARY)
            return;

         if (mouseEvent.isStillSincePress() && mouseEvent.getEventType() == MouseEvent.MOUSE_CLICKED)
            positionValidated.set(true);
      };
   }

   @Override
   public void handle(long now)
   {
      if (startEditModeEnabled != null && startEditModeEnabled.get() && goalEditModeEnabled.get())
         throw new RuntimeException("Cannot edit start AND goal together.");

      if (startEditModeEnabled != null && startEditModeEnabled.get() || goalEditModeEnabled.get())
      {
         attachEvenHandlers();
      }
      else
      {
         removeEventHandlers();
         return;
      }

      if (startEditModeEnabled != null && startEditModeEnabled.get())
      {
         Point3D interception = planarRegionSelector.pollSelectedPoint();
         if (interception != null)
         {
            if (selectedRegionTopic != null)
            {
               messager.submitMessage(selectedRegionTopic, planarRegionSelector.getSelectedRegion());
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
         Point3D interception = planarRegionSelector.pollSelectedPoint();
         if (interception != null)
         {
            if (selectedRegionTopic != null)
            {
               messager.submitMessage(selectedRegionTopic, planarRegionSelector.getSelectedRegion());
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
         sceneNode.addEventHandler(MouseEvent.ANY, planarRegionSelector);
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
         sceneNode.removeEventHandler(MouseEvent.ANY, planarRegionSelector);
         isRayCastInterceptorAttached = false;
      }
      if (isLeftClickInterceptorAttached)
      {
         sceneNode.removeEventHandler(MouseEvent.ANY, leftClickInterceptor);
         isLeftClickInterceptorAttached = false;
      }
   }
}
