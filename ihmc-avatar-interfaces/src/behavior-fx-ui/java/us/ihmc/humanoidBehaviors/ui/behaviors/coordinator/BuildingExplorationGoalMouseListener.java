package us.ihmc.humanoidBehaviors.ui.behaviors.coordinator;

import javafx.animation.AnimationTimer;
import javafx.event.EventHandler;
import javafx.scene.Node;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.eventHandlers.PlanarRegionSelector;

import java.util.concurrent.atomic.AtomicBoolean;

public class BuildingExplorationGoalMouseListener extends AnimationTimer
{
   private final AtomicBoolean placingGoal = new AtomicBoolean();
   private final AtomicBoolean positionValidated = new AtomicBoolean();

   private final PlanarRegionSelector planarRegionSelector = new PlanarRegionSelector();
   private final EventHandler<MouseEvent> leftClickInterceptor;

   private boolean isRayCastInterceptorAttached = false;
   private boolean isLeftClickInterceptorAttached = false;

   private final Messager messager;
   private final Node sceneNode;

   public BuildingExplorationGoalMouseListener(Messager messager, Node sceneNode)
   {
      this.messager = messager;
      this.sceneNode = sceneNode;

      messager.registerTopicListener(BuildingExplorationBehaviorAPI.PlanarRegions, planarRegionSelector::setPlanarRegionsList);
      messager.registerTopicListener(BuildingExplorationBehaviorAPI.PlaceGoal, p -> placingGoal.set(true));

      leftClickInterceptor = mouseEvent ->
      {
         if (mouseEvent.getButton() != MouseButton.PRIMARY)
            return;

         if (mouseEvent.isStillSincePress() && mouseEvent.getEventType() == MouseEvent.MOUSE_CLICKED)
            placingGoal.set(false);
      };
   }

   @Override
   public void handle(long now)
   {
      if (placingGoal.get())
      {
         attachEventHandlers();
      }
      else
      {
         removeEventHandlers();
         return;
      }

      Point3D intersection = planarRegionSelector.pollSelectedPoint();
      if (intersection != null)
      {
         messager.submitMessage(BuildingExplorationBehaviorAPI.Goal, intersection);
      }
   }

   private void attachEventHandlers()
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
