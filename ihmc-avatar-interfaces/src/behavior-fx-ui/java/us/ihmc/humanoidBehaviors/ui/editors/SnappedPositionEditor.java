package us.ihmc.humanoidBehaviors.ui.editors;

import javafx.event.EventHandler;
import javafx.scene.Node;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.PickResult;
import javafx.scene.shape.MeshView;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.model.FXUIStateTransitionTrigger;
import us.ihmc.humanoidBehaviors.ui.model.interfaces.PositionEditable;
import us.ihmc.humanoidBehaviors.ui.tools.PrivateAnimationTimer;
import us.ihmc.log.LogTools;
import us.ihmc.tools.thread.TypedNotification;

import java.util.function.Consumer;

public class SnappedPositionEditor
{
   private final Node sceneNode;
   private final PrivateAnimationTimer positioningAnimationTimer;

   // these are necessary to keep a consistent reference
   private final EventHandler<MouseEvent> mouseMoved = this::mouseMoved;
   private final EventHandler<MouseEvent> mouseClicked = this::mouseClicked;

   private final TypedNotification<Point3D> mouseMovedMeshIntersection = new TypedNotification<>();
   private final TypedNotification<Point3D> mouseClickedMeshIntersection = new TypedNotification<>();
   private final Notification mouseRightClicked = new Notification();

   private PositionEditable selectedGraphic;
   private Consumer<FXUIStateTransitionTrigger> onExit;

   public SnappedPositionEditor(Node sceneNode)
   {
      this.sceneNode = sceneNode;

      positioningAnimationTimer = new PrivateAnimationTimer(this::handlePositioning);
   }

   public void edit(PositionEditable selectedGraphic, Consumer<FXUIStateTransitionTrigger> onExit)
   {
      this.selectedGraphic = selectedGraphic;
      this.onExit = onExit;

      BehaviorUI.claimEditing(this);

      sceneNode.addEventHandler(MouseEvent.MOUSE_MOVED, mouseMoved);
      sceneNode.addEventHandler(MouseEvent.MOUSE_CLICKED, mouseClicked);
      this.selectedGraphic.setMouseTransparent(true);

      positioningAnimationTimer.start();
   }

   private void handlePositioning(double now)
   {
      mouseMovedMeshIntersection.poll();
      mouseClickedMeshIntersection.poll();

      if (mouseClickedMeshIntersection.hasNext())  // use the clicked position if clicked
      {
         selectedGraphic.setPosition(mouseClickedMeshIntersection.peek());
      }
      else if (mouseMovedMeshIntersection.hasNext())  // just for selection preview
      {
         selectedGraphic.setPosition(mouseMovedMeshIntersection.peek());
      }

      if (mouseClickedMeshIntersection.hasNext())
      {
         LogTools.debug("Selected position is validated: {}", mouseClickedMeshIntersection.peek());
         deactivate(FXUIStateTransitionTrigger.POSITION_LEFT_CLICK);
      }

      if (mouseRightClicked.poll())
      {
         deactivate(FXUIStateTransitionTrigger.RIGHT_CLICK);
      }
   }

   private void deactivate(FXUIStateTransitionTrigger exitType)
   {
      positioningAnimationTimer.stop();

      LogTools.debug("Snapped position editor deactivated.");
      sceneNode.removeEventHandler(MouseEvent.MOUSE_MOVED, mouseMoved);
      sceneNode.removeEventHandler(MouseEvent.MOUSE_CLICKED, mouseClicked);
      selectedGraphic.setMouseTransparent(false);

      BehaviorUI.ACTIVE_EDITOR = null; // do this before exit because it probably switches to new editor

      onExit.accept(exitType);
   }

   private void mouseMoved(MouseEvent event)
   {
      Point3D intersection = calculateMouseIntersection(event);
      if (intersection != null)
      {
         mouseMovedMeshIntersection.add(intersection);
      }
   }

   private void mouseClicked(MouseEvent event)
   {
      if (!event.isConsumed() && event.isStillSincePress() && BehaviorUI.ACTIVE_EDITOR == this)
      {
         LogTools.debug("consume mouseClicked");
         event.consume();
         if (event.getButton() == MouseButton.PRIMARY)
         {
            Point3D intersection = calculateMouseIntersection(event);
            if (intersection != null)
            {
               mouseClickedMeshIntersection.add(intersection);
            }
            else
            {
               LogTools.debug("Click mesh couldn't be found");
            }
         }
         else if (event.getButton() == MouseButton.SECONDARY)  // maybe move this to patrol controller? or implement cancel
         {
            mouseRightClicked.set();
         }
      }
   }

   private Point3D calculateMouseIntersection(MouseEvent event)
   {
      PickResult pickResult = event.getPickResult();
      Node intersectedNode = pickResult.getIntersectedNode();

      if (intersectedNode != null && intersectedNode instanceof MeshView) // TODO make sure it's a planar region
      {
         javafx.geometry.Point3D localPoint = pickResult.getIntersectedPoint();
         javafx.geometry.Point3D scenePoint = intersectedNode.getLocalToSceneTransform().transform(localPoint);

         Point3D intersection = new Point3D();
         intersection.setX(scenePoint.getX());
         intersection.setY(scenePoint.getY());
         intersection.setZ(scenePoint.getZ());

         return intersection;
      }
      else
      {
         return null;
      }
   }
}
