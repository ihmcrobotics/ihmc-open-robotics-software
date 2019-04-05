package us.ihmc.humanoidBehaviors.ui.editors;

import javafx.application.Platform;
import javafx.event.EventHandler;
import javafx.scene.Node;
import javafx.scene.SubScene;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.PickResult;
import javafx.scene.shape.MeshView;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.tools.thread.ActivationReference;
import us.ihmc.humanoidBehaviors.tools.thread.TypedNotification;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.model.FXUIStateMachine;
import us.ihmc.humanoidBehaviors.ui.model.FXUIStateTransitionTrigger;
import us.ihmc.humanoidBehaviors.ui.model.interfaces.PositionEditable;
import us.ihmc.humanoidBehaviors.ui.tools.PrivateAnimationTimer;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;

import java.util.concurrent.atomic.AtomicReference;

public class SnappedPositionEditor
{
   private final Messager messager;
   private final SubScene subScene;

   // these are necessary to keep a consistent reference
   private final EventHandler<MouseEvent> mouseMoved = this::mouseMoved;
   private final EventHandler<MouseEvent> mouseClicked = this::mouseClicked;

   private final ActivationReference<Object> activeEditor;
   private final AtomicReference<FXUIStateMachine> activeStateMachine;

   private final TypedNotification<Point3D> mouseMovedMeshIntersection = new TypedNotification<>();
   private final TypedNotification<Point3D> mouseClickedMeshIntersection = new TypedNotification<>();
   private final Notification mouseRightClicked = new Notification();

   private final FXUIStateMachine positionEditorStateMachine;
   private PositionEditable selectedGraphic;
   private PrivateAnimationTimer positioningAnimationTimer;

   public SnappedPositionEditor(Messager messager, SubScene subScene)
   {
      this.messager = messager;
      this.subScene = subScene;

      activeEditor = new ActivationReference<>(messager.createInput(BehaviorUI.API.ActiveEditor, null), this);
      activeStateMachine = messager.createInput(BehaviorUI.API.ActiveStateMachine, null);

      positionEditorStateMachine = new FXUIStateMachine(messager, FXUIStateTransitionTrigger.POSITION_LEFT_CLICK, trigger ->
      {
         messager.submitMessage(BehaviorUI.API.ActiveEditor, BehaviorUI.SNAPPED_POSITION_EDITOR);
      });
      positionEditorStateMachine.mapTransition(FXUIStateTransitionTrigger.POSITION_LEFT_CLICK, trigger ->
      {
         messager.submitMessage(BehaviorUI.API.ActiveEditor, null);
      });
   }

   public void activateForSinglePoint(PositionEditable selectedGraphic)
   {
      messager.submitMessage(BehaviorUI.API.ActiveStateMachine, positionEditorStateMachine);
      positionEditorStateMachine.start();

      activate(selectedGraphic);
   }

   public void activate(PositionEditable selectedGraphic)
   {
      this.selectedGraphic = selectedGraphic;

      LogTools.debug("Snapped position editor activated");

      subScene.addEventHandler(MouseEvent.MOUSE_MOVED, mouseMoved);
      subScene.addEventHandler(MouseEvent.MOUSE_CLICKED, mouseClicked);
      this.selectedGraphic.setMouseTransparent(true);

      positioningAnimationTimer = new PrivateAnimationTimer(this::handlePositioning);
      positioningAnimationTimer.start();
   }

   private void handlePositioning(double now)
   {
      mouseMovedMeshIntersection.poll();
      mouseClickedMeshIntersection.poll();

      if (mouseClickedMeshIntersection.hasNext())  // use the clicked position if clicked
      {
         selectedGraphic.setPosition(mouseClickedMeshIntersection.read());
      }
      else if (mouseMovedMeshIntersection.hasNext())  // just for selection preview
      {
         selectedGraphic.setPosition(mouseMovedMeshIntersection.read());
      }

      if (mouseClickedMeshIntersection.hasNext())
      {
         LogTools.debug("Selected position is validated: {}", mouseClickedMeshIntersection.read());
         deactivate();
         activeStateMachine.get().transition(FXUIStateTransitionTrigger.POSITION_LEFT_CLICK);
      }

      if (mouseRightClicked.poll())
      {
         deactivate();
         activeStateMachine.get().transition(FXUIStateTransitionTrigger.RIGHT_CLICK);
      }
   }

   private void deactivate()
   {
      positioningAnimationTimer.stop();

      LogTools.debug("Snapped position editor deactivated.");
      subScene.removeEventHandler(MouseEvent.MOUSE_MOVED, mouseMoved);
      subScene.removeEventHandler(MouseEvent.MOUSE_CLICKED, mouseClicked);
      selectedGraphic.setMouseTransparent(false);
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
      if (!event.isConsumed() && event.isStillSincePress())
      {
         if (activeEditor.peekActivated())
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
