package us.ihmc.humanoidBehaviors.ui.editors;

import javafx.scene.Node;
import javafx.scene.SubScene;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.PickResult;
import javafx.scene.shape.MeshView;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.model.FXUIEditor;
import us.ihmc.humanoidBehaviors.ui.model.FXUIStateMachine;
import us.ihmc.humanoidBehaviors.ui.model.FXUIStateTransitionTrigger;
import us.ihmc.humanoidBehaviors.ui.model.interfaces.PositionEditable;
import us.ihmc.humanoidBehaviors.ui.references.OverTypedReference;
import us.ihmc.humanoidBehaviors.ui.references.TypedNotification;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;

public class SnappedPositionEditor extends FXUIEditor
{
   private final TypedNotification<Point3D> mouseMovedMeshIntersection = new TypedNotification<>();
   private final TypedNotification<Point3D> mouseClickedMeshIntersection = new TypedNotification<>();
   private final Notification mouseRightClicked = new Notification();

   private final FXUIStateMachine positionEditorStateMachine;
   private final OverTypedReference<PositionEditable> selectedGraphicReference;

   public SnappedPositionEditor(Messager messager, SubScene subScene)
   {
      super(messager, subScene);

      positionEditorStateMachine = new FXUIStateMachine(messager, FXUIStateTransitionTrigger.POSITION_LEFT_CLICK, trigger ->
      {
         messager.submitMessage(BehaviorUI.API.ActiveEditor, BehaviorUI.SNAPPED_POSITION_EDITOR);
      });
      positionEditorStateMachine.mapTransition(FXUIStateTransitionTrigger.POSITION_LEFT_CLICK, trigger ->
      {
         messager.submitMessage(BehaviorUI.API.ActiveEditor, null);
         messager.submitMessage(BehaviorUI.API.SelectedGraphic, null);
      });

      selectedGraphicReference = new OverTypedReference<>(messager.createInput(BehaviorUI.API.SelectedGraphic));
   }

   public void activate()
   {
      messager.submitMessage(BehaviorUI.API.ActiveStateMachine, positionEditorStateMachine);
      positionEditorStateMachine.start();
   }

   @Override
   public void handle(long now)
   {
      if (activeEditor.pollActivated())
      {
         if (activeEditor.activationChanged())
         {
            LogTools.debug("Snapped position editor activated");
            subScene.addEventHandler(MouseEvent.MOUSE_MOVED, mouseMoved);
            subScene.addEventHandler(MouseEvent.MOUSE_CLICKED, mouseClicked);
            selectedGraphicReference.get().setMouseTransparent(true);
         }

         mouseMovedMeshIntersection.poll();
         mouseClickedMeshIntersection.poll();

         if (mouseClickedMeshIntersection.hasNext())  // use the clicked position if clicked
         {
            selectedGraphicReference.get().setPosition(mouseClickedMeshIntersection.read());
         }
         else if (mouseMovedMeshIntersection.hasNext())  // just for selection preview
         {
            selectedGraphicReference.get().setPosition(mouseMovedMeshIntersection.read());
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
   }

   private void deactivate()
   {
      LogTools.debug("Snapped position editor deactivated.");
      subScene.removeEventHandler(MouseEvent.MOUSE_MOVED, mouseMoved);
      subScene.removeEventHandler(MouseEvent.MOUSE_CLICKED, mouseClicked);
      selectedGraphicReference.get().setMouseTransparent(false);
   }

   @Override
   protected void mouseMoved(MouseEvent event)
   {
      Point3D intersection = calculateMouseIntersection(event);
      if (intersection != null)
      {
         mouseMovedMeshIntersection.add(intersection);
      }
   }

   @Override
   protected void mouseClicked(MouseEvent event)
   {
      if (event.isStillSincePress())
      {
         LogTools.debug("{} mouseClicked", getClass().getSimpleName());
         if (activeEditor.peekActivated())
         {
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

   public Point3D calculateMouseIntersection(MouseEvent event)
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
