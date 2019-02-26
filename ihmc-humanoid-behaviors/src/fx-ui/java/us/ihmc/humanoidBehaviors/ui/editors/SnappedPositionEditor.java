package us.ihmc.humanoidBehaviors.ui.editors;

import javafx.scene.Node;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.PickResult;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.Sphere;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.model.*;
import us.ihmc.humanoidBehaviors.ui.references.ActivationReference;
import us.ihmc.humanoidBehaviors.ui.references.NotificationReference;
import us.ihmc.humanoidBehaviors.ui.references.QueueReference;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;

import java.util.concurrent.atomic.AtomicReference;

public class SnappedPositionEditor extends FXUIEditor
{
   private final Messager messager;
   private final Node sceneNode;

   private final ActivationReference<FXUIEditor> activeEditor;
   private final QueueReference<Point3D> mouseMovedMeshIntersection = new QueueReference<>();
   private final QueueReference<Point3D> mouseClickedMeshIntersection = new QueueReference<>();
   private final NotificationReference mouseRightClicked = new NotificationReference();
   private final AtomicReference<FXUIStateMachine> activeStateMachine;

   private final FXUIStateMachine positionEditorStateMachine;

   public SnappedPositionEditor(Messager messager, Node sceneNode)
   {
      this.messager = messager;
      this.sceneNode = sceneNode;


      activeEditor = new ActivationReference<>(messager.createInput(BehaviorUI.API.ActiveEditor, null), this);
      activeStateMachine = messager.createInput(BehaviorUI.API.ActiveStateMachine, null);

      positionEditorStateMachine = new FXUIStateMachine(messager,
                                                        FXUIState.SNAPPED_POSITION_EDITOR,
                                                        FXUIStateTransition.SNAPPED_POSITION_LEFT_CLICK)
      {
         @Override
         protected void handleTransition(FXUIStateTransition transition)
         {
            if (transition.isStart())
            {
               messager.submitMessage(BehaviorUI.API.ActiveEditor, BehaviorUI.SNAPPED_POSITION_EDITOR);
            }
            else if (transition == FXUIStateTransition.SNAPPED_POSITION_LEFT_CLICK)
            {
               messager.submitMessage(BehaviorUI.API.ActiveEditor, null);
               messager.submitMessage(BehaviorUI.API.SelectedGraphic, null);
            }
         }
      };

      sceneNode.addEventHandler(MouseEvent.MOUSE_CLICKED, this::mouseClicked);
   }

   @Override
   public void handle(long now)
   {
      if (activeEditor.pollActivated())
      {
         if (activeEditor.activationChanged())
         {
            LogTools.debug("SnappedPositionEditor activated");
            sceneNode.addEventHandler(MouseEvent.MOUSE_MOVED, this::mouseMoved);
         }

         mouseMovedMeshIntersection.poll();
         mouseClickedMeshIntersection.poll();

         if (mouseClickedMeshIntersection.hasNext())  // use the clicked position if clicked
         {
            messager.submitMessage(SnappedPositionEditor.API.SelectedPosition, mouseClickedMeshIntersection.read());
         }
         else if (mouseMovedMeshIntersection.hasNext())  // just for selection preview
         {
            messager.submitMessage(SnappedPositionEditor.API.SelectedPosition, mouseMovedMeshIntersection.read());
         }

         if (mouseClickedMeshIntersection.hasNext())
         {
            LogTools.debug("Selected position is validated: {}", mouseClickedMeshIntersection.read());
            activeStateMachine.get().transition(now, FXUIStateTransition.SNAPPED_POSITION_LEFT_CLICK);
         }

         if (mouseRightClicked.poll())
         {
            activeStateMachine.get().transition(now, FXUIStateTransition.SNAPPED_POSITION_RIGHT_CLICK);
         }

      }
      else if (activeEditor.activationChanged())
      {
         LogTools.debug("Snapped position editor deactivated.");
         sceneNode.removeEventHandler(MouseEvent.MOUSE_MOVED, this::mouseMoved);
      }
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
      if (event.isStillSincePress())
      {
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
         else
         {
            if (event.getButton() == MouseButton.PRIMARY)
            {
               PickResult pickResult = event.getPickResult();
               Node intersectedNode = pickResult.getIntersectedNode();

               LogTools.debug("intersectedNode: {}", intersectedNode);

               if (intersectedNode instanceof Sphere)
               {
                  messager.submitMessage(BehaviorUI.API.ActiveStateMachine, positionEditorStateMachine);
                  messager.submitMessage(BehaviorUI.API.SelectedGraphic, intersectedNode);

                  positionEditorStateMachine.start();
               }
            }
         }
      }
   }

   public Point3D calculateMouseIntersection(MouseEvent event)
   {
      PickResult pickResult = event.getPickResult();
      Node intersectedNode = pickResult.getIntersectedNode();

      if (intersectedNode != null && intersectedNode instanceof MeshView)
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
         if (event.getEventType() == MouseEvent.MOUSE_CLICKED)
         {
            LogTools.debug("pick failed. pickResult: {}, intersectedNode: {}", pickResult, intersectedNode);
         }
         return null;
      }
   }

   public static class API
   {
      private static final FXUIMessagerAPIFactory apiFactory = new FXUIMessagerAPIFactory(SnappedPositionEditor.class);

      public static final Topic<Point3D> SelectedPosition = apiFactory.createTopic("SelectedPosition", Point3D.class);

      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
