package us.ihmc.humanoidBehaviors.ui.editors;

import com.sun.javafx.scene.CameraHelper;
import javafx.scene.Camera;
import javafx.scene.SubScene;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.model.*;
import us.ihmc.humanoidBehaviors.ui.references.NotificationReference;
import us.ihmc.humanoidBehaviors.ui.references.QueueReference;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;

public class OrientationYawEditor extends FXUIEditor
{
   private final QueueReference<Point3D> mouseMovedOrientation = new QueueReference<>();
   private final QueueReference<Point3D> mouseClickedOrientation = new QueueReference<>();
   private final NotificationReference mouseRightClicked = new NotificationReference();

   private final FXUIStateMachine orientationEditorStateMachine;

   public OrientationYawEditor(Messager messager, SubScene sceneNode)
   {
      super(messager, sceneNode);

      orientationEditorStateMachine = new FXUIStateMachine(messager, FXUIState.ORIENTATION_EDITOR, FXUIStateTransition.POSITION_LEFT_CLICK)
      {
         @Override
         protected void handleTransition(FXUIStateTransition transition)
         {
            if (transition.isStart())
            {
               messager.submitMessage(BehaviorUI.API.ActiveEditor, BehaviorUI.ORIENTATION_EDITOR);
            }
            else if (transition == FXUIStateTransition.ORIENTATION_LEFT_CLICK)
            {
               messager.submitMessage(BehaviorUI.API.ActiveEditor, null);
               messager.submitMessage(BehaviorUI.API.SelectedGraphic, null);
            }
         }
      };
   }

   public void activate()
   {
      messager.submitMessage(BehaviorUI.API.ActiveStateMachine, orientationEditorStateMachine);
      orientationEditorStateMachine.start();
   }

   @Override
   public void handle(long now)
   {
      if (activeEditor.pollActivated())
      {
         if (activeEditor.activationChanged())
         {
            LogTools.debug("Orientation editor activated");
            subScene.addEventHandler(MouseEvent.MOUSE_MOVED, mouseMoved);
            subScene.addEventHandler(MouseEvent.MOUSE_CLICKED, mouseClicked);
         }

         mouseMovedOrientation.poll();
         mouseClickedOrientation.poll();

         if (mouseClickedOrientation.hasNext())  // use the clicked position if clicked
         {
            messager.submitMessage(OrientationYawEditor.API.SelectedOrientation, mouseClickedOrientation.read());
         }
         else if (mouseMovedOrientation.hasNext())  // just for selection preview
         {
            messager.submitMessage(OrientationYawEditor.API.SelectedOrientation, mouseMovedOrientation.read());
         }

         if (mouseClickedOrientation.hasNext())
         {
            LogTools.debug("Selected orientation is validated: {}", mouseClickedOrientation.read());
            activeStateMachine.get().transition(now, FXUIStateTransition.ORIENTATION_LEFT_CLICK);
         }

         if (mouseRightClicked.poll())
         {
            activeStateMachine.get().transition(now, FXUIStateTransition.RIGHT_CLICK);
         }
      }
      else if (activeEditor.activationChanged())
      {
         LogTools.debug("Orientation editor deactivated.");
         subScene.removeEventHandler(MouseEvent.MOUSE_MOVED, mouseMoved);
         subScene.removeEventHandler(MouseEvent.MOUSE_CLICKED, mouseClicked);
      }
   }

   @Override
   protected void mouseMoved(MouseEvent event)
   {
      Point3D point3D = intersectRayWithPlane(event);
      mouseMovedOrientation.add(point3D);
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
               Point3D point3D = intersectRayWithPlane(event);
               mouseClickedOrientation.add(point3D);
            }
            else if (event.getButton() == MouseButton.SECONDARY)  // maybe move this to patrol controller? or implement cancel
            {
               mouseRightClicked.set();
            }
         }
      }
   }

   private Point3D intersectRayWithPlane(MouseEvent event)
   {
      Line3D line = getPickRay(subScene.getCamera(), event);

      Vector3D planeNormal = new Vector3D(0.0, 0.0, 1.0);  // TODO link to planar region normal
      Point3D pointOnPlane = new Point3D();    // using 0.0 as ground for now

      return EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(pointOnPlane, planeNormal, line.getPoint(), line.getDirection());
   }

   private Line3D getPickRay(Camera camera, MouseEvent event)
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

   public static class API
   {
      private static final FXUIMessagerAPIFactory apiFactory = new FXUIMessagerAPIFactory(OrientationYawEditor.class);

      public static final Topic<Point3D> SelectedOrientation = apiFactory.createTopic("SelectedOrientation", Point3D.class);

      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
