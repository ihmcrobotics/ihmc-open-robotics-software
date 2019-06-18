package us.ihmc.humanoidBehaviors.ui.editors;

import com.sun.javafx.scene.CameraHelper;
import javafx.scene.Camera;
import javafx.scene.SubScene;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.model.FXUIEditor;
import us.ihmc.humanoidBehaviors.ui.model.FXUIStateMachine;
import us.ihmc.humanoidBehaviors.ui.model.FXUIStateTransitionTrigger;
import us.ihmc.humanoidBehaviors.ui.model.interfaces.OrientationEditable;
import us.ihmc.humanoidBehaviors.ui.references.OverTypedReference;
import us.ihmc.humanoidBehaviors.ui.references.TypedNotification;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;

public class OrientationYawEditor extends FXUIEditor
{
   private final TypedNotification<Point3D> mouseMovedOrientation = new TypedNotification<>();
   private final TypedNotification<Point3D> mouseClickedOrientation = new TypedNotification<>();
   private final Notification mouseRightClicked = new Notification();

   private final FXUIStateMachine orientationEditorStateMachine;
   private final OverTypedReference<OrientationEditable> selectedGraphicReference;

   public OrientationYawEditor(Messager messager, SubScene sceneNode)
   {
      super(messager, sceneNode);

      orientationEditorStateMachine = new FXUIStateMachine(messager, FXUIStateTransitionTrigger.POSITION_LEFT_CLICK, trigger ->
      {
         messager.submitMessage(BehaviorUI.API.ActiveEditor, BehaviorUI.ORIENTATION_EDITOR);
      });
      orientationEditorStateMachine.mapTransition(FXUIStateTransitionTrigger.ORIENTATION_LEFT_CLICK, trigger ->
      {
         messager.submitMessage(BehaviorUI.API.ActiveEditor, null);
         messager.submitMessage(BehaviorUI.API.SelectedGraphic, null);
      });

      selectedGraphicReference = new OverTypedReference<>(messager.createInput(BehaviorUI.API.SelectedGraphic));
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
            selectedGraphicReference.get().setMouseTransparent(true);
         }

         mouseMovedOrientation.poll();
         mouseClickedOrientation.poll();

         if (mouseClickedOrientation.hasNext())  // use the clicked position if clicked
         {
            selectedGraphicReference.get().setOrientation(mouseClickedOrientation.read());
         }
         else if (mouseMovedOrientation.hasNext())  // just for selection preview
         {
            selectedGraphicReference.get().setOrientation(mouseMovedOrientation.read());
         }

         if (mouseClickedOrientation.hasNext())
         {
            LogTools.debug("Selected orientation is validated: {}", mouseClickedOrientation.read());
            deactivate();
            activeStateMachine.get().transition(FXUIStateTransitionTrigger.ORIENTATION_LEFT_CLICK);
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
      LogTools.debug("Orientation editor deactivated.");
      subScene.removeEventHandler(MouseEvent.MOUSE_MOVED, mouseMoved);
      subScene.removeEventHandler(MouseEvent.MOUSE_CLICKED, mouseClicked);
      selectedGraphicReference.get().setMouseTransparent(false);
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
}
