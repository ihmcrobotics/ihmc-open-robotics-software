package us.ihmc.humanoidBehaviors.ui.editors;

import com.sun.javafx.scene.CameraHelper;
import javafx.event.EventHandler;
import javafx.scene.Camera;
import javafx.scene.Node;
import javafx.scene.SubScene;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidBehaviors.tools.thread.TypedNotification;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.model.FXUIStateTransitionTrigger;
import us.ihmc.humanoidBehaviors.ui.model.interfaces.OrientationEditable;
import us.ihmc.humanoidBehaviors.ui.tools.PrivateAnimationTimer;
import us.ihmc.log.LogTools;

import java.util.function.Consumer;

public class OrientationYawEditor
{
   private final SubScene sceneNode;
   private final Consumer<FXUIStateTransitionTrigger> onExit;

   // these are necessary to keep a consistent reference
   private final EventHandler<MouseEvent> mouseMoved = this::mouseMoved;
   private final EventHandler<MouseEvent> mouseClicked = this::mouseClicked;

   private final TypedNotification<Point3D> mouseMovedOrientation = new TypedNotification<>();
   private final TypedNotification<Point3D> mouseClickedOrientation = new TypedNotification<>();
   private final Notification mouseRightClicked = new Notification();

   private OrientationEditable selectedGraphic;
   private PrivateAnimationTimer orientationAnimationTimer;

   public OrientationYawEditor(SubScene sceneNode, OrientationEditable selectedGraphic, Consumer<FXUIStateTransitionTrigger> onExit)
   {
      this.sceneNode = sceneNode;
      this.selectedGraphic = selectedGraphic;
      this.onExit = onExit;

      BehaviorUI.claimEditing(this);

      sceneNode.addEventHandler(MouseEvent.MOUSE_MOVED, mouseMoved);
      sceneNode.addEventHandler(MouseEvent.MOUSE_CLICKED, mouseClicked);
      this.selectedGraphic.setMouseTransparent(true);

      orientationAnimationTimer = new PrivateAnimationTimer(this::handleOrientation);
      orientationAnimationTimer.start();
   }

   private void handleOrientation(long now)
   {
      mouseMovedOrientation.poll();
      mouseClickedOrientation.poll();

      if (mouseClickedOrientation.hasNext())  // use the clicked position if clicked
      {
         selectedGraphic.setOrientation(mouseClickedOrientation.read());
      }
      else if (mouseMovedOrientation.hasNext())  // just for selection preview
      {
         selectedGraphic.setOrientation(mouseMovedOrientation.read());
      }

      if (mouseClickedOrientation.hasNext())
      {
         LogTools.debug("Selected orientation is validated: {}", mouseClickedOrientation.read());
         deactivate(FXUIStateTransitionTrigger.ORIENTATION_LEFT_CLICK);
      }

      if (mouseRightClicked.poll())
      {
         deactivate(FXUIStateTransitionTrigger.RIGHT_CLICK);
      }
   }

   private void deactivate(FXUIStateTransitionTrigger exitType)
   {
      orientationAnimationTimer.stop();

      LogTools.debug("Orientation editor deactivated.");
      sceneNode.removeEventHandler(MouseEvent.MOUSE_MOVED, mouseMoved);
      sceneNode.removeEventHandler(MouseEvent.MOUSE_CLICKED, mouseClicked);
      selectedGraphic.setMouseTransparent(false);

      BehaviorUI.ACTIVE_EDITOR = null; // do this before exit because it probably switches to new editor

      onExit.accept(exitType);
   }

   private void mouseMoved(MouseEvent event)
   {
      Point3D point3D = intersectRayWithPlane(event);
      mouseMovedOrientation.add(point3D);
   }

   private void mouseClicked(MouseEvent event)
   {
      if (!event.isConsumed() && event.isStillSincePress() && BehaviorUI.ACTIVE_EDITOR == this)
      {
         LogTools.debug("consume mouseClicked");
         event.consume();
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

   private Point3D intersectRayWithPlane(MouseEvent event)
   {
      Line3D line = getPickRay(sceneNode.getCamera(), event);

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
