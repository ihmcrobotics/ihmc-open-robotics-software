package us.ihmc.humanoidBehaviors.ui.editors;

import com.sun.javafx.scene.CameraHelper;
import javafx.event.EventHandler;
import javafx.scene.Camera;
import javafx.scene.SubScene;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.humanoidBehaviors.tools.thread.TypedNotification;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.model.FXUIStateTransitionTrigger;
import us.ihmc.humanoidBehaviors.ui.model.interfaces.PoseEditable;
import us.ihmc.humanoidBehaviors.ui.tools.PrivateAnimationTimer;
import us.ihmc.log.LogTools;

import java.util.function.Consumer;

public class OrientationYawEditor
{
   private final SubScene sceneNode;
   private final PrivateAnimationTimer orientationAnimationTimer;

   // these are necessary to keep a consistent reference
   private final EventHandler<MouseEvent> mouseMoved = this::mouseMoved;
   private final EventHandler<MouseEvent> mouseClicked = this::mouseClicked;

   private final TypedNotification<Orientation3DReadOnly> mouseMovedOrientation = new TypedNotification<>();
   private final TypedNotification<Orientation3DReadOnly> mouseClickedOrientation = new TypedNotification<>();
   private final Notification mouseRightClicked = new Notification();

   private PoseEditable selectedGraphic;
   private Consumer<FXUIStateTransitionTrigger> onExit;

   public OrientationYawEditor(SubScene sceneNode)
   {
      this.sceneNode = sceneNode;

      orientationAnimationTimer = new PrivateAnimationTimer(this::handleOrientation);
   }

   public void edit(PoseEditable selectedGraphic, Consumer<FXUIStateTransitionTrigger> onExit)
   {
      this.selectedGraphic = selectedGraphic;
      this.onExit = onExit;

      BehaviorUI.claimEditing(this);

      sceneNode.addEventHandler(MouseEvent.MOUSE_MOVED, mouseMoved);
      sceneNode.addEventHandler(MouseEvent.MOUSE_CLICKED, mouseClicked);
      this.selectedGraphic.setMouseTransparent(true);

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
      Orientation3DReadOnly rotationVector = intersectRayWithPlane(event);
      mouseMovedOrientation.add(rotationVector);
   }

   private void mouseClicked(MouseEvent event)
   {
      if (!event.isConsumed() && event.isStillSincePress() && BehaviorUI.ACTIVE_EDITOR == this)
      {
         LogTools.debug("consume mouseClicked");
         event.consume();
         if (event.getButton() == MouseButton.PRIMARY)
         {
            Orientation3DReadOnly orientation = intersectRayWithPlane(event);
            mouseClickedOrientation.add(orientation);
         }
         else if (event.getButton() == MouseButton.SECONDARY)  // maybe move this to patrol controller? or implement cancel
         {
            mouseRightClicked.set();
         }
      }
   }

   private Orientation3DReadOnly intersectRayWithPlane(MouseEvent event)
   {
      Line3D line = getPickRay(sceneNode.getCamera(), event);

      Vector3D planeNormal = new Vector3D(0.0, 0.0, 1.0);  // TODO link to planar region normal
      Point3DReadOnly pickPoint = selectedGraphic.getPosition();

      Point3D pickDirection = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(pickPoint, planeNormal, line.getPoint(), line.getDirection());

      Vector3D rotationVector = new Vector3D(pickDirection);
      rotationVector.sub(pickPoint);

      double yaw = Math.atan2(rotationVector.getY(), rotationVector.getX()); // TODO Allow 3D when linking planar region normal

      return new YawPitchRoll(yaw, 0.0, 0.0);
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
