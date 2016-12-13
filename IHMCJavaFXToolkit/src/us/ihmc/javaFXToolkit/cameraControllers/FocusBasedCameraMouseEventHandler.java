package us.ihmc.javaFXToolkit.cameraControllers;

import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import javafx.animation.AnimationTimer;
import javafx.beans.property.ReadOnlyDoubleProperty;
import javafx.event.Event;
import javafx.event.EventHandler;
import javafx.geometry.Point3D;
import javafx.scene.PerspectiveCamera;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.ScrollEvent;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Sphere;
import javafx.scene.transform.Affine;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Transform;
import javafx.scene.transform.Translate;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.GeometryTools;

public class FocusBasedCameraMouseEventHandler implements EventHandler<Event>
{
   private static final double DEFAULT_DISTANCE_FROM_FOCUS_POINT = 10.0;

   private Point3D focusPoint = new Point3D(0.0, 0.0, 0.0);
   private final Sphere focusPointViz;

   private final Translate focusPointTranslation = new Translate();
   private final Affine cameraOrientation = new Affine();
   private final Translate offsetFromFocusPoint = new Translate(0.0, 0.0, -DEFAULT_DISTANCE_FROM_FOCUS_POINT);

   private double focusPointSlowModifier = 0.005;
   private double focusPointFastModifier = 2.0 * focusPointSlowModifier;

   private final Point2d oldMouseLocation = new Point2d();
   private final Point2d newMouseLocation = new Point2d();
   private final Point2d centerLocation = new Point2d();

   private double rotateSlowModifier = 150.0;
   private double rotateFastModifier = 0.5 * rotateSlowModifier;
   private double rollModifierScaleFactor = 30.0;

   private final PerspectiveCamera camera;
   private final Vector3d up;

   private final ReadOnlyDoubleProperty sceneWidthProperty;
   private final ReadOnlyDoubleProperty sceneHeightProperty;

   private boolean keepCameraLeveled = true;
   private boolean performHorizontalTranslation = true;

   private double minAngleBetweenCameraZAxisAndUp = 0.2;

   public FocusBasedCameraMouseEventHandler(ReadOnlyDoubleProperty sceneWidthProperty, ReadOnlyDoubleProperty sceneHeightProperty, PerspectiveCamera camera,
         Vector3d up)
   {
      this.sceneWidthProperty = sceneWidthProperty;
      this.sceneHeightProperty = sceneHeightProperty;
      this.camera = camera;
      this.up = new Vector3d(up);
      this.up.normalize();
      camera.getTransforms().addAll(focusPointTranslation, cameraOrientation, offsetFromFocusPoint);

      changeCameraPosition(-2.0, 0.7, 1.0);

      focusPointViz = new Sphere(0.01);
      PhongMaterial material = new PhongMaterial();
      material.setDiffuseColor(Color.DARKRED);
      material.setSpecularColor(Color.RED);
      focusPointViz.setMaterial(material);
      focusPointViz.getTransforms().add(focusPointTranslation);

      new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            shiftCameraFocusPoint();
         }
      }.start();
   }

   public void setKeepCameraLeveled(boolean keepCameraLeveled)
   {
      this.keepCameraLeveled = keepCameraLeveled;
   }

   public void changeCameraPosition(double x, double y, double z)
   {
      Point3d desiredCameraPosition = new Point3d(x, y, z);
      Point3d desiredFocusPoint = new Point3d(focusPoint.getX(), focusPoint.getY(), focusPoint.getZ());

      double distanceFromFocusPoint = desiredCameraPosition.distance(desiredFocusPoint);
      offsetFromFocusPoint.setZ(-distanceFromFocusPoint);

      Vector3d zAxis = new Vector3d();
      zAxis.sub(desiredFocusPoint, desiredCameraPosition);
      zAxis.normalize();

      Vector3d xAxis = new Vector3d();

      Vector3d down = new Vector3d(up);
      down.negate();

      xAxis.cross(down, zAxis);
      xAxis.normalize();

      Vector3d yAxis = new Vector3d();
      yAxis.cross(zAxis, xAxis);

      Matrix3d rotation = new Matrix3d();
      rotation.setColumn(0, xAxis);
      rotation.setColumn(1, yAxis);
      rotation.setColumn(2, zAxis);
      JavaFXTools.convertRotationMatrixToAffine(rotation, cameraOrientation);
   }

   @Override
   public void handle(Event event)
   {
      if (event instanceof ScrollEvent)
         handleScrollEvent((ScrollEvent) event);
      if (event instanceof KeyEvent)
         handleKeyEvent((KeyEvent) event);
      if (event instanceof MouseEvent)
         handleMouseEvent((MouseEvent) event);
   }

   private void handleScrollEvent(ScrollEvent event)
   {
      double deltaZoom = event.getDeltaY();

      double newOffset = offsetFromFocusPoint.getTz() + Math.abs(offsetFromFocusPoint.getTz()) / 10.0 * Math.signum(deltaZoom);
      newOffset = MathTools.clipToMinMax(newOffset, -0.90 * camera.getFarClip(), -1.10 * camera.getNearClip());
      offsetFromFocusPoint.setZ(newOffset);
   }

   private void handleMouseEvent(MouseEvent event)
   {
      if (event.getButton() != MouseButton.PRIMARY)
         return;

      if (event.getEventType() == MouseEvent.MOUSE_PRESSED || event.getEventType() == MouseEvent.MOUSE_DRAGGED)
      {
         // Acquire the new mouse coordinates from the recent event
         centerLocation.set(sceneWidthProperty.doubleValue() / 2.0, sceneHeightProperty.doubleValue() / 2.0);
         newMouseLocation.set(event.getSceneX(), event.getSceneY());

         if (event.getEventType() == MouseEvent.MOUSE_DRAGGED)
         {
            // Calculate the rotation change of the camera pitch
            double modifier = event.isShiftDown() ? rotateFastModifier : rotateSlowModifier;

            double angleShift = newMouseLocation.distance(oldMouseLocation) / modifier;
            if (Math.abs(angleShift) < 1.0e-5)
               return;

            Vector2d dragDirection = new Vector2d();
            dragDirection.sub(newMouseLocation, oldMouseLocation);
            double dragMagnitude = dragDirection.length();
            if (dragMagnitude < 1.0e-3)
               return;
            dragDirection.scale(1.0 / dragMagnitude);

            Vector3d deltaAxis = new Vector3d();
            deltaAxis.set(-dragDirection.getY(), dragDirection.getX(), 0.0);

            AxisAngle4d deltaAxisAngle = new AxisAngle4d(deltaAxis, angleShift);
            Affine deltaOrientation = JavaFXTools.createAffineFromAxisAngle(deltaAxisAngle);
            cameraOrientation.append(deltaOrientation);

            if (keepCameraLeveled)
            {
               Vector3d zAxis = new Vector3d(cameraOrientation.getMxz(), cameraOrientation.getMyz(), cameraOrientation.getMzz());
               double angleWithUp = zAxis.angle(up);

               if (angleWithUp > Math.PI - minAngleBetweenCameraZAxisAndUp)
               {
                  double correctionAngle = Math.PI - minAngleBetweenCameraZAxisAndUp - angleWithUp;
                  Vector3d correctionAxis = new Vector3d();
                  correctionAxis.cross(up, zAxis);
                  correctionAxis.normalize();
                  deltaAxisAngle = new AxisAngle4d(correctionAxis, correctionAngle);
                  deltaOrientation = JavaFXTools.createAffineFromAxisAngle(deltaAxisAngle);
                  cameraOrientation.prepend(deltaOrientation);
               }
               else if (angleWithUp < minAngleBetweenCameraZAxisAndUp)
               {
                  double correctionAngle = minAngleBetweenCameraZAxisAndUp - angleWithUp;
                  Vector3d correctionAxis = new Vector3d();
                  correctionAxis.cross(up, zAxis);
                  correctionAxis.normalize();
                  deltaAxisAngle = new AxisAngle4d(correctionAxis, correctionAngle);
                  deltaOrientation = JavaFXTools.createAffineFromAxisAngle(deltaAxisAngle);
                  cameraOrientation.prepend(deltaOrientation);
               }

               zAxis = new Vector3d(cameraOrientation.getMxz(), cameraOrientation.getMyz(), cameraOrientation.getMzz());

               Vector3d down = new Vector3d(up);
               down.negate();
               Vector3d xAxisLeveled = new Vector3d();
               xAxisLeveled.cross(down, zAxis);
               xAxisLeveled.normalize();
               Vector3d yAxisLeveled = new Vector3d();
               yAxisLeveled.cross(zAxis, xAxisLeveled);

               Matrix3d rotation = new Matrix3d();
               rotation.setColumn(0, xAxisLeveled);
               rotation.setColumn(1, yAxisLeveled);
               rotation.setColumn(2, zAxis);
               JavaFXTools.convertRotationMatrixToAffine(rotation, cameraOrientation);
            }
            else
            {
               Vector2d centerToMouseLocation = new Vector2d();
               centerToMouseLocation.sub(newMouseLocation, centerLocation);
               double roll = 0.0;
               roll = GeometryTools.cross(dragDirection, centerToMouseLocation) / modifier / rollModifierScaleFactor;
               Rotate rollRotate = new Rotate(Math.toDegrees(roll), Rotate.Z_AXIS);
               cameraOrientation.append(rollRotate);
            }

         }

         oldMouseLocation.set(newMouseLocation);
      }
   }

   private final AtomicBoolean isShiftDown = new AtomicBoolean(false);
   private final Set<KeyCode> keyBeingPressed = new HashSet<>();

   private void handleKeyEvent(KeyEvent event)
   {
      KeyCode keyCode = event.getCode();
      if (event.getEventType() == KeyEvent.KEY_PRESSED)
         keyBeingPressed.add(keyCode);
      else if (event.getEventType() == KeyEvent.KEY_RELEASED)
         keyBeingPressed.remove(keyCode);

      isShiftDown.set(event.isShiftDown());
   }

   private void shiftCameraFocusPoint()
   {
      double change = focusPointSlowModifier;
      //Add shift modifier to simulate running speed
      if (isShiftDown.get())
         change = focusPointFastModifier;

      change *= Math.pow(Math.abs(offsetFromFocusPoint.getTz()), 1.5);
      change = Math.min(change, 0.1);

      Vector3d focusPointShift = new Vector3d();
      
      for (KeyCode keyDown : keyBeingPressed)
      {
         if (keyDown == KeyCode.W)
            focusPointShift.setZ(focusPointShift.getZ() - change);
         if (keyDown == KeyCode.S)
            focusPointShift.setZ(focusPointShift.getZ() + change);
         
         if (keyDown == KeyCode.D)
            focusPointShift.setX(focusPointShift.getX() - change);
         if (keyDown == KeyCode.A)
            focusPointShift.setX(focusPointShift.getX() + change);
         
         if (keyDown == KeyCode.Q)
            focusPointShift.setY(focusPointShift.getY() + change);
         if (keyDown == KeyCode.Z)
            focusPointShift.setY(focusPointShift.getY() - change);
      }

      if (performHorizontalTranslation)
      {
         Vector3d cameraZAxis = new Vector3d(cameraOrientation.getMxz(), cameraOrientation.getMyz(), cameraOrientation.getMzz());
         Vector3d down = new Vector3d(up);
         down.negate();
         Vector3d xAxisLeveled = new Vector3d();
         xAxisLeveled.cross(down, cameraZAxis);
         xAxisLeveled.normalize();
         Vector3d yAxisLeveled = new Vector3d(down);
         Vector3d zAxisLeveled = new Vector3d();
         zAxisLeveled.cross(xAxisLeveled, yAxisLeveled);

         Matrix3d rotation = new Matrix3d();
         rotation.setColumn(0, xAxisLeveled);
         rotation.setColumn(1, yAxisLeveled);
         rotation.setColumn(2, zAxisLeveled);
         rotation.transform(focusPointShift);
      }
      else
      {
         Transform localToParentTransform = camera.getLocalToParentTransform();
         JavaFXTools.applyTranform(localToParentTransform, focusPointShift);
      }

      JavaFXTools.addEquals(focusPoint, focusPointShift);
      focusPointTranslation.setX(focusPointTranslation.getX() - focusPointShift.getX());
      focusPointTranslation.setY(focusPointTranslation.getY() - focusPointShift.getY());
      focusPointTranslation.setZ(focusPointTranslation.getZ() - focusPointShift.getZ());
   }

   public Sphere getFocusPointViz()
   {
      return focusPointViz;
   }

   public Translate getTranslate()
   {
      return focusPointTranslation;
   }
}
