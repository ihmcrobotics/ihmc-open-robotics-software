package us.ihmc.javaFXToolkit.cameraControllers;

import java.util.function.Predicate;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import javafx.animation.AnimationTimer;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.ReadOnlyDoubleProperty;
import javafx.event.Event;
import javafx.event.EventHandler;
import javafx.geometry.Point3D;
import javafx.scene.Node;
import javafx.scene.PerspectiveCamera;
import javafx.scene.SubScene;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.PickResult;
import javafx.scene.input.ScrollEvent;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Sphere;
import javafx.scene.transform.Affine;
import javafx.scene.transform.Translate;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.robotics.MathTools;

/**
 * This class provides a simple controller for a JavaFX {@link PerspectiveCamera}.
 * The control is achieved via event handling by adding this controller as an {@link EventHandler} to the scene or sub-scene the camera is attached to.
 * <p>
 * Behavior of this camera controller:
 * <li> The camera is always pointing toward a focus point.
 * <li> The focus point can be translated via keyboard bindings, or instantly moved with a mouse shortcut only if {@link #setupRayBasedFocusTranslation(Predicate)} or {@link #enableShiftClickFocusTranslation()} has been called.
 * <li> The camera zoom can be changed vi the mouse wheel.
 * <li> Using the mouse, the camera can be rotated around the focus point.
 * @author Sylvain Bertrand
 */
public class FocusBasedCameraMouseEventHandler implements EventHandler<Event>
{
   private static final double DEFAULT_DISTANCE_FROM_FOCUS_POINT = 10.0;

   private final Sphere focusPointViz;

   private final Translate focusPointTranslation;
   private final Affine cameraOrientation;
   private final Translate offsetFromFocusPoint = new Translate(0.0, 0.0, -DEFAULT_DISTANCE_FROM_FOCUS_POINT);

   private final CameraZoomCalculator zoomCalculator = new CameraZoomCalculator();
   private final CameraRotationCalculator rotationCalculator;
   private final CameraTranslationCalculator translationCalculator;

   private final EventHandler<ScrollEvent> zoomEventHandler = zoomCalculator.createScrollEventHandler();
   private final EventHandler<MouseEvent> rotationEventHandler;
   private final EventHandler<KeyEvent> translationEventHandler;

   public FocusBasedCameraMouseEventHandler(ReadOnlyDoubleProperty sceneWidthProperty, ReadOnlyDoubleProperty sceneHeightProperty, PerspectiveCamera camera,
                                            Vector3d up, Vector3d forward)
   {
      Vector3d left = new Vector3d();
      left.cross(up, forward);
      if (!MathTools.epsilonEquals(left.length(), 1.0, 1.0 - 5))
         throw new RuntimeException("The vectors up and forward must be orthogonal. Received: up = " + up + ", forward = " + forward);

      zoomCalculator.zoomProperty().bindBidirectional(offsetFromFocusPoint.zProperty());
      zoomCalculator.setInvertZoomDirection(true);
      zoomCalculator.setMinZoom(-0.90 * camera.getFarClip());
      zoomCalculator.setMaxZoom(-1.10 * camera.getNearClip());

      rotationCalculator = new CameraRotationCalculator(up, forward);
      rotationCalculator.setFastModifierPredicate(event -> event.isShiftDown());
      cameraOrientation = rotationCalculator.getRotation();
      rotationEventHandler = rotationCalculator.createMouseEventHandler(sceneWidthProperty, sceneHeightProperty);

      translationCalculator = new CameraTranslationCalculator(up);
      translationCalculator.setFastModifierPredicate(event -> event.isShiftDown());
      translationCalculator.setCameraOrientation(cameraOrientation);
      translationCalculator.setZoom(zoomCalculator.zoomProperty());
      focusPointTranslation = translationCalculator.getTranslation();
      translationEventHandler = translationCalculator.createKeyEventHandler();

      changeCameraPosition(-2.0, 0.7, 1.0);

      camera.getTransforms().addAll(focusPointTranslation, cameraOrientation, offsetFromFocusPoint);

      Point3d cameraPosition = new Point3d();
      JavaFXTools.applyTranform(camera.getLocalToSceneTransform(), cameraPosition);

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
            double sphereRadius = 0.0025 * Math.abs(offsetFromFocusPoint.getTz());
            focusPointViz.setRadius(sphereRadius);
         }
      }.start();
   }

   public void changeCameraPosition(double x, double y, double z)
   {
      Point3d desiredCameraPosition = new Point3d(x, y, z);
      Point3d desiredFocusPoint = new Point3d(focusPointTranslation.getX(), focusPointTranslation.getY(), focusPointTranslation.getZ());

      double distanceFromFocusPoint = desiredCameraPosition.distance(desiredFocusPoint);
      offsetFromFocusPoint.setZ(-distanceFromFocusPoint);
      rotationCalculator.setRotationFromCameraAndFocusPositions(desiredCameraPosition, desiredFocusPoint, 0.0);
   }

   @Override
   public void handle(Event event)
   {
      if (event instanceof ScrollEvent)
         zoomEventHandler.handle((ScrollEvent) event);
      if (event instanceof KeyEvent)
         translationEventHandler.handle((KeyEvent) event);

      if (event instanceof MouseEvent)
      {
         if (rayBasedFocusTranslation != null)
            rayBasedFocusTranslation.handle((MouseEvent) event);

         rotationEventHandler.handle((MouseEvent) event);
      }
   }

   private EventHandler<MouseEvent> rayBasedFocusTranslation = null;

   public void enableShiftClickFocusTranslation()
   {
      setupRayBasedFocusTranslation(event -> event.isShiftDown());
   }

   public void setupRayBasedFocusTranslation(Predicate<MouseEvent> condition)
   {
      rayBasedFocusTranslation = new EventHandler<MouseEvent>()
      {
         @Override
         public void handle(MouseEvent event)
         {
            if (event.getButton() != MouseButton.PRIMARY)
               return;

            if (condition.test(event) && event.isStillSincePress() && event.getEventType() == MouseEvent.MOUSE_CLICKED)
            {
               PickResult pickResult = event.getPickResult();
               Node intersectedNode = pickResult.getIntersectedNode();
               if (intersectedNode == null || intersectedNode instanceof SubScene)
                  return;
               Point3D localPoint = pickResult.getIntersectedPoint();
               Point3D scenePoint = intersectedNode.getLocalToSceneTransform().transform(localPoint);
               focusPointTranslation.setX(scenePoint.getX());
               focusPointTranslation.setY(scenePoint.getY());
               focusPointTranslation.setZ(scenePoint.getZ());
            }
         }
      };
   }

   public Sphere getFocusPointViz()
   {
      return focusPointViz;
   }

   public Translate getTranslate()
   {
      return focusPointTranslation;
   }

   //-----------------------------------------------------
   // Zoom properties
   //-----------------------------------------------------

   /** See {@link CameraZoomCalculator#minZoomProperty()}. */
   public final DoubleProperty minZoomProperty()
   {
      return zoomCalculator.minZoomProperty();
   }

   /** See {@link CameraZoomCalculator#getMinZoom()}. */
   public final double getMinZoom()
   {
      return minZoomProperty().get();
   }

   /** See {@link CameraZoomCalculator#setMinZoom(double)}. */
   public final void setMinZoom(final double minZoom)
   {
      minZoomProperty().set(minZoom);
   }

   /** See {@link CameraZoomCalculator#maxZoomProperty()}. */
   public final DoubleProperty maxZoomProperty()
   {
      return zoomCalculator.maxZoomProperty();
   }

   /** See {@link CameraZoomCalculator#getMaxZoom()}. */
   public final double getMaxZoom()
   {
      return maxZoomProperty().get();
   }

   /** See {@link CameraZoomCalculator#setMaxZoom(double)}. */
   public final void setMaxZoom(final double maxZoom)
   {
      maxZoomProperty().set(maxZoom);
   }

   /** See {@link CameraZoomCalculator#zoomSpeedFactorProperty()}. */
   public final DoubleProperty zoomSpeedFactorProperty()
   {
      return zoomCalculator.zoomSpeedFactorProperty();
   }

   /** See {@link CameraZoomCalculator#getZoomSpeedFactor()}. */
   public final double getZoomSpeedFactor()
   {
      return zoomSpeedFactorProperty().get();
   }

   /** See {@link CameraZoomCalculator#setZoomSpeedFactor(double)}. */
   public final void setZoomSpeedFactor(final double zoomSpeedFactor)
   {
      zoomSpeedFactorProperty().set(zoomSpeedFactor);
   }

   /** See {@link CameraZoomCalculator#invertZoomDirectionProperty()}. */
   public final BooleanProperty invertZoomDirectionProperty()
   {
      return zoomCalculator.invertZoomDirectionProperty();
   }

   /** See {@link CameraZoomCalculator#isInvertZoomDirection()}. */
   public final boolean isInvertZoomDirection()
   {
      return invertZoomDirectionProperty().get();
   }

   /** See {@link CameraZoomCalculator#setInvertZoomDirection(boolean)}. */
   public final void setInvertZoomDirection(final boolean invertZoomDirection)
   {
      invertZoomDirectionProperty().set(invertZoomDirection);
   }

   //-----------------------------------------------------
   // Rotation properties
   //-----------------------------------------------------

   /** See {@link CameraRotationCalculator#latitudeProperty()}. */
   public final DoubleProperty latitudeProperty()
   {
      return rotationCalculator.latitudeProperty();
   }

   /** See {@link CameraRotationCalculator#getLatitude()}. */
   public final double getLatitude()
   {
      return latitudeProperty().get();
   }

   /** See {@link CameraRotationCalculator#setLatitude(double)}. */
   public final void setLatitude(final double latitude)
   {
      latitudeProperty().set(latitude);
   }

   /** See {@link CameraRotationCalculator#longitudeProperty()}. */
   public final DoubleProperty longitudeProperty()
   {
      return rotationCalculator.longitudeProperty();
   }

   /** See {@link CameraRotationCalculator#getLongitude()}. */
   public final double getLongitude()
   {
      return longitudeProperty().get();
   }

   /** See {@link CameraRotationCalculator#setLongitude(double)}. */
   public final void setLongitude(final double longitude)
   {
      longitudeProperty().set(longitude);
   }

   /** See {@link CameraRotationCalculator#rollProperty()}. */
   public final DoubleProperty rollProperty()
   {
      return rotationCalculator.rollProperty();
   }

   /** See {@link CameraRotationCalculator#getRoll()}. */
   public final double getRoll()
   {
      return rollProperty().get();
   }

   /** See {@link CameraRotationCalculator#setRoll(double)}. */
   public final void setRoll(final double roll)
   {
      rollProperty().set(roll);
   }

   /** See {@link CameraRotationCalculator#keepRotationLeveledProperty()}. */
   public final BooleanProperty keepRotationLeveledProperty()
   {
      return rotationCalculator.keepRotationLeveledProperty();
   }

   /** See {@link CameraRotationCalculator#isKeepRotationLeveled()}. */
   public final boolean isKeepRotationLeveled()
   {
      return keepRotationLeveledProperty().get();
   }

   /** See {@link CameraRotationCalculator#setKeepRotationLeveled(boolean)}. */
   public final void setKeepRotationLeveled(final boolean keepRotationLeveled)
   {
      keepRotationLeveledProperty().set(keepRotationLeveled);
   }

   /** See {@link CameraRotationCalculator#fastModifierPredicateProperty()}. */
   public final ObjectProperty<Predicate<MouseEvent>> rotationFastModifierPredicateProperty()
   {
      return rotationCalculator.fastModifierPredicateProperty();
   }

   /** See {@link CameraRotationCalculator#getFastModifierPredicate()}. */
   public final Predicate<MouseEvent> getRotationFastModifierPredicate()
   {
      return rotationFastModifierPredicateProperty().get();
   }

   /** See {@link CameraRotationCalculator#setFastModifierPredicate(Predicate)}. */
   public final void setRotationFastModifierPredicate(final Predicate<MouseEvent> fastModifierPredicate)
   {
      rotationFastModifierPredicateProperty().set(fastModifierPredicate);
   }

   /** See {@link CameraRotationCalculator#slowModifierProperty()}. */
   public final DoubleProperty rotationSlowModifierProperty()
   {
      return rotationCalculator.slowModifierProperty();
   }

   /** See {@link CameraRotationCalculator#getSlowModifier()}. */
   public final double getRotationSlowModifier()
   {
      return rotationSlowModifierProperty().get();
   }

   /** See {@link CameraRotationCalculator#setSlowModifier(double)}. */
   public final void setRotationSlowModifier(final double slowModifier)
   {
      rotationSlowModifierProperty().set(slowModifier);
   }

   /** See {@link CameraRotationCalculator#fastModifierProperty()}. */
   public final DoubleProperty rotationFastModifierProperty()
   {
      return rotationCalculator.fastModifierProperty();
   }

   /** See {@link CameraRotationCalculator#getFastModifier()}. */
   public final double getRotationFastModifier()
   {
      return rotationFastModifierProperty().get();
   }

   /** See {@link CameraRotationCalculator#setFastModifier(double)}. */
   public final void setRotationFastModifier(final double fastModifier)
   {
      rotationFastModifierProperty().set(fastModifier);
   }

   /** See {@link CameraRotationCalculator#rollModifierProperty()}. */
   public final DoubleProperty rollModifierProperty()
   {
      return rotationCalculator.rollModifierProperty();
   }

   /** See {@link CameraRotationCalculator#getRollModifier()}. */
   public final double getRollModifier()
   {
      return rollModifierProperty().get();
   }

   /** See {@link CameraRotationCalculator#setRollModifier(double)}. */
   public final void setRollModifier(final double rollModifier)
   {
      rollModifierProperty().set(rollModifier);
   }

   /** See {@link CameraRotationCalculator#rotationMouseButtonProperty()}. */
   public final ObjectProperty<MouseButton> rotationMouseButtonProperty()
   {
      return rotationCalculator.rotationMouseButtonProperty();
   }

   /** See {@link CameraRotationCalculator#getRotationMouseButton()}. */
   public final MouseButton getRotationMouseButton()
   {
      return rotationMouseButtonProperty().get();
   }

   /** See {@link CameraRotationCalculator#setRotationMouseButton(MouseButton)}. */
   public final void setRotationMouseButton(final MouseButton rotationMouseButton)
   {
      rotationMouseButtonProperty().set(rotationMouseButton);
   }

   /** See {@link CameraRotationCalculator#restrictLatitudeProperty()}. */
   public final BooleanProperty restrictLatitudeProperty()
   {
      return rotationCalculator.restrictLatitudeProperty();
   }

   /** See {@link CameraRotationCalculator#isRestrictLatitude()}. */
   public final boolean isRestrictLatitude()
   {
      return restrictLatitudeProperty().get();
   }

   /** See {@link CameraRotationCalculator#setRestrictLatitude(boolean)}. */
   public final void setRestrictLatitude(final boolean restrictLatitude)
   {
      restrictLatitudeProperty().set(restrictLatitude);
   }

   /** See {@link CameraRotationCalculator#minLatitudeProperty()}. */
   public final DoubleProperty minLatitudeProperty()
   {
      return rotationCalculator.minLatitudeProperty();
   }

   /** See {@link CameraRotationCalculator#getMinLatitude()}. */
   public final double getMinLatitude()
   {
      return minLatitudeProperty().get();
   }

   /** See {@link CameraRotationCalculator#setMinLatitude(double)}. */
   public final void setMinLatitude(final double minLatitude)
   {
      minLatitudeProperty().set(minLatitude);
   }

   /** See {@link CameraRotationCalculator#maxLatitudeProperty()}. */
   public final DoubleProperty maxLatitudeProperty()
   {
      return rotationCalculator.maxLatitudeProperty();
   }

   /** See {@link CameraRotationCalculator#getMaxLatitude()}. */
   public final double getMaxLatitude()
   {
      return maxLatitudeProperty().get();
   }

   /** See {@link CameraRotationCalculator#setMaxLatitude(double)}. */
   public final void setMaxLatitude(final double maxLatitude)
   {
      maxLatitudeProperty().set(maxLatitude);
   }

   //-----------------------------------------------------
   // Translation properties
   //-----------------------------------------------------

   /** See {@link CameraTranslationCalculator#maxLatitudeProperty()}. */
   public final BooleanProperty keepTranslationLeveledProperty()
   {
      return translationCalculator.keepTranslationLeveledProperty();
   }

   /** See {@link CameraTranslationCalculator#isKeepTranslationLeveled()}. */
   public final boolean isKeepTranslationLeveled()
   {
      return keepTranslationLeveledProperty().get();
   }

   /** See {@link CameraTranslationCalculator#setKeepTranslationLeveled(boolean)}. */
   public final void setKeepTranslationLeveled(final boolean keepTranslationLeveled)
   {
      keepTranslationLeveledProperty().set(keepTranslationLeveled);
   }

   /** See {@link CameraTranslationCalculator#fastModifierPredicateProperty()}. */
   public final ObjectProperty<Predicate<KeyEvent>> translationFastModifierPredicateProperty()
   {
      return translationCalculator.fastModifierPredicateProperty();
   }

   /** See {@link CameraTranslationCalculator#getFastModifierPredicate()}. */
   public final Predicate<KeyEvent> getTranslationFastModifierPredicate()
   {
      return translationFastModifierPredicateProperty().get();
   }

   /** See {@link CameraTranslationCalculator#setFastModifierPredicate(Predicate)}. */
   public final void setTranslationFastModifierPredicate(final Predicate<KeyEvent> fastModifierPredicate)
   {
      translationFastModifierPredicateProperty().set(fastModifierPredicate);
   }

   /** See {@link CameraTranslationCalculator#slowModifierProperty()}. */
   public final DoubleProperty translationSlowModifierProperty()
   {
      return translationCalculator.slowModifierProperty();
   }

   /** See {@link CameraTranslationCalculator#getSlowModifier}. */
   public final double getTranslationSlowModifier()
   {
      return translationSlowModifierProperty().get();
   }

   /** See {@link CameraTranslationCalculator#setSlowModifier(double)}. */
   public final void setTranslationSlowModifier(final double slowModifier)
   {
      translationSlowModifierProperty().set(slowModifier);
   }

   /** See {@link CameraTranslationCalculator#fastModifierProperty()}. */
   public final DoubleProperty translationFastModifierProperty()
   {
      return translationCalculator.fastModifierProperty();
   }

   /** See {@link CameraTranslationCalculator#getFastModifier()}. */
   public final double getTranslationFastModifier()
   {
      return translationFastModifierProperty().get();
   }

   /** See {@link CameraTranslationCalculator#setFastModifier(double)}. */
   public final void setTranslationFastModifier(final double fastModifier)
   {
      translationFastModifierProperty().set(fastModifier);
   }

   /** See {@link CameraTranslationCalculator#minTranslationOffsetProperty()}. */
   public final DoubleProperty minTranslationOffsetProperty()
   {
      return translationCalculator.minTranslationOffsetProperty();
   }

   /** See {@link CameraTranslationCalculator#getMinTranslationOffset()}. */
   public final double getMinTranslationOffset()
   {
      return minTranslationOffsetProperty().get();
   }

   /** See {@link CameraTranslationCalculator#setMinTranslationOffset(double)}. */
   public final void setMinTranslationOffset(final double minTranslationOffset)
   {
      minTranslationOffsetProperty().set(minTranslationOffset);
   }

   /** See {@link CameraTranslationCalculator#zoomToTranslationPowProperty()}. */
   public final DoubleProperty zoomToTranslationPowProperty()
   {
      return translationCalculator.zoomToTranslationPowProperty();
   }

   /** See {@link CameraTranslationCalculator#getZoomToTranslationPow()}. */
   public final double getZoomToTranslationPow()
   {
      return zoomToTranslationPowProperty().get();
   }

   /** See {@link CameraTranslationCalculator#setZoomToTranslationPow(double)}. */
   public final void setZoomToTranslationPow(final double zoomToTranslationPow)
   {
      zoomToTranslationPowProperty().set(zoomToTranslationPow);
   }

   /** See {@link CameraTranslationCalculator#forwardKeyProperty()}. */
   public final ObjectProperty<KeyCode> forwardKeyProperty()
   {
      return translationCalculator.forwardKeyProperty();
   }

   /** See {@link CameraTranslationCalculator#getForwardKey()}. */
   public final KeyCode getForwardKey()
   {
      return forwardKeyProperty().get();
   }

   /** See {@link CameraTranslationCalculator#setForwardKey(KeyCode)}. */
   public final void setForwardKey(final KeyCode forwardKey)
   {
      forwardKeyProperty().set(forwardKey);
   }

   /** See {@link CameraTranslationCalculator#backwardKeyProperty()}. */
   public final ObjectProperty<KeyCode> backwardKeyProperty()
   {
      return translationCalculator.backwardKeyProperty();
   }

   /** See {@link CameraTranslationCalculator#getBackwardKey()}. */
   public final KeyCode getBackwardKey()
   {
      return backwardKeyProperty().get();
   }

   /** See {@link CameraTranslationCalculator#setBackwardKey(KeyCode)}. */
   public final void setBackwardKey(final KeyCode backwardKey)
   {
      backwardKeyProperty().set(backwardKey);
   }

   /** See {@link CameraTranslationCalculator#leftKeyProperty()}. */
   public final ObjectProperty<KeyCode> leftKeyProperty()
   {
      return translationCalculator.leftKeyProperty();
   }

   /** See {@link CameraTranslationCalculator#getLeftKey()}. */
   public final KeyCode getLeftKey()
   {
      return leftKeyProperty().get();
   }

   /** See {@link CameraTranslationCalculator#setLeftKey(KeyCode)}. */
   public final void setLeftKey(final KeyCode leftKey)
   {
      leftKeyProperty().set(leftKey);
   }

   /** See {@link CameraTranslationCalculator#rightKeyProperty()}. */
   public final ObjectProperty<KeyCode> rightKeyProperty()
   {
      return translationCalculator.rightKeyProperty();
   }

   /** See {@link CameraTranslationCalculator#getRightKey()}. */
   public final KeyCode getRightKey()
   {
      return rightKeyProperty().get();
   }

   /** See {@link CameraTranslationCalculator#setRightKey(KeyCode)}. */
   public final void setRightKey(final KeyCode rightKey)
   {
      rightKeyProperty().set(rightKey);
   }

   /** See {@link CameraTranslationCalculator#upKeyProperty()}. */
   public final ObjectProperty<KeyCode> upKeyProperty()
   {
      return translationCalculator.upKeyProperty();
   }

   /** See {@link CameraTranslationCalculator#getUpKey()}. */
   public final KeyCode getUpKey()
   {
      return upKeyProperty().get();
   }

   /** See {@link CameraTranslationCalculator#setUpKey(KeyCode)}. */
   public final void setUpKey(final KeyCode upKey)
   {
      upKeyProperty().set(upKey);
   }

   /** See {@link CameraTranslationCalculator#downKeyProperty()}. */
   public final ObjectProperty<KeyCode> downKeyProperty()
   {
      return translationCalculator.downKeyProperty();
   }

   /** See {@link CameraTranslationCalculator#getDownKey()}. */
   public final KeyCode getDownKey()
   {
      return downKeyProperty().get();
   }

   /** See {@link CameraTranslationCalculator#setDownKey(KeyCode)}. */
   public final void setDownKey(final KeyCode downKey)
   {
      downKeyProperty().set(downKey);
   }
}
