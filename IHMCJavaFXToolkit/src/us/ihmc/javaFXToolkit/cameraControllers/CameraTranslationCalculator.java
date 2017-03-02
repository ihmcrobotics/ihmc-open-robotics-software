package us.ihmc.javaFXToolkit.cameraControllers;

import java.util.function.Predicate;

import javafx.animation.AnimationTimer;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.ReadOnlyDoubleProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.event.EventHandler;
import javafx.scene.PerspectiveCamera;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.transform.Transform;
import javafx.scene.transform.Translate;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.javaFXToolkit.JavaFXTools;

/**
 * This class provides the tools necessary to build a simple controller for computing the translation of a JavaFX {@link PerspectiveCamera}.
 * This class is ready to be used with an {@link EventHandler} via {@link #createKeyEventHandler()}.
 * The output of this calculator is the {@link #translation} property which can be bound to an external property or used directly to apply a transformation to the camera.
 * This transformation is not implemented here to provide increased flexibility.
 * The translation computed by this calculator is expressed in world frame, i.e. before applying a rotation to the camera.
 * @author Sylvain Bertrand
 */
public class CameraTranslationCalculator
{
   /**
    * The current translation of the camera.
    * This is the output of this calculator which can be bound to an external property or used directly to apply a transformation to the camera.
    */
   private final Translate translation = new Translate();
   /** Current orientation of the camera necessary when translating the camera in its local frame. */
   private final ObjectProperty<Transform> cameraOrientation = new SimpleObjectProperty<>(this, "cameraOrientation", null);
   /** Current zoom of the camera. It is used to vary the translation speed when using the keyboard. As the zoom increases, the translation speed increases too. */
   private ReadOnlyDoubleProperty currentZoom = null;

   /** When set to true, the translations forward/backward and left/right will be performed on a horizontal plane, i.e. perpendicular the given up axis. */
   private final BooleanProperty keepTranslationLeveled = new SimpleBooleanProperty(this, "keepTranslationLeveled", true);
   /** Condition to trigger the use of the fast modifier to make the camera translate faster when using the keyboard. */
   private final ObjectProperty<Predicate<KeyEvent>> fastModifierPredicate = new SimpleObjectProperty<>(this, "fastModifierPredicate", null);
   /** Slow camera translation modifier when using the keyboard. */
   private final DoubleProperty slowModifier = new SimpleDoubleProperty(this, "slowModifier", 0.005);
   /** Fast camera translation modifier when using the keyboard. It is triggered when the condition held in {@link #fastModifierPredicate} is fulfilled. */
   private final DoubleProperty fastModifier = new SimpleDoubleProperty(this, "fastModifier", 0.010);
   /** Minimum value of a translation offset when using the keyboard. */
   private final DoubleProperty minTranslationOffset = new SimpleDoubleProperty(this, "minTranslationOffset", 0.1);
   /** The zoom-to-translation pow is used to define the relation between the current zoom value and the translation speed of the camera. */
   private final DoubleProperty zoomToTranslationPow = new SimpleDoubleProperty(this, "zoomToTranslationPow", 1.5);

   /** Key binding for moving the camera forward. Its default value is: {@code KeyCode.W}. */
   private final ObjectProperty<KeyCode> forwardKey = new SimpleObjectProperty<>(this, "forwardKey", KeyCode.W);
   /** Key binding for moving the camera backward. Its default value is: {@code KeyCode.S}. */
   private final ObjectProperty<KeyCode> backwardKey = new SimpleObjectProperty<>(this, "backwardKey", KeyCode.S);

   /** Key binding for moving the camera to the left. Its default value is: {@code KeyCode.A}. */
   private final ObjectProperty<KeyCode> leftKey = new SimpleObjectProperty<>(this, "leftKey", KeyCode.A);
   /** Key binding for moving the camera to the right. Its default value is: {@code KeyCode.D}. */
   private final ObjectProperty<KeyCode> rightKey = new SimpleObjectProperty<>(this, "rightKey", KeyCode.D);

   /** Key binding for moving the camera upward. Its default value is: {@code KeyCode.Q}. */
   private final ObjectProperty<KeyCode> upKey = new SimpleObjectProperty<>(this, "upKey", KeyCode.Q);
   /** Key binding for moving the camera downward. Its default value is: {@code KeyCode.Z}. */
   private final ObjectProperty<KeyCode> downKey = new SimpleObjectProperty<>(this, "downKey", KeyCode.Z);

   private final Vector3D down = new Vector3D();

   /**
    * Creates a calculator for the camera translation.
    * @param up indicates which way is up.
    */
   public CameraTranslationCalculator(Vector3D up)
   {
      down.setAndNegate(up);
   }

   /**
    * Creates an {@link EventHandler} to translate the camera using keyboard bindings.
    * @return an {@link EventHandler} to translate the camera with the keyboard.
    */
   public EventHandler<KeyEvent> createKeyEventHandler()
   {
      final Vector3D activeTranslationOffset = new Vector3D();

      AnimationTimer translateAnimation = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            updateObserverTranslation(activeTranslationOffset);
         }
      };
      translateAnimation.start();

      EventHandler<KeyEvent> keyEventHandler = new EventHandler<KeyEvent>()
      {
         @Override
         public void handle(KeyEvent event)
         {
            double modifier;
            if (fastModifierPredicate.get() == null || !fastModifierPredicate.get().test(event))
               modifier = slowModifier.get();
            else
               modifier = fastModifier.get();

            if (currentZoom != null)
            {
               modifier *= Math.pow(Math.abs(currentZoom.get()), zoomToTranslationPow.get());
               modifier = Math.min(modifier, minTranslationOffset.get());
            }

            KeyCode keyDown = event.getCode();
            boolean isKeyReleased = event.getEventType() == KeyEvent.KEY_RELEASED;

            if (keyDown == forwardKey.get())
               activeTranslationOffset.setZ(isKeyReleased ? 0.0 : modifier);
            if (keyDown == backwardKey.get())
               activeTranslationOffset.setZ(isKeyReleased ? 0.0 : -modifier);

            if (keyDown == rightKey.get())
               activeTranslationOffset.setX(isKeyReleased ? 0.0 : modifier);
            if (keyDown == leftKey.get())
               activeTranslationOffset.setX(isKeyReleased ? 0.0 : -modifier);

            if (keyDown == downKey.get())
               activeTranslationOffset.setY(isKeyReleased ? 0.0 : modifier);
            if (keyDown == upKey.get())
               activeTranslationOffset.setY(isKeyReleased ? 0.0 : -modifier);
         }
      };

      return keyEventHandler;
   }

   /**
    * Update the camera translation after applying a translation offset in the camera local frame.
    * @param translationOffset the translation offset in local frame to apply. Not modified.
    */
   public void updateObserverTranslation(Vector3D translationOffset)
   {
      updateObserverTranslation(translationOffset.getX(), translationOffset.getY(), translationOffset.getZ());
   }

   /**
    * Update the camera translation after applying a translation offset in the camera local frame.
    * @param dx the forward/backward translation offset in the camera local frame.
    * @param dy the left/right translation offset in the camera local frame.
    * @param dz the up/down translation offset in the camera local frame.
    */
   public void updateObserverTranslation(double dx, double dy, double dz)
   {
      if (cameraOrientation.get() == null)
      {
         updateWorldTranslation(dx, dy, dz);
         return;
      }

      Vector3D shift = new Vector3D(dx, dy, dz);

      if (keepTranslationLeveled.get())
      {
         double mxz = cameraOrientation.get().getMxz();
         double myz = cameraOrientation.get().getMyz();
         double mzz = cameraOrientation.get().getMzz();
         Vector3D cameraZAxis = new Vector3D(mxz, myz, mzz);
         Vector3D xAxisLeveled = new Vector3D();
         xAxisLeveled.cross(down, cameraZAxis);
         xAxisLeveled.normalize();
         Vector3D yAxisLeveled = new Vector3D(down);
         Vector3D zAxisLeveled = new Vector3D();
         zAxisLeveled.cross(xAxisLeveled, yAxisLeveled);

         RotationMatrix rotation = new RotationMatrix();
         rotation.setColumns(xAxisLeveled, yAxisLeveled, zAxisLeveled);
         rotation.transform(shift);
      }
      else
      {
         JavaFXTools.applyTranform(cameraOrientation.get(), shift);
      }

      JavaFXTools.addEquals(translation, shift);
   }

   /**
    * Update the camera translation after applying a translation offset in the world frame.
    * @param dx the translation offset along the world x-axis.
    * @param dy the translation offset along the world y-axis.
    * @param dz the translation offset along the world z-axis.
    */
   public void updateWorldTranslation(double dx, double dy, double dz)
   {
      Vector3D shift = new Vector3D(dx, dy, dz);
      JavaFXTools.addEquals(translation, shift);
   }

   /**
    * Sets the reference to the current camera orientation to enable translation in the camera frame, i.e. first person.
    * @param cameraOrientation the reference to the current camera orientation. Not modified.
    */
   public void setCameraOrientation(Transform cameraOrientation)
   {
      this.cameraOrientation.set(cameraOrientation);
   }

   /**
    * Sets the reference to the current camera zoom to enable translation speed varying on the zoom value.
    * @param zoom
    */
   public void setZoom(ReadOnlyDoubleProperty zoom)
   {
      currentZoom = zoom;
   }

   /**
    * Get the reference to the translation of the camera.
    * This is the output of this calculator which can be bound to an external property or used directly to apply a transformation to the camera.
    * @return the camera's translation.
    */
   public Translate getTranslation()
   {
      return translation;
   }

   public final BooleanProperty keepTranslationLeveledProperty()
   {
      return keepTranslationLeveled;
   }

   public final boolean isKeepTranslationLeveled()
   {
      return keepTranslationLeveledProperty().get();
   }

   public final void setKeepTranslationLeveled(final boolean keepTranslationLeveled)
   {
      keepTranslationLeveledProperty().set(keepTranslationLeveled);
   }

   public final ObjectProperty<Predicate<KeyEvent>> fastModifierPredicateProperty()
   {
      return fastModifierPredicate;
   }

   public final Predicate<KeyEvent> getFastModifierPredicate()
   {
      return fastModifierPredicateProperty().get();
   }

   public final void setFastModifierPredicate(final Predicate<KeyEvent> fastModifierPredicate)
   {
      fastModifierPredicateProperty().set(fastModifierPredicate);
   }

   public final DoubleProperty slowModifierProperty()
   {
      return slowModifier;
   }

   public final double getSlowModifier()
   {
      return slowModifierProperty().get();
   }

   public final void setSlowModifier(final double slowModifier)
   {
      slowModifierProperty().set(slowModifier);
   }

   public final DoubleProperty fastModifierProperty()
   {
      return fastModifier;
   }

   public final double getFastModifier()
   {
      return fastModifierProperty().get();
   }

   public final void setFastModifier(final double fastModifier)
   {
      fastModifierProperty().set(fastModifier);
   }

   public final DoubleProperty minTranslationOffsetProperty()
   {
      return minTranslationOffset;
   }

   public final double getMinTranslationOffset()
   {
      return minTranslationOffsetProperty().get();
   }

   public final void setMinTranslationOffset(final double minTranslationOffset)
   {
      minTranslationOffsetProperty().set(minTranslationOffset);
   }

   public final DoubleProperty zoomToTranslationPowProperty()
   {
      return zoomToTranslationPow;
   }

   public final double getZoomToTranslationPow()
   {
      return zoomToTranslationPowProperty().get();
   }

   public final void setZoomToTranslationPow(final double zoomToTranslationPow)
   {
      zoomToTranslationPowProperty().set(zoomToTranslationPow);
   }

   public final ObjectProperty<KeyCode> forwardKeyProperty()
   {
      return forwardKey;
   }

   public final KeyCode getForwardKey()
   {
      return forwardKeyProperty().get();
   }

   public final void setForwardKey(final KeyCode forwardKey)
   {
      forwardKeyProperty().set(forwardKey);
   }

   public final ObjectProperty<KeyCode> backwardKeyProperty()
   {
      return backwardKey;
   }

   public final KeyCode getBackwardKey()
   {
      return backwardKeyProperty().get();
   }

   public final void setBackwardKey(final KeyCode backwardKey)
   {
      backwardKeyProperty().set(backwardKey);
   }

   public final ObjectProperty<KeyCode> leftKeyProperty()
   {
      return leftKey;
   }

   public final KeyCode getLeftKey()
   {
      return leftKeyProperty().get();
   }

   public final void setLeftKey(final KeyCode leftKey)
   {
      leftKeyProperty().set(leftKey);
   }

   public final ObjectProperty<KeyCode> rightKeyProperty()
   {
      return rightKey;
   }

   public final KeyCode getRightKey()
   {
      return rightKeyProperty().get();
   }

   public final void setRightKey(final KeyCode rightKey)
   {
      rightKeyProperty().set(rightKey);
   }

   public final ObjectProperty<KeyCode> upKeyProperty()
   {
      return upKey;
   }

   public final KeyCode getUpKey()
   {
      return upKeyProperty().get();
   }

   public final void setUpKey(final KeyCode upKey)
   {
      upKeyProperty().set(upKey);
   }

   public final ObjectProperty<KeyCode> downKeyProperty()
   {
      return downKey;
   }

   public final KeyCode getDownKey()
   {
      return downKeyProperty().get();
   }

   public final void setDownKey(final KeyCode downKey)
   {
      downKeyProperty().set(downKey);
   }
}
