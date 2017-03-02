package us.ihmc.javaFXToolkit.cameraControllers;

import java.util.function.Predicate;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.ReadOnlyDoubleProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.event.EventHandler;
import javafx.scene.PerspectiveCamera;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.transform.Affine;
import javafx.scene.transform.Rotate;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.AngleTools;

/**
 * This class provides the tools necessary to build a simple controller for computing the orientation of a JavaFX {@link PerspectiveCamera}.
 * This class is ready to be used with an {@link EventHandler} via {@link #createMouseEventHandler(ReadOnlyDoubleProperty, ReadOnlyDoubleProperty)}.
 * The output of this calculator is the {@link #rotation} property which can be bound to an external property or used directly to apply a transformation to the camera.
 * This transformation is not implemented here to provide increased flexibility.
 * <p>
 * Note on the approach use to compute the rotation.
 * The camera is assumed to be navigating on a virtual sphere and directed to its center.
 * Its orientation is computed from its geographic coordinates (latitude, longitude).
 * As for the geographic coordinates:
 * <li> the longitude is in the range [-180 degrees; +180 degrees],
 * <li> the latitude is in the range [-90 degrees; +90 degrees],
 * <li> a latitude of +90 degrees corresponds to the camera located at the north pole looking straight down, 
 * <li> a latitude of -90 degrees corresponds to the camera located at the south pole looking straight up.
 * @author Sylvain Bertrand
 */
public class CameraRotationCalculator
{
   /**
    * The current orientation of the camera.
    * This is the output of this calculator which can be bound to an external property or used directly to apply a transformation to the camera.
    */
   private final Affine rotation = new Affine();
   /**
    * Rotation offset computed such that when the {@link #latitude}, {@link #longitude}, and {@link #roll} are all zero, the camera is orientated as follows:
    * <li> the viewing direction (e.g. z-axis) of the camera is aligned with the given forward axis.
    * <li> the vertical direction of the screen (e.g. y-axis) is colinear with the given up axis.
    */
   private final Affine offset = new Affine();
   /**
    * Latitude of the camera on the virtual sphere:
    * <li> the latitude is in the range [-90 degrees; +90 degrees],
    * <li> a latitude of +90 degrees corresponds to the camera located at the north pole looking straight down, 
    * <li> a latitude of -90 degrees corresponds to the camera located at the south pole looking straight up. 
    */
   private final DoubleProperty latitude = new SimpleDoubleProperty(this, "latitude", 0.0);
   /**
    * Longitude of the camera on the virtual sphere:
    * <li> the longitude is in the range [-180 degrees; +180 degrees],
    * <li> at a longitude of zero degree, the camera is directed torward the forward axis.
    */
   private final DoubleProperty longitude = new SimpleDoubleProperty(this, "longitude", 0.0);
   /**
    * Roll of the camera. When it is zero, the camera is leveled.
    */
   private final DoubleProperty roll = new SimpleDoubleProperty(this, "roll", 0.0);
   /**
    * When set to true, the camera roll is zeroed to keep the camera level.
    */
   private final BooleanProperty keepRotationLeveled = new SimpleBooleanProperty(this, "keepRotationLeveled", true);
   /** Condition to trigger the use of the fast modifier to make the camera rotate faster when using the mouse. */
   private final ObjectProperty<Predicate<MouseEvent>> fastModifierPredicate = new SimpleObjectProperty<>(this, "fastModifierPredicate", null);
   /** Slow camera rotation modifier when using the mouse. */
   private final DoubleProperty slowModifier = new SimpleDoubleProperty(this, "slowModifier", 0.005);
   /** Fast camera rotation modifier when using the mouse. It is triggered when the condition held in {@link #fastModifierPredicate} is fulfilled. */
   private final DoubleProperty fastModifier = new SimpleDoubleProperty(this, "fastModifier", 0.010);
   /** Camera roll modifier when using the mouse. */
   private final DoubleProperty rollModifier = new SimpleDoubleProperty(this, "rollModifier", 0.005);
   /** Indicates which mouse button triggers the camera rotation when using the {@link EventHandler} via {@link #createMouseEventHandler(ReadOnlyDoubleProperty, ReadOnlyDoubleProperty)}. */
   private final ObjectProperty<MouseButton> rotationMouseButton = new SimpleObjectProperty<>(this, "rotationMouseButton", MouseButton.PRIMARY);

   /** When set to true, the latitude of the camera is restricted to the range [{@link #minLatitude}, {@link #maxLatitude}]. */
   private final BooleanProperty restrictLatitude = new SimpleBooleanProperty(this, "restrictLatitude", true);
   /** Minimum latitude of the camera. Only used when {@link #restrictLatitude} is set to true. */
   private final DoubleProperty minLatitude = new SimpleDoubleProperty(this, "minLatitude", -Math.PI / 2.0 + 0.2);
   /** Maximum latitude of the camera. Only used when {@link #restrictLatitude} is set to true. */
   private final DoubleProperty maxLatitude = new SimpleDoubleProperty(this, "maxLatitude", Math.PI / 2.0 - 0.2);

   private final Vector3D up = new Vector3D();
   private final Vector3D down = new Vector3D();
   private final Vector3D forward = new Vector3D();

   /**
    * Creates a calculator for the camera rotation.
    * @param up indicates which way is up.
    * @param forward indicates which is forward.
    * @throws RuntimeException if {@code up} and {@code forward} are not orthogonal.
    */
   public CameraRotationCalculator(Vector3D up, Vector3D forward)
   {
      Vector3D left = new Vector3D();
      left.cross(up, forward);
      if (!MathTools.epsilonEquals(left.length(), 1.0, 1.0 - 5))
         throw new RuntimeException("The vectors up and forward must be orthogonal. Received: up = " + up + ", forward = " + forward);

      this.up.set(up);
      this.forward.set(forward);
      down.setAndNegate(up);

      computeOffset();
   }

   private void computeOffset()
   {
      Vector3D cameraZAxis = new Vector3D(forward);
      Vector3D cameraYAxis = new Vector3D(down);
      Vector3D cameraXAxis = new Vector3D();
      cameraXAxis.cross(cameraYAxis, cameraZAxis);
      RotationMatrix rotationOffset = new RotationMatrix();
      rotationOffset.setColumns(cameraXAxis, cameraYAxis, cameraZAxis);
      JavaFXTools.convertRotationMatrixToAffine(rotationOffset, offset);
   }

   /**
    * Creates an {@link EventHandler} to rotate the camera when a mouse click and drag is performed.
    * @param sceneWidthProperty width of the scene the camera belongs to.
    * @param sceneHeightProperty height of the scene the camera belongs to.
    * @return an {@link EventHandler} to rotate the camera with the mouse.
    */
   public EventHandler<MouseEvent> createMouseEventHandler(ReadOnlyDoubleProperty sceneWidthProperty, ReadOnlyDoubleProperty sceneHeightProperty)
   {
      return new EventHandler<MouseEvent>()
      {
         private final Point2D oldMouseLocation = new Point2D();

         @Override
         public void handle(MouseEvent event)
         {
            if (event.getButton() != rotationMouseButton.get())
               return;

            // Filters single clicks
            if (event.isStillSincePress())
               return;

            if (event.getEventType() == MouseEvent.MOUSE_PRESSED)
            {
               oldMouseLocation.set(event.getSceneX(), event.getSceneY());
               return;
            }

            if (event.getEventType() != MouseEvent.MOUSE_DRAGGED)
               return;

            Point2D centerLocation = new Point2D();
            Point2D newMouseLocation = new Point2D();
            // Acquire the new mouse coordinates from the recent event
            centerLocation.set(sceneWidthProperty.get() / 2.0, sceneHeightProperty.get() / 2.0);
            newMouseLocation.set(event.getSceneX(), event.getSceneY());

            double modifier;
            if (fastModifierPredicate.get() == null || !fastModifierPredicate.get().test(event))
               modifier = slowModifier.get();
            else
               modifier = fastModifier.get();

            Vector2D drag = new Vector2D();
            drag.sub(newMouseLocation, oldMouseLocation);

            Vector2D centerToMouseLocation = new Vector2D();
            centerToMouseLocation.sub(newMouseLocation, centerLocation);
            double rollShift = 0.0 * modifier * rollModifier.get() * drag.cross(centerToMouseLocation);

            drag.scale(modifier);
            updateRotation(drag.getY(), -drag.getX(), rollShift);

            oldMouseLocation.set(newMouseLocation);
         }
      };
   }

   /**
    * Update the camera rotation after applying rotation offsets.
    * @param deltaLatitude the shift in latitude to apply to the camera rotation.
    * @param deltaLongitude the shift in longitude to apply to the camera rotation.
    * @param deltaRoll the shift in roll to apply to the camera rotation.
    */
   public void updateRotation(double deltaLatitude, double deltaLongitude, double deltaRoll)
   {
      double newLatitude = latitude.get() + deltaLatitude;
      if (restrictLatitude.get())
         newLatitude = MathTools.clamp(newLatitude, minLatitude.get(), maxLatitude.get());
      else
         newLatitude = MathTools.clamp(newLatitude, Math.PI);
      latitude.set(newLatitude);
      double newLongitude = longitude.get() + deltaLongitude;
      newLongitude = AngleTools.trimAngleMinusPiToPi(newLongitude);
      longitude.set(newLongitude);

      if (keepRotationLeveled.get())
      {
         roll.set(0.0);
      }
      else
      {
         double newRoll = roll.get() + deltaRoll;
         newRoll = AngleTools.trimAngleMinusPiToPi(newRoll);
         roll.set(newRoll);
      }

      updateRotation();
   }

   /**
    * Computes and sets the camera rotation for given camera position and a given point to focus on.
    * @param cameraPosition desired camera position. Not modified.
    * @param focusPoint desired focus position. Not modified.
    * @param cameraRoll desired camera roll.
    */
   public void setRotationFromCameraAndFocusPositions(Point3D cameraPosition, Point3D focusPoint, double cameraRoll)
   {
      Vector3D fromFocusToCamera = new Vector3D();
      fromFocusToCamera.sub(cameraPosition, focusPoint);
      fromFocusToCamera.normalize();
      Vector3D fromCameraToFocus = new Vector3D();
      fromCameraToFocus.setAndNegate(fromFocusToCamera);
      // We remove the component along up to be able to compute the longitude
      fromCameraToFocus.scaleAdd(-fromCameraToFocus.dot(up), up, fromCameraToFocus);

      double newLatitude = Math.PI / 2.0 - fromFocusToCamera.angle(up);
      double newLongitude = fromCameraToFocus.angle(forward);

      Vector3D cross = new Vector3D();
      cross.cross(fromCameraToFocus, forward);

      if (cross.dot(up) > 0.0)
         newLongitude = -newLongitude;

      latitude.set(newLatitude);
      longitude.set(newLongitude);
      roll.set(cameraRoll);
      updateRotation();
   }

   /**
    * Sets the camera's latitude, longitude, and roll.
    * @param latitude the new camera latitude.
    * @param longitude the new camera longitude.
    * @param roll the new camera roll.
    */
   public void setRotation(double latitude, double longitude, double roll)
   {
      if (restrictLatitude.get())
         this.latitude.set(MathTools.clamp(latitude, minLatitude.get(), maxLatitude.get()));
      else
         this.latitude.set(MathTools.clamp(latitude, Math.PI / 2.0));

      this.longitude.set(AngleTools.trimAngleMinusPiToPi(longitude));
      this.roll.set(AngleTools.trimAngleMinusPiToPi(roll));
      updateRotation();
   }

   private void updateRotation()
   {
      Rotate latitudeRotate = new Rotate(Math.toDegrees(-latitude.get()), Rotate.X_AXIS);
      Rotate longitudeRotate = new Rotate(Math.toDegrees(-longitude.get()), Rotate.Y_AXIS);
      Rotate rollRotate = new Rotate(Math.toDegrees(roll.get()), Rotate.Z_AXIS);

      Affine newRotation = new Affine();
      newRotation.append(offset);
      newRotation.append(longitudeRotate);
      newRotation.append(latitudeRotate);
      newRotation.append(rollRotate);

      rotation.setToTransform(newRotation);
   }

   /**
    * Get the reference to the rotation of the camera.
    * This is the output of this calculator which can be bound to an external property or used directly to apply a transformation to the camera.
    * @return the camera's rotation.
    */
   public Affine getRotation()
   {
      return rotation;
   }

   public final DoubleProperty latitudeProperty()
   {
      return latitude;
   }

   public final double getLatitude()
   {
      return latitudeProperty().get();
   }

   public final void setLatitude(final double latitude)
   {
      latitudeProperty().set(latitude);
   }

   public final DoubleProperty longitudeProperty()
   {
      return longitude;
   }

   public final double getLongitude()
   {
      return longitudeProperty().get();
   }

   public final void setLongitude(final double longitude)
   {
      longitudeProperty().set(longitude);
   }

   public final DoubleProperty rollProperty()
   {
      return roll;
   }

   public final double getRoll()
   {
      return rollProperty().get();
   }

   public final void setRoll(final double roll)
   {
      rollProperty().set(roll);
   }

   public final BooleanProperty keepRotationLeveledProperty()
   {
      return keepRotationLeveled;
   }

   public final boolean isKeepRotationLeveled()
   {
      return keepRotationLeveledProperty().get();
   }

   public final void setKeepRotationLeveled(final boolean keepRotationLeveled)
   {
      keepRotationLeveledProperty().set(keepRotationLeveled);
   }

   public final ObjectProperty<Predicate<MouseEvent>> fastModifierPredicateProperty()
   {
      return fastModifierPredicate;
   }

   public final Predicate<MouseEvent> getFastModifierPredicate()
   {
      return fastModifierPredicateProperty().get();
   }

   public final void setFastModifierPredicate(final Predicate<MouseEvent> fastModifierPredicate)
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

   public final DoubleProperty rollModifierProperty()
   {
      return rollModifier;
   }

   public final double getRollModifier()
   {
      return rollModifierProperty().get();
   }

   public final void setRollModifier(final double rollModifier)
   {
      rollModifierProperty().set(rollModifier);
   }

   public final ObjectProperty<MouseButton> rotationMouseButtonProperty()
   {
      return rotationMouseButton;
   }

   public final MouseButton getRotationMouseButton()
   {
      return rotationMouseButtonProperty().get();
   }

   public final void setRotationMouseButton(final MouseButton rotationMouseButton)
   {
      rotationMouseButtonProperty().set(rotationMouseButton);
   }

   public final BooleanProperty restrictLatitudeProperty()
   {
      return restrictLatitude;
   }

   public final boolean isRestrictLatitude()
   {
      return restrictLatitudeProperty().get();
   }

   public final void setRestrictLatitude(final boolean restrictLatitude)
   {
      restrictLatitudeProperty().set(restrictLatitude);
   }

   public final DoubleProperty minLatitudeProperty()
   {
      return minLatitude;
   }

   public final double getMinLatitude()
   {
      return minLatitudeProperty().get();
   }

   public final void setMinLatitude(final double minLatitude)
   {
      minLatitudeProperty().set(minLatitude);
   }

   public final DoubleProperty maxLatitudeProperty()
   {
      return maxLatitude;
   }

   public final double getMaxLatitude()
   {
      return maxLatitudeProperty().get();
   }

   public final void setMaxLatitude(final double maxLatitude)
   {
      maxLatitudeProperty().set(maxLatitude);
   }
}
