package us.ihmc.javaFXToolkit.cameraControllers;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.event.EventHandler;
import javafx.scene.PerspectiveCamera;
import javafx.scene.input.ScrollEvent;
import javafx.scene.transform.Translate;
import us.ihmc.robotics.MathTools;

/**
 * This class provides the tools necessary to build a simple controller for the zoom of a JavaFX {@link PerspectiveCamera}.
 * This class is ready to be used with an {@link EventHandler} via {@link #createScrollEventHandler()}.
 * The output of this calculator is the {@link #zoom} property which can be bound to an external property or a component of {@link Translate} to apply a transformation on the camera.
 * This transformation not implemented here to provide increased flexibility.
 * @author Sylvain Bertrand
 */
public class CameraZoomCalculator
{
   /**
    * The current zoom of the camera. In {@link FocusBasedCameraMouseEventHandler} it is used as the distance between the camera and the point it focuses on.
    * This is the output of this calculator and can be bound to an external property or a component of {@link Translate} to apply a transformation on the camera.
    */
   private final DoubleProperty zoom = new SimpleDoubleProperty(this, "zoom", 10.0);
   /** Minimum value the zoom can be. */
   private final DoubleProperty minZoom = new SimpleDoubleProperty(this, "minZoom", 0.1);
   /** Maximum value the zoom can be. */
   private final DoubleProperty maxZoom = new SimpleDoubleProperty(this, "maxZoom", 100.0);
   /** Zoom speed factor with respect to its current value. The larger is the zoom, the faster it "goes". */
   private final DoubleProperty zoomSpeedFactor = new SimpleDoubleProperty(this, "zoomSpeedFactor", 0.1);
   /**
    * <p> Only applicable when using the {@link EventHandler} via {@link #createScrollEventHandler()}. </p>
    * When set to true, the direction of the zoom is reversed.
    */
   private final BooleanProperty invertZoomDirection = new SimpleBooleanProperty(this, "invertZoomDirection", false);

   public CameraZoomCalculator()
   {
   }

   /**
    * @return an {@link EventHandler} for {@link ScrollEvent} that uses the mouse wheel to update the zoom value.
    */
   public EventHandler<ScrollEvent> createScrollEventHandler()
   {
      return new EventHandler<ScrollEvent>()
      {
         @Override
         public void handle(ScrollEvent event)
         {
            double direction = Math.signum(event.getDeltaY());
            if (invertZoomDirection.get())
               direction = -direction;
            zoomInternal(direction);
         }
      };
   }

   /**
    * Zoom in once.
    */
   public void zoomIn()
   {
      zoomInternal(-1.0);
   }

   /**
    * Zoom out once.
    */
   public void zoomOut()
   {
      zoomInternal(1.0);
   }

   private void zoomInternal(double direction)
   {
      double newOffset = zoom.get() + direction * zoom.get() * zoomSpeedFactor.get();
      newOffset = MathTools.clamp(newOffset, minZoom.get(), maxZoom.get());
      zoom.set(newOffset);
   }

   public final DoubleProperty zoomProperty()
   {
      return zoom;
   }

   public final double getZoom()
   {
      return zoomProperty().get();
   }

   public final void setZoom(final double zoom)
   {
      zoomProperty().set(zoom);
   }

   public final DoubleProperty minZoomProperty()
   {
      return minZoom;
   }

   public final double getMinZoom()
   {
      return minZoomProperty().get();
   }

   public final void setMinZoom(final double minZoom)
   {
      minZoomProperty().set(minZoom);
   }

   public final DoubleProperty maxZoomProperty()
   {
      return maxZoom;
   }

   public final double getMaxZoom()
   {
      return maxZoomProperty().get();
   }

   public final void setMaxZoom(final double maxZoom)
   {
      maxZoomProperty().set(maxZoom);
   }

   public final DoubleProperty zoomSpeedFactorProperty()
   {
      return zoomSpeedFactor;
   }

   public final double getZoomSpeedFactor()
   {
      return zoomSpeedFactorProperty().get();
   }

   public final void setZoomSpeedFactor(final double zoomSpeedFactor)
   {
      zoomSpeedFactorProperty().set(zoomSpeedFactor);
   }

   public final BooleanProperty invertZoomDirectionProperty()
   {
      return invertZoomDirection;
   }

   public final boolean isInvertZoomDirection()
   {
      return invertZoomDirectionProperty().get();
   }

   public final void setInvertZoomDirection(final boolean invertZoomDirection)
   {
      invertZoomDirectionProperty().set(invertZoomDirection);
   }
}
