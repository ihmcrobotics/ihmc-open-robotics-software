package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoFrameVector2D;

public class FilteredVelocityYoFrameVector2d extends YoFrameVector2D
{
   private final double dt;
   private final DoubleProvider alphaProvider;

   private final YoBoolean hasBeenCalled;
   private final FrameTuple2DReadOnly currentPosition;
   private final YoFrameVector2D lastPosition;

   /**
    * @deprecated Use
    *             {@link #FilteredVelocityYoFrameVector2d(String, String, DoubleProvider, double, YoVariableRegistry, FrameTuple2DReadOnly)}
    *             instead.
    */
   @Deprecated
   public static FilteredVelocityYoFrameVector2d createFilteredVelocityYoFrameVector2d(String namePrefix, String nameSuffix, DoubleProvider alpha, double dt,
                                                                                       YoVariableRegistry registry,
                                                                                       FrameTuple2DReadOnly frameTuple2DToDifferentiate)
   {
      return new FilteredVelocityYoFrameVector2d(namePrefix, nameSuffix, alpha, dt, registry, frameTuple2DToDifferentiate);
   }

   /**
    * @deprecated Use
    *             {@link #FilteredVelocityYoFrameVector2d(String, String, DoubleProvider, double, YoVariableRegistry, FrameTuple2DReadOnly)}
    *             instead.
    */
   @Deprecated
   public static FilteredVelocityYoFrameVector2d createFilteredVelocityYoFrameVector2d(String namePrefix, String nameSuffix, String description,
                                                                                       DoubleProvider alpha, double dt, YoVariableRegistry registry,
                                                                                       FrameTuple2DReadOnly frameTuple2DToDifferentiate)
   {
      return new FilteredVelocityYoFrameVector2d(namePrefix, nameSuffix, alpha, dt, registry, frameTuple2DToDifferentiate);
   }

   /**
    * @deprecated Use
    *             {@link #FilteredVelocityYoFrameVector2d(String, String, DoubleProvider, double, YoVariableRegistry, ReferenceFrame)}
    *             instead.
    */
   @Deprecated
   public static FilteredVelocityYoFrameVector2d createFilteredVelocityYoFrameVector2d(String namePrefix, String nameSuffix, DoubleProvider alpha, double dt,
                                                                                       YoVariableRegistry registry, ReferenceFrame referenceFrame)
   {
      return new FilteredVelocityYoFrameVector2d(namePrefix, nameSuffix, alpha, dt, registry, referenceFrame);
   }

   public FilteredVelocityYoFrameVector2d(String namePrefix, String nameSuffix, DoubleProvider alpha, double dt, YoVariableRegistry registry,
                                          FrameTuple2DReadOnly frameTuple2DToDifferentiate)
   {
      super(namePrefix, nameSuffix, frameTuple2DToDifferentiate.getReferenceFrame(), registry);
      this.alphaProvider = alpha;
      this.dt = dt;

      hasBeenCalled = new YoBoolean(namePrefix + nameSuffix + "HasBeenCalled", registry);
      currentPosition = frameTuple2DToDifferentiate;
      lastPosition = new YoFrameVector2D(namePrefix + "_lastPosition", nameSuffix, getReferenceFrame(), registry);
      reset();
   }

   public FilteredVelocityYoFrameVector2d(String namePrefix, String nameSuffix, DoubleProvider alpha, double dt, YoVariableRegistry registry,
                                          ReferenceFrame referenceFrame)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);

      this.alphaProvider = alpha;
      this.dt = dt;

      hasBeenCalled = new YoBoolean(namePrefix + nameSuffix + "HasBeenCalled", registry);
      currentPosition = null;
      lastPosition = new YoFrameVector2D(namePrefix + "_lastPosition", nameSuffix, getReferenceFrame(), registry);
      reset();
   }

   public void reset()
   {
      hasBeenCalled.set(false);
   }

   public void update()
   {
      if (currentPosition == null)
      {
         throw new NullPointerException(getClass().getSimpleName() + " must be constructed with a non null "
               + "position variable to call update(), otherwise use update(FrameTuple3DReadOnly)");
      }

      update(currentPosition);
   }

   public void update(FrameTuple2DReadOnly frameTuple)
   {
      checkReferenceFrameMatch(frameTuple);
      update((Tuple2DReadOnly) frameTuple);
   }

   private final Vector2D currentRawDerivative = new Vector2D();

   public void update(Tuple2DReadOnly currentPosition)
   {
      if (!hasBeenCalled.getBooleanValue())
      {
         hasBeenCalled.set(true);
         lastPosition.set(currentPosition);
         setToZero();
      }

      currentRawDerivative.sub(currentPosition, lastPosition);
      currentRawDerivative.scale(1.0 / dt);

      interpolate(currentRawDerivative, this, alphaProvider.getValue());

      lastPosition.set(currentPosition);
   }
}
