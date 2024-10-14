package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

/**
 * <p>
 * {@link FilteredFiniteDifferenceYoFrameVector2D}
 * </p>
 *
 * <p>
 * Differentiates and Filters a {@link YoFrameVector2D} to get its derivative. This derviative is then low pass filtered.
 * </p>
 * <pre>
 *            vel_{n} = alpha * vel{n-1} + (1 - alpha) * (pos_{n} - pos_{n-1})
 * </pre>
 */
public class FilteredFiniteDifferenceYoFrameVector2D extends YoFrameVector2D
{
   private final double dt;
   private final DoubleProvider alphaProvider;

   private final YoBoolean hasBeenCalled;
   private final FrameTuple2DReadOnly currentPosition;
   private final YoFrameVector2D lastPosition;

   public FilteredFiniteDifferenceYoFrameVector2D(String namePrefix,
                                                  String nameSuffix,
                                                  DoubleProvider alpha,
                                                  double dt,
                                                  YoRegistry registry,
                                                  FrameTuple2DReadOnly frameTuple2DToDifferentiate)
   {
      this(namePrefix, nameSuffix, alpha, dt, registry, frameTuple2DToDifferentiate, frameTuple2DToDifferentiate.getReferenceFrame());
   }

   public FilteredFiniteDifferenceYoFrameVector2D(String namePrefix,
                                                  String nameSuffix,
                                                  DoubleProvider alpha,
                                                  double dt,
                                                  YoRegistry registry,
                                                  ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, alpha, dt, registry, null, referenceFrame);
   }

   private FilteredFiniteDifferenceYoFrameVector2D(String namePrefix,
                                                   String nameSuffix,
                                                   DoubleProvider alpha,
                                                   double dt,
                                                   YoRegistry registry,
                                                   FrameTuple2DReadOnly frameTuple2DToDifferentiate,
                                                   ReferenceFrame referenceFrame)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);

      this.alphaProvider = alpha;
      this.dt = dt;

      hasBeenCalled = VariableTools.createHasBeenCalledYoBoolean(namePrefix, nameSuffix, registry);
      currentPosition = frameTuple2DToDifferentiate;
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
