package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

/**
 * <p>
 * {@link FilteredFiniteDifferenceYoFrameVector3D}
 * </p>
 *
 * <p>
 * Differentiates and Filters a {@link YoFrameVector3D} to get its derivative. This derviative is then low pass filtered.
 * </p>
 * <pre>
 *            vel_{n} = alpha * vel{n-1} + (1 - alpha) * (pos_{n} - pos_{n-1})
 * </pre>
 */
public class FilteredFiniteDifferenceYoFrameVector3D extends YoFrameVector3D
{
   private final double dt;
   private final DoubleProvider alphaProvider;

   private final YoBoolean hasBeenCalled;
   private final FrameTuple3DReadOnly currentPosition;
   private final YoFrameVector3D lastPosition;

   public FilteredFiniteDifferenceYoFrameVector3D(String namePrefix,
                                                  String nameSuffix,
                                                  DoubleProvider alpha,
                                                  double dt,
                                                  YoRegistry registry,
                                                  FrameTuple3DReadOnly frameTuple3DToDifferentiate)
   {
      this(namePrefix, nameSuffix, alpha, dt, registry, frameTuple3DToDifferentiate, frameTuple3DToDifferentiate.getReferenceFrame());
   }

   public FilteredFiniteDifferenceYoFrameVector3D(String namePrefix,
                                                  String nameSuffix,
                                                  DoubleProvider alpha,
                                                  double dt,
                                                  YoRegistry registry,
                                                  ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, alpha, dt, registry, null, referenceFrame);
   }

   private FilteredFiniteDifferenceYoFrameVector3D(String namePrefix,
                                                   String nameSuffix,
                                                   DoubleProvider alpha,
                                                   double dt,
                                                   YoRegistry registry,
                                                   FrameTuple3DReadOnly frameTuple3DToDifferentiate,
                                                   ReferenceFrame referenceFrame)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);

      this.alphaProvider = alpha;
      this.dt = dt;

      hasBeenCalled = VariableTools.createHasBeenCalledYoBoolean(namePrefix, nameSuffix, registry);
      currentPosition = frameTuple3DToDifferentiate;
      lastPosition = new YoFrameVector3D(namePrefix + "_lastPosition", nameSuffix, getReferenceFrame(), registry);
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

   public void update(FrameTuple3DReadOnly frameTuple)
   {
      checkReferenceFrameMatch(frameTuple);
      update((Tuple3DReadOnly) frameTuple);
   }

   private final Vector3D currentRawDerivative = new Vector3D();

   public void update(Tuple3DReadOnly currentPosition)
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
