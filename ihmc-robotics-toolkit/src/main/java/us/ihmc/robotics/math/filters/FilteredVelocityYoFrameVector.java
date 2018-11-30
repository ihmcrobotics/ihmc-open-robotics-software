package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

/**
 * <p>
 * FilteredVelocityYoFrameVector
 * </p>
 *
 * <p>
 * Differentiates and Filters a YoFrameVector to get its derivative.
 * </p>
 *
 * <p>
 * IHMC
 * </p>
 *
 * @author IHMC Biped Team
 * @version 1.0
 */
public class FilteredVelocityYoFrameVector extends YoFrameVector3D
{
   private final double dt;
   private final DoubleProvider alphaProvider;

   private final YoBoolean hasBeenCalled;
   private final FrameTuple3DReadOnly currentPosition;
   private final YoFrameVector3D lastPosition;

   /**
    * @deprecated Use
    *             {@link #FilteredVelocityYoFrameVector(String, String, DoubleProvider, double, YoVariableRegistry, FrameTuple3DReadOnly)}
    *             instead.
    */
   @Deprecated
   public static FilteredVelocityYoFrameVector createFilteredVelocityYoFrameVector(String namePrefix, String nameSuffix, DoubleProvider alpha, double dt,
                                                                                   YoVariableRegistry registry,
                                                                                   FrameTuple3DReadOnly frameTuple3DToDifferentiate)
   {
      return new FilteredVelocityYoFrameVector(namePrefix, nameSuffix, alpha, dt, registry, frameTuple3DToDifferentiate);
   }

   /**
    * @deprecated Use
    *             {@link #FilteredVelocityYoFrameVector(String, String, DoubleProvider, double, YoVariableRegistry, ReferenceFrame)}
    *             instead.
    */
   @Deprecated
   public static FilteredVelocityYoFrameVector createFilteredVelocityYoFrameVector(String namePrefix, String nameSuffix, DoubleProvider alpha, double dt,
                                                                                   YoVariableRegistry registry, ReferenceFrame referenceFrame)
   {
      return new FilteredVelocityYoFrameVector(namePrefix, nameSuffix, alpha, dt, registry, referenceFrame);
   }

   public FilteredVelocityYoFrameVector(String namePrefix, String nameSuffix, DoubleProvider alpha, double dt, YoVariableRegistry registry,
                                        FrameTuple3DReadOnly frameTuple3DToDifferentiate)
   {
      super(namePrefix, nameSuffix, frameTuple3DToDifferentiate.getReferenceFrame(), registry);
      this.alphaProvider = alpha;
      this.dt = dt;

      hasBeenCalled = new YoBoolean(namePrefix + nameSuffix + "HasBeenCalled", registry);
      currentPosition = frameTuple3DToDifferentiate;
      lastPosition = new YoFrameVector3D(namePrefix + "_lastPosition", nameSuffix, getReferenceFrame(), registry);
      reset();
   }

   public FilteredVelocityYoFrameVector(String namePrefix, String nameSuffix, DoubleProvider alpha, double dt, YoVariableRegistry registry,
                                        ReferenceFrame referenceFrame)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);

      this.alphaProvider = alpha;
      this.dt = dt;

      hasBeenCalled = new YoBoolean(namePrefix + nameSuffix + "HasBeenCalled", registry);
      currentPosition = null;
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

      double alpha = alphaProvider.getValue();
      interpolate(currentRawDerivative, this, alpha);

      lastPosition.set(currentPosition);
   }
}
