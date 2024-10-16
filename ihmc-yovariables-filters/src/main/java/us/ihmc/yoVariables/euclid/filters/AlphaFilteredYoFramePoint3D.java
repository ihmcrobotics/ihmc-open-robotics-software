package us.ihmc.yoVariables.euclid.filters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.filters.VariableTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class AlphaFilteredYoFramePoint3D extends YoFramePoint3D
{
   private final DoubleProvider alpha;

   private final FrameTuple3DReadOnly unfilteredPoint;
   private final YoBoolean hasBeenCalled;

   public AlphaFilteredYoFramePoint3D(String namePrefix, String nameSuffix, YoRegistry registry, double alpha, ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, VariableTools.createAlphaYoDouble(namePrefix, nameSuffix, alpha, registry), referenceFrame);
   }

   public AlphaFilteredYoFramePoint3D(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider alpha, ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, alpha, referenceFrame, null);
   }

   public AlphaFilteredYoFramePoint3D(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider alpha,
                                      FrameTuple3DReadOnly unfilteredPoint)
   {
      this(namePrefix, nameSuffix, registry, alpha, unfilteredPoint.getReferenceFrame(), unfilteredPoint);
   }

   private AlphaFilteredYoFramePoint3D(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider alpha, ReferenceFrame referenceFrame,
                                       FrameTuple3DReadOnly unfilteredPoint)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);

      this.alpha = alpha;
      this.unfilteredPoint = unfilteredPoint;

      hasBeenCalled = VariableTools.createHasBeenCalledYoBoolean(namePrefix, nameSuffix, registry);
      reset();
   }

   public void reset()
   {
      hasBeenCalled.set(false);
   }

   public void update()
   {
      if (unfilteredPoint == null)
      {
         throw new NullPointerException(getClass().getSimpleName() + " must be constructed with a non null "
               + "position variable to call update(), otherwise use update(double)");
      }

      update(unfilteredPoint);
   }

   public void update(FrameTuple3DReadOnly unfilteredPoint)
   {
      checkReferenceFrameMatch(unfilteredPoint);
      update((Tuple3DReadOnly) unfilteredPoint);
   }

   public void update(Tuple3DReadOnly unfilteredTuple3D)
   {
      update(unfilteredTuple3D.getX(), unfilteredTuple3D.getY(), unfilteredTuple3D.getZ());
   }

   private final Point3D unfilteredPoint3D = new Point3D();

   public void update(double xUnfiltered, double yUnfiltered, double zUnfiltered)
   {
      if (!hasBeenCalled.getValue())
      {
         hasBeenCalled.set(true);
         set(xUnfiltered, yUnfiltered, zUnfiltered);
      }
      else
      {
         unfilteredPoint3D.set(xUnfiltered, yUnfiltered, zUnfiltered);
         interpolate(unfilteredPoint3D, this, alpha.getValue());
      }
   }
}
