package us.ihmc.yoVariables.euclid.filters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.filters.VariableTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class AlphaFilteredYoFramePoint2D extends YoFramePoint2D
{
   private final DoubleProvider alpha;

   private final FrameTuple2DReadOnly unfilteredPoint;
   private final YoBoolean hasBeenCalled;

   public AlphaFilteredYoFramePoint2D(String namePrefix, String nameSuffix, YoRegistry registry, double alpha, ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, VariableTools.createAlphaYoDouble(namePrefix, nameSuffix, alpha, registry), referenceFrame);
   }

   public AlphaFilteredYoFramePoint2D(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider alpha, ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, alpha, referenceFrame, null);
   }

   public AlphaFilteredYoFramePoint2D(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider alpha,
                                      FrameTuple2DReadOnly unfilteredPoint)
   {
      this(namePrefix, nameSuffix, registry, alpha, unfilteredPoint.getReferenceFrame(), unfilteredPoint);
   }

   private AlphaFilteredYoFramePoint2D(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider alpha, ReferenceFrame referenceFrame,
                                       FrameTuple2DReadOnly unfilteredPoint)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);

      this.alpha = alpha;
      this.unfilteredPoint = unfilteredPoint;
      if (unfilteredPoint != null)
         checkReferenceFrameMatch(unfilteredPoint);

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

   public void update(FrameTuple2DReadOnly unfilteredPoint)
   {
      checkReferenceFrameMatch(unfilteredPoint);
      update((Tuple2DReadOnly) unfilteredPoint);
   }

   public void update(Tuple2DReadOnly unfilteredPoint)
   {
      update(unfilteredPoint.getX(), unfilteredPoint.getY());
   }

   private final Point2D unfilteredPoint2D = new Point2D();

   public void update(double xUnfiltered, double yUnfiltered)
   {
      if (!hasBeenCalled.getValue())
      {
         hasBeenCalled.set(true);
         set(xUnfiltered, yUnfiltered);
      }
      else
      {
         unfilteredPoint2D.set(xUnfiltered, yUnfiltered);
         interpolate(unfilteredPoint2D, this, alpha.getValue());
      }
   }
}
