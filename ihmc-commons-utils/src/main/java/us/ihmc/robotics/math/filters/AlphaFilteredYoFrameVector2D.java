package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class AlphaFilteredYoFrameVector2D extends YoFrameVector2D
{
   private final DoubleProvider alpha;

   private final FrameTuple2DReadOnly unfilteredPosition;
   private final YoBoolean hasBeenCalled;

   public AlphaFilteredYoFrameVector2D(String namePrefix, String nameSuffix, YoRegistry registry, double alpha, ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, VariableTools.createAlphaYoDouble(namePrefix, nameSuffix, alpha, registry), referenceFrame);
   }

   public AlphaFilteredYoFrameVector2D(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider alpha, ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, alpha, referenceFrame, null);
   }

   public AlphaFilteredYoFrameVector2D(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider alpha,
                                       FrameTuple2DReadOnly unfilteredFrameTuple2D)
   {
      this(namePrefix, nameSuffix, registry, alpha, unfilteredFrameTuple2D.getReferenceFrame(), unfilteredFrameTuple2D);
   }

   private AlphaFilteredYoFrameVector2D(String namePrefix, String nameSuffix, YoRegistry registry, DoubleProvider alpha, ReferenceFrame referenceFrame,
                                        FrameTuple2DReadOnly unfilteredPosition)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);

      this.alpha = alpha;
      this.unfilteredPosition = unfilteredPosition;

      hasBeenCalled = VariableTools.createHasBeenCalledYoBoolean(namePrefix, nameSuffix, registry);
      reset();
   }

   public void reset()
   {
      hasBeenCalled.set(false);
   }

   public void update()
   {
      if (unfilteredPosition == null)
      {
         throw new NullPointerException(getClass().getSimpleName() + " must be constructed with a non null "
               + "position variable to call update(), otherwise use update(double)");
      }

      update(unfilteredPosition);
   }

   public void update(FrameTuple2DReadOnly unfilteredFrameTuple2D)
   {
      checkReferenceFrameMatch(unfilteredFrameTuple2D);
      update((Tuple2DReadOnly) unfilteredFrameTuple2D);
   }

   public void update(Tuple2DReadOnly unfilteredTuple2D)
   {
      update(unfilteredTuple2D.getX(), unfilteredTuple2D.getY());
   }

   private final Vector2D unfilteredVector2D = new Vector2D();

   public void update(double xUnfiltered, double yUnfiltered)
   {
      if (!hasBeenCalled.getValue())
      {
         hasBeenCalled.set(true);
         set(xUnfiltered, yUnfiltered);
      }
      else
      {
         unfilteredVector2D.set(xUnfiltered, yUnfiltered);
         interpolate(unfilteredVector2D, this, alpha.getValue());
      }
   }
}
