package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoFramePoint2D;

public class AlphaFilteredYoFramePoint2d extends YoFramePoint2D
{
   private final DoubleProvider alphaProvider;

   private final FrameTuple2DReadOnly position;
   private final YoBoolean hasBeenCalled;

   /**
    * @deprecated Use
    *             {@link #AlphaFilteredYoFramePoint2d(String, String, YoVariableRegistry, double, ReferenceFrame)}
    *             instead.
    */
   @Deprecated
   public static AlphaFilteredYoFramePoint2d createAlphaFilteredYoFramePoint2d(String namePrefix, String nameSuffix, YoVariableRegistry registry, double alpha,
                                                                               ReferenceFrame referenceFrame)
   {
      return new AlphaFilteredYoFramePoint2d(namePrefix, nameSuffix, registry, alpha, referenceFrame);
   }

   /**
    * @deprecated Use
    *             {@link #AlphaFilteredYoFramePoint2d(String, String, YoVariableRegistry, DoubleProvider, ReferenceFrame)}
    *             instead.
    */
   @Deprecated
   public static AlphaFilteredYoFramePoint2d createAlphaFilteredYoFramePoint2d(String namePrefix, String nameSuffix, YoVariableRegistry registry,
                                                                               DoubleProvider alpha, ReferenceFrame referenceFrame)
   {
      return createAlphaFilteredYoFramePoint2d(namePrefix, nameSuffix, "", registry, alpha, referenceFrame);
   }

   /**
    * @deprecated Use
    *             {@link #AlphaFilteredYoFramePoint2d(String, String, YoVariableRegistry, DoubleProvider, ReferenceFrame)}
    *             instead.
    */
   @Deprecated
   public static AlphaFilteredYoFramePoint2d createAlphaFilteredYoFramePoint2d(String namePrefix, String nameSuffix, String description,
                                                                               YoVariableRegistry registry, DoubleProvider alpha, ReferenceFrame referenceFrame)
   {
      return new AlphaFilteredYoFramePoint2d(namePrefix, nameSuffix, registry, alpha, referenceFrame);
   }

   /**
    * @deprecated Use
    *             {@link #AlphaFilteredYoFramePoint2d(String, String, YoVariableRegistry, double, FrameTuple2DReadOnly)}
    *             instead.
    */
   @Deprecated
   public static AlphaFilteredYoFramePoint2d createAlphaFilteredYoFramePoint2d(String namePrefix, String nameSuffix, YoVariableRegistry registry, double alpha,
                                                                               FrameTuple2DReadOnly unfilteredFrameTuple2D)
   {
      return new AlphaFilteredYoFramePoint2d(namePrefix, nameSuffix, registry, alpha, unfilteredFrameTuple2D);
   }

   /**
    * @deprecated Use
    *             {@link #AlphaFilteredYoFramePoint2d(String, String, YoVariableRegistry, DoubleProvider, FrameTuple2DReadOnly)}
    *             instead.
    */
   @Deprecated
   public static AlphaFilteredYoFramePoint2d createAlphaFilteredYoFramePoint2d(String namePrefix, String nameSuffix, YoVariableRegistry registry,
                                                                               DoubleProvider alpha, FrameTuple2DReadOnly unfilteredFrameTuple2D)
   {
      return new AlphaFilteredYoFramePoint2d(namePrefix, nameSuffix, registry, alpha, unfilteredFrameTuple2D);
   }

   public AlphaFilteredYoFramePoint2d(String namePrefix, String nameSuffix, YoVariableRegistry registry, double alpha, ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, AlphaFilteredYoVariable.createAlphaYoDouble(namePrefix + nameSuffix, alpha, registry), referenceFrame);
   }

   public AlphaFilteredYoFramePoint2d(String namePrefix, String nameSuffix, YoVariableRegistry registry, DoubleProvider alpha, ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, alpha, referenceFrame, null);
   }

   public AlphaFilteredYoFramePoint2d(String namePrefix, String nameSuffix, YoVariableRegistry registry, double alpha,
                                      FrameTuple2DReadOnly unfilteredFrameTuple2D)
   {
      this(namePrefix, nameSuffix, registry, AlphaFilteredYoVariable.createAlphaYoDouble(namePrefix + nameSuffix, alpha, registry),
           unfilteredFrameTuple2D.getReferenceFrame(), unfilteredFrameTuple2D);
   }

   public AlphaFilteredYoFramePoint2d(String namePrefix, String nameSuffix, YoVariableRegistry registry, DoubleProvider alpha,
                                      FrameTuple2DReadOnly unfilteredFrameTuple2D)
   {
      this(namePrefix, nameSuffix, registry, alpha, unfilteredFrameTuple2D.getReferenceFrame(), unfilteredFrameTuple2D);
   }

   private AlphaFilteredYoFramePoint2d(String namePrefix, String nameSuffix, YoVariableRegistry registry, DoubleProvider alpha, ReferenceFrame referenceFrame,
                                       FrameTuple2DReadOnly unfilteredFrameTuple2D)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);

      alphaProvider = alpha;

      position = unfilteredFrameTuple2D;
      hasBeenCalled = new YoBoolean(namePrefix + nameSuffix + "HasBeenCalled", registry);
      reset();
   }

   public void reset()
   {
      hasBeenCalled.set(false);
   }

   public void update()
   {
      if (position == null)
      {
         throw new NullPointerException(getClass().getSimpleName() + " must be constructed with a non null "
               + "position variable to call update(), otherwise use update(double)");
      }

      update(position);
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
         interpolate(unfilteredPoint2D, this, alphaProvider.getValue());
      }
   }
}
