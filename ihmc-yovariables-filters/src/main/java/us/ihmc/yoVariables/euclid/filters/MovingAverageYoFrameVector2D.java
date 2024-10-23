package us.ihmc.yoVariables.euclid.filters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.filters.MovingAverageYoDouble;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;

public class MovingAverageYoFrameVector2D extends YoFrameVector2D
{
   private final MovingAverageYoDouble x, y;

   public MovingAverageYoFrameVector2D(String namePrefix, String nameSuffix, YoRegistry registry, int beta, ReferenceFrame referenceFrame)
   {
      this(new MovingAverageYoDouble(YoGeometryNameTools.createXName(namePrefix, nameSuffix), registry, beta),
           new MovingAverageYoDouble(YoGeometryNameTools.createYName(namePrefix, nameSuffix), registry, beta),
           referenceFrame);
   }

   public MovingAverageYoFrameVector2D(String namePrefix, String nameSuffix, YoRegistry registry, int beta, YoFrameVector2D unfilteredVector)
   {
      this(new MovingAverageYoDouble(YoGeometryNameTools.createXName(namePrefix, nameSuffix), registry, beta, unfilteredVector.getYoX()),
           new MovingAverageYoDouble(YoGeometryNameTools.createYName(namePrefix, nameSuffix), registry, beta, unfilteredVector.getYoY()),
           unfilteredVector.getReferenceFrame());
   }

   private MovingAverageYoFrameVector2D(MovingAverageYoDouble x, MovingAverageYoDouble y, ReferenceFrame referenceFrame)
   {
      super(x, y, referenceFrame);

      this.x = x;
      this.y = y;
   }

   public void update()
   {
      x.update();
      y.update();
   }

   public void update(double xUnfiltered, double yUnfiltered)
   {
      x.update(xUnfiltered);
      y.update(yUnfiltered);
   }

   public void update(Vector2DReadOnly vector2dUnfiltered)
   {
      update(vector2dUnfiltered.getX(), vector2dUnfiltered.getY());
   }

   public void update(FrameVector2DReadOnly vector2dUnfiltered)
   {
      checkReferenceFrameMatch(vector2dUnfiltered);
      update((Vector2DReadOnly) vector2dUnfiltered);
   }

   public void reset()
   {
      x.reset();
      y.reset();
   }
}
