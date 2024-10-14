package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;

public class MovingAverageYoFramePoint2D extends YoFramePoint2D
{
   private final MovingAverageYoDouble x, y;

   public MovingAverageYoFramePoint2D(String namePrefix, String nameSuffix, YoRegistry registry, int beta, ReferenceFrame referenceFrame)
   {
      this(new MovingAverageYoDouble(YoGeometryNameTools.createXName(namePrefix, nameSuffix), registry, beta),
           new MovingAverageYoDouble(YoGeometryNameTools.createYName(namePrefix, nameSuffix), registry, beta),
           referenceFrame);
   }

   public MovingAverageYoFramePoint2D(String namePrefix, String nameSuffix, YoRegistry registry, int beta, YoFramePoint2D unfilteredPoint)
   {
      this(new MovingAverageYoDouble(YoGeometryNameTools.createXName(namePrefix, nameSuffix), registry, beta, unfilteredPoint.getYoX()),
           new MovingAverageYoDouble(YoGeometryNameTools.createYName(namePrefix, nameSuffix), registry, beta, unfilteredPoint.getYoY()),
           unfilteredPoint.getReferenceFrame());
   }

   private MovingAverageYoFramePoint2D(MovingAverageYoDouble x, MovingAverageYoDouble y, ReferenceFrame referenceFrame)
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

   public void update(Point2DReadOnly point2dUnfiltered)
   {
      update(point2dUnfiltered.getX(), point2dUnfiltered.getY());
   }

   public void update(FramePoint2DReadOnly point2dUnfiltered)
   {
      checkReferenceFrameMatch(point2dUnfiltered);
      update((Point2DReadOnly) point2dUnfiltered);
   }

   public void reset()
   {
      x.reset();
      y.reset();
   }
}
