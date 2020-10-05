package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;

public class BetaFilteredYoFramePoint2d extends YoFramePoint2D
{
   private final BetaFilteredYoVariable x, y;

   private BetaFilteredYoFramePoint2d(BetaFilteredYoVariable x, BetaFilteredYoVariable y, ReferenceFrame referenceFrame)
   {
      super(x, y, referenceFrame);

      this.x = x;
      this.y = y;
   }

   public static BetaFilteredYoFramePoint2d createBetaFilteredYoFramePoint2d(String namePrefix, String nameSuffix, YoRegistry registry, int beta, ReferenceFrame referenceFrame)
   {
      // beta is a int
      BetaFilteredYoVariable x = new BetaFilteredYoVariable(YoGeometryNameTools.createXName(namePrefix, nameSuffix), registry, beta);
      BetaFilteredYoVariable y = new BetaFilteredYoVariable(YoGeometryNameTools.createYName(namePrefix, nameSuffix), registry, beta);

      BetaFilteredYoFramePoint2d ret = new BetaFilteredYoFramePoint2d(x, y, referenceFrame);

      return ret;
   }

   public static BetaFilteredYoFramePoint2d createBetaFilteredYoFramePoint2d(String namePrefix, String nameSuffix, String description, YoRegistry registry, int beta, ReferenceFrame referenceFrame)
   {
      // beta is a int
      BetaFilteredYoVariable x = new BetaFilteredYoVariable(YoGeometryNameTools.createXName(namePrefix, nameSuffix), description, registry, beta);
      BetaFilteredYoVariable y = new BetaFilteredYoVariable(YoGeometryNameTools.createYName(namePrefix, nameSuffix), description, registry, beta);

      BetaFilteredYoFramePoint2d ret = new BetaFilteredYoFramePoint2d(x, y, referenceFrame);

      return ret;
   }

   public static BetaFilteredYoFramePoint2d createBetaFilteredYoFramePoint2d(String namePrefix, String nameSuffix, YoRegistry registry, int beta, YoFramePoint2D unfilteredPoint)
   {
      // beta is a int
      BetaFilteredYoVariable x = new BetaFilteredYoVariable(YoGeometryNameTools.createXName(namePrefix, nameSuffix), registry, beta, unfilteredPoint.getYoX());
      BetaFilteredYoVariable y = new BetaFilteredYoVariable(YoGeometryNameTools.createYName(namePrefix, nameSuffix), registry, beta, unfilteredPoint.getYoY());

      BetaFilteredYoFramePoint2d ret = new BetaFilteredYoFramePoint2d(x, y, unfilteredPoint.getReferenceFrame());

      return ret;
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

   public void update(Point2D point2dUnfiltered)
   {
      x.update(point2dUnfiltered.getX());
      y.update(point2dUnfiltered.getY());
   }

   public void update(FramePoint2D point2dUnfiltered)
   {
      checkReferenceFrameMatch(point2dUnfiltered);
      x.update(point2dUnfiltered.getX());
      y.update(point2dUnfiltered.getY());
   }

   public void reset()
   {
      x.reset();
      y.reset();
   }
}
