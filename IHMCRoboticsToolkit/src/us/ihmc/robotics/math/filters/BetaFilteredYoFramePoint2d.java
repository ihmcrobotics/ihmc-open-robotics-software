package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Point2d;

public class BetaFilteredYoFramePoint2d extends YoFramePoint2d
{
   private final BetaFilteredYoVariable x, y;

   private BetaFilteredYoFramePoint2d(BetaFilteredYoVariable x, BetaFilteredYoVariable y, ReferenceFrame referenceFrame)
   {
      super(x, y, referenceFrame);

      this.x = x;
      this.y = y;
   }

   public static BetaFilteredYoFramePoint2d createBetaFilteredYoFramePoint2d(String namePrefix, String nameSuffix, YoVariableRegistry registry, int beta, ReferenceFrame referenceFrame)
   {
      // beta is a int
      BetaFilteredYoVariable x = new BetaFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, beta);
      BetaFilteredYoVariable y = new BetaFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, beta);

      BetaFilteredYoFramePoint2d ret = new BetaFilteredYoFramePoint2d(x, y, referenceFrame);

      return ret;
   }

   public static BetaFilteredYoFramePoint2d createBetaFilteredYoFramePoint2d(String namePrefix, String nameSuffix, String description, YoVariableRegistry registry, int beta, ReferenceFrame referenceFrame)
   {
      // beta is a int
      BetaFilteredYoVariable x = new BetaFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), description, registry, beta);
      BetaFilteredYoVariable y = new BetaFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), description, registry, beta);

      BetaFilteredYoFramePoint2d ret = new BetaFilteredYoFramePoint2d(x, y, referenceFrame);

      return ret;
   }

   public static BetaFilteredYoFramePoint2d createBetaFilteredYoFramePoint2d(String namePrefix, String nameSuffix, YoVariableRegistry registry, int beta, YoFramePoint2d unfilteredPoint)
   {
      // beta is a int
      BetaFilteredYoVariable x = new BetaFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, beta, unfilteredPoint.getYoX());
      BetaFilteredYoVariable y = new BetaFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, beta, unfilteredPoint.getYoY());

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

   public void update(Point2d point2dUnfiltered)
   {
      x.update(point2dUnfiltered.getX());
      y.update(point2dUnfiltered.getY());
   }

   public void update(FramePoint2d point2dUnfiltered)
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
