package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.robotics.math.frames.YoFrameVector2d;

public class BetaFilteredYoFrameVector2d extends YoFrameVector2d
{
   private final BetaFilteredYoVariable x, y;

   private BetaFilteredYoFrameVector2d(BetaFilteredYoVariable x, BetaFilteredYoVariable y, ReferenceFrame referenceFrame)
   {
      super(x, y, referenceFrame);

      this.x = x;
      this.y = y;
   }

   public static BetaFilteredYoFrameVector2d createBetaFilteredYoFrameVector2d(String namePrefix, String nameSuffix, YoVariableRegistry registry, int beta, ReferenceFrame referenceFrame)
   {
      // beta is a int
      BetaFilteredYoVariable x = new BetaFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, beta);
      BetaFilteredYoVariable y = new BetaFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, beta);

      BetaFilteredYoFrameVector2d ret = new BetaFilteredYoFrameVector2d(x, y, referenceFrame);

      return ret;
   }

   public static BetaFilteredYoFrameVector2d createBetaFilteredYoFrameVector2d(String namePrefix, String nameSuffix, YoVariableRegistry registry, int beta, YoFrameVector2d unfilteredVector)
   {
      // beta is a int
      BetaFilteredYoVariable x = new BetaFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, beta, unfilteredVector.getYoX());
      BetaFilteredYoVariable y = new BetaFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, beta, unfilteredVector.getYoY());

      BetaFilteredYoFrameVector2d ret = new BetaFilteredYoFrameVector2d(x, y, unfilteredVector.getReferenceFrame());

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

   public void update(Vector2D vector2dUnfiltered)
   {
      x.update(vector2dUnfiltered.getX());
      y.update(vector2dUnfiltered.getY());
   }

   public void update(FrameVector2D vector2dUnfiltered)
   {
      checkReferenceFrameMatch(vector2dUnfiltered);
      x.update(vector2dUnfiltered.getX());
      y.update(vector2dUnfiltered.getY());
   }

   public void reset()
   {
      x.reset();
      y.reset();
   }
}
