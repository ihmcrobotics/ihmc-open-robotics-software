package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class AlphaFilteredYoFramePoint2d extends YoFramePoint2d
{
   private final AlphaFilteredYoVariable x, y;

   private AlphaFilteredYoFramePoint2d(AlphaFilteredYoVariable x, AlphaFilteredYoVariable y, ReferenceFrame referenceFrame)
   {
      super(x, y, referenceFrame);

      this.x = x;
      this.y = y;
   }

   public static AlphaFilteredYoFramePoint2d createAlphaFilteredYoFramePoint2d(String namePrefix, String nameSuffix, YoVariableRegistry registry, double alpha, ReferenceFrame referenceFrame)
   {
      // alpha is a double
      AlphaFilteredYoVariable x = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, alpha);
      AlphaFilteredYoVariable y = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, alpha);

      AlphaFilteredYoFramePoint2d ret = new AlphaFilteredYoFramePoint2d(x, y, referenceFrame);

      return ret;
   }

   public static AlphaFilteredYoFramePoint2d createAlphaFilteredYoFramePoint2d(String namePrefix, String nameSuffix, YoVariableRegistry registry, DoubleYoVariable alpha, ReferenceFrame referenceFrame)
   {
      return createAlphaFilteredYoFramePoint2d(namePrefix, nameSuffix, "", registry, alpha, referenceFrame);
   }

   public static AlphaFilteredYoFramePoint2d createAlphaFilteredYoFramePoint2d(String namePrefix, String nameSuffix, String description, YoVariableRegistry registry, DoubleYoVariable alpha, ReferenceFrame referenceFrame)
   {
      // alpha is a double
      AlphaFilteredYoVariable x = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), description, registry, alpha);
      AlphaFilteredYoVariable y = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), description, registry, alpha);

      AlphaFilteredYoFramePoint2d ret = new AlphaFilteredYoFramePoint2d(x, y, referenceFrame);

      return ret;
   }

   public static AlphaFilteredYoFramePoint2d createAlphaFilteredYoFramePoint2d(String namePrefix, String nameSuffix, YoVariableRegistry registry, double alpha, YoFramePoint2d unfilteredPoint)
   {
      // alpha is a double
      AlphaFilteredYoVariable x = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, alpha, unfilteredPoint.getYoX());
      AlphaFilteredYoVariable y = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, alpha, unfilteredPoint.getYoY());

      AlphaFilteredYoFramePoint2d ret = new AlphaFilteredYoFramePoint2d(x, y, unfilteredPoint.getReferenceFrame());

      return ret;
   }

   public static AlphaFilteredYoFramePoint2d createAlphaFilteredYoFramePoint2d(String namePrefix, String nameSuffix, YoVariableRegistry registry, DoubleYoVariable alpha, YoFramePoint2d unfilteredPoint)
   {
      // alpha is a YoVariable
      AlphaFilteredYoVariable x = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, alpha, unfilteredPoint.getYoX());
      AlphaFilteredYoVariable y = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, alpha, unfilteredPoint.getYoY());

      AlphaFilteredYoFramePoint2d ret = new AlphaFilteredYoFramePoint2d(x, y, unfilteredPoint.getReferenceFrame());

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
