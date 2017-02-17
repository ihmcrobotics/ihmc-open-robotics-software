package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;


public class AlphaFilteredYoFrameVector2d extends YoFrameVector2d
{
   private final AlphaFilteredYoVariable x, y;

   private AlphaFilteredYoFrameVector2d(AlphaFilteredYoVariable x, AlphaFilteredYoVariable y, ReferenceFrame referenceFrame)
   {
      super(x, y, referenceFrame);

      this.x = x;
      this.y = y;
   }

   public static AlphaFilteredYoFrameVector2d createAlphaFilteredYoFrameVector2d(String namePrefix, String nameSuffix, YoVariableRegistry registry, double alpha, ReferenceFrame referenceFrame)
   {
      // alpha is a double
      AlphaFilteredYoVariable x = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, alpha);
      AlphaFilteredYoVariable y = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, alpha);

      AlphaFilteredYoFrameVector2d ret = new AlphaFilteredYoFrameVector2d(x, y, referenceFrame);

      return ret;
   }

   public static AlphaFilteredYoFrameVector2d createAlphaFilteredYoFrameVector2d(String namePrefix, String nameSuffix, YoVariableRegistry registry, DoubleYoVariable alpha, ReferenceFrame referenceFrame)
   {
      return createAlphaFilteredYoFrameVector2d(namePrefix, nameSuffix, "", registry, alpha, referenceFrame);
   }

   public static AlphaFilteredYoFrameVector2d createAlphaFilteredYoFrameVector2d(String namePrefix, String nameSuffix, String description, YoVariableRegistry registry, DoubleYoVariable alpha, ReferenceFrame referenceFrame)
   {
      AlphaFilteredYoVariable x = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), description, registry, alpha);
      AlphaFilteredYoVariable y = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), description, registry, alpha);

      AlphaFilteredYoFrameVector2d ret = new AlphaFilteredYoFrameVector2d(x, y, referenceFrame);

      return ret;
   }

   public static AlphaFilteredYoFrameVector2d createAlphaFilteredYoFrameVector2d(String namePrefix, String nameSuffix, YoVariableRegistry registry, double alpha, YoFrameVector2d unfilteredVector)
   {
      // alpha is a double
      AlphaFilteredYoVariable x = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry, alpha, unfilteredVector.getYoX());
      AlphaFilteredYoVariable y = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry, alpha, unfilteredVector.getYoY());

      AlphaFilteredYoFrameVector2d ret = new AlphaFilteredYoFrameVector2d(x, y, unfilteredVector.getReferenceFrame());

      return ret;
   }


   public static AlphaFilteredYoFrameVector2d createAlphaFilteredYoFrameVector2d(String namePrefix, String nameSuffix, YoVariableRegistry registry, DoubleYoVariable alpha, YoFrameVector2d unfilteredVector)
   {
      return createAlphaFilteredYoFrameVector2d(namePrefix, nameSuffix, "", registry, alpha, unfilteredVector);
   }

   public static AlphaFilteredYoFrameVector2d createAlphaFilteredYoFrameVector2d(String namePrefix, String nameSuffix, String description, YoVariableRegistry registry, DoubleYoVariable alpha, YoFrameVector2d unfilteredVector)
   {
      // alpha is a YoVariable
      AlphaFilteredYoVariable x = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), description, registry, alpha, unfilteredVector.getYoX());
      AlphaFilteredYoVariable y = new AlphaFilteredYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), description, registry, alpha, unfilteredVector.getYoY());

      AlphaFilteredYoFrameVector2d ret = new AlphaFilteredYoFrameVector2d(x, y, unfilteredVector.getReferenceFrame());

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

   public void update(FrameVector2d vector2dUnfiltered)
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
