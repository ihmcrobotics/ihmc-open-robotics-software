package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameTuple;
import us.ihmc.robotics.math.frames.YoFrameTuple;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class DeadzoneYoFrameVector extends YoFrameVector implements ProcessingYoVariable
{
   private final DeadzoneYoVariable x, y, z;

   private DeadzoneYoFrameVector(DeadzoneYoVariable x, DeadzoneYoVariable y, DeadzoneYoVariable z, ReferenceFrame referenceFrame)
   {
      super(x, y, z, referenceFrame);

      this.x = x;
      this.y = y;
      this.z = z;
   }

   public static DeadzoneYoFrameVector createDeadzoneYoFrameVector(String namePrefix, YoVariableRegistry registry, DoubleYoVariable deadzoneSize, ReferenceFrame referenceFrame)
   {
      return createDeadzoneYoFrameVector(namePrefix, "", registry, deadzoneSize, referenceFrame);
   }

   public static DeadzoneYoFrameVector createDeadzoneYoFrameVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, DoubleYoVariable deadzoneSize, ReferenceFrame referenceFrame)
   {
      DeadzoneYoVariable x = new DeadzoneYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), deadzoneSize, registry);
      DeadzoneYoVariable y = new DeadzoneYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), deadzoneSize, registry);
      DeadzoneYoVariable z = new DeadzoneYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), deadzoneSize, registry);

      DeadzoneYoFrameVector ret = new DeadzoneYoFrameVector(x, y, z, referenceFrame);

      return ret;
   }

   public static DeadzoneYoFrameVector createDeadzoneYoFrameVector(String namePrefix, YoVariableRegistry registry, DoubleYoVariable deadzoneSize, YoFrameTuple<?> rawTuple)
   {
      return createDeadzoneYoFrameVector(namePrefix, "", registry, deadzoneSize, rawTuple);
   }

   public static DeadzoneYoFrameVector createDeadzoneYoFrameVector(String namePrefix, String nameSuffix, YoVariableRegistry registry, DoubleYoVariable deadzoneSize, YoFrameTuple<?> rawTuple)
   {
      DeadzoneYoVariable x = new DeadzoneYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), rawTuple.getYoX(), deadzoneSize, registry);
      DeadzoneYoVariable y = new DeadzoneYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), rawTuple.getYoY(), deadzoneSize, registry);
      DeadzoneYoVariable z = new DeadzoneYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), rawTuple.getYoZ(), deadzoneSize, registry);

      DeadzoneYoFrameVector ret = new DeadzoneYoFrameVector(x, y, z, rawTuple.getReferenceFrame());

      return ret;
   }

   @Override
   public void update()
   {
      x.update();
      y.update();
      z.update();
   }

   public void update(FrameTuple<?> frameTuple)
   {
      checkReferenceFrameMatch(frameTuple);

      x.update(frameTuple.getX());
      y.update(frameTuple.getY());
      z.update(frameTuple.getZ());
   }
}
