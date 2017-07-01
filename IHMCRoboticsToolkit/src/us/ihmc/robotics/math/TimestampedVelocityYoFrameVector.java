package us.ihmc.robotics.math;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.robotics.math.frames.YoFrameVector;

public class TimestampedVelocityYoFrameVector extends YoFrameVector
{
   private final TimestampedVelocityYoVariable xDot, yDot, zDot;

   public static TimestampedVelocityYoFrameVector createFilteredVelocityYoFrameVector(String namePrefix, String nameSuffix, YoDouble timestamp,
           double dt, YoVariableRegistry registry, YoFrameVector yoFrameVectorToDifferentiate, double epsilonChange)
   {
      TimestampedVelocityYoVariable xDot = new TimestampedVelocityYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), "", yoFrameVectorToDifferentiate.getYoX(),
                                              timestamp, registry, epsilonChange);
      TimestampedVelocityYoVariable yDot = new TimestampedVelocityYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), "", yoFrameVectorToDifferentiate.getYoY(),
                                              timestamp, registry, epsilonChange);
      TimestampedVelocityYoVariable zDot = new TimestampedVelocityYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), "", yoFrameVectorToDifferentiate.getYoZ(),
                                              timestamp, registry, epsilonChange);

      TimestampedVelocityYoFrameVector ret = new TimestampedVelocityYoFrameVector(xDot, yDot, zDot, yoFrameVectorToDifferentiate.getReferenceFrame());

      return ret;
   }

   public static TimestampedVelocityYoFrameVector createFilteredVelocityYoFrameVector(String namePrefix, String nameSuffix, YoDouble timestamp,
           double dt, YoVariableRegistry registry, YoFramePoint yoFramePointToDifferentiate, double epsilonChange)
   {
      TimestampedVelocityYoVariable xDot = new TimestampedVelocityYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), "", yoFramePointToDifferentiate.getYoX(),
                                              timestamp, registry, epsilonChange);
      TimestampedVelocityYoVariable yDot = new TimestampedVelocityYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), "", yoFramePointToDifferentiate.getYoY(),
                                              timestamp, registry, epsilonChange);
      TimestampedVelocityYoVariable zDot = new TimestampedVelocityYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), "", yoFramePointToDifferentiate.getYoZ(),
                                              timestamp, registry, epsilonChange);

      TimestampedVelocityYoFrameVector ret = new TimestampedVelocityYoFrameVector(xDot, yDot, zDot, yoFramePointToDifferentiate.getReferenceFrame());

      return ret;
   }

   private TimestampedVelocityYoFrameVector(TimestampedVelocityYoVariable xDot, TimestampedVelocityYoVariable yDot, TimestampedVelocityYoVariable zDot,
           ReferenceFrame referenceFrame)
   {
      super(xDot, yDot, zDot, referenceFrame);

      this.xDot = xDot;
      this.yDot = yDot;
      this.zDot = zDot;
   }

   public void update()
   {
      xDot.update();
      yDot.update();
      zDot.update();
   }

   public void reset()
   {
      xDot.reset();
      yDot.reset();
      zDot.reset();
   }
}
