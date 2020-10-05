package us.ihmc.robotics.math;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoDouble;

public class TimestampedVelocityYoFrameVector extends YoFrameVector3D
{
   private final TimestampedVelocityYoVariable xDot, yDot, zDot;

   public static TimestampedVelocityYoFrameVector createFilteredVelocityYoFrameVector(String namePrefix, String nameSuffix, YoDouble timestamp,
           double dt, YoRegistry registry, YoFrameVector3D yoFrameVectorToDifferentiate, double epsilonChange)
   {
      TimestampedVelocityYoVariable xDot = new TimestampedVelocityYoVariable(YoGeometryNameTools.createXName(namePrefix, nameSuffix), "", yoFrameVectorToDifferentiate.getYoX(),
                                              timestamp, registry, epsilonChange);
      TimestampedVelocityYoVariable yDot = new TimestampedVelocityYoVariable(YoGeometryNameTools.createYName(namePrefix, nameSuffix), "", yoFrameVectorToDifferentiate.getYoY(),
                                              timestamp, registry, epsilonChange);
      TimestampedVelocityYoVariable zDot = new TimestampedVelocityYoVariable(YoGeometryNameTools.createZName(namePrefix, nameSuffix), "", yoFrameVectorToDifferentiate.getYoZ(),
                                              timestamp, registry, epsilonChange);

      TimestampedVelocityYoFrameVector ret = new TimestampedVelocityYoFrameVector(xDot, yDot, zDot, yoFrameVectorToDifferentiate.getReferenceFrame());

      return ret;
   }

   public static TimestampedVelocityYoFrameVector createFilteredVelocityYoFrameVector(String namePrefix, String nameSuffix, YoDouble timestamp,
           double dt, YoRegistry registry, YoFramePoint3D yoFramePointToDifferentiate, double epsilonChange)
   {
      TimestampedVelocityYoVariable xDot = new TimestampedVelocityYoVariable(YoGeometryNameTools.createXName(namePrefix, nameSuffix), "", yoFramePointToDifferentiate.getYoX(),
                                              timestamp, registry, epsilonChange);
      TimestampedVelocityYoVariable yDot = new TimestampedVelocityYoVariable(YoGeometryNameTools.createYName(namePrefix, nameSuffix), "", yoFramePointToDifferentiate.getYoY(),
                                              timestamp, registry, epsilonChange);
      TimestampedVelocityYoVariable zDot = new TimestampedVelocityYoVariable(YoGeometryNameTools.createZName(namePrefix, nameSuffix), "", yoFramePointToDifferentiate.getYoZ(),
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
