package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;

public class BacklashCompensatingVelocityYoFrameVector extends YoFrameVector3D
{
   private final BacklashCompensatingVelocityYoVariable xDot, yDot, zDot;

   public static BacklashCompensatingVelocityYoFrameVector createBacklashCompensatingVelocityYoFrameVector(String namePrefix, String nameSuffix, YoDouble alpha, double dt, YoDouble slopTime,
           YoVariableRegistry registry, YoFrameVector3D yoFrameVectorToDifferentiate)
   {
      BacklashCompensatingVelocityYoVariable xDot = new BacklashCompensatingVelocityYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), "", alpha, yoFrameVectorToDifferentiate.getYoX(), dt, slopTime,
                                           registry);
      BacklashCompensatingVelocityYoVariable yDot = new BacklashCompensatingVelocityYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), "", alpha, yoFrameVectorToDifferentiate.getYoY(), dt, slopTime,
                                           registry);
      BacklashCompensatingVelocityYoVariable zDot = new BacklashCompensatingVelocityYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), "", alpha, yoFrameVectorToDifferentiate.getYoZ(), dt, slopTime,
                                           registry);

      BacklashCompensatingVelocityYoFrameVector ret = new BacklashCompensatingVelocityYoFrameVector(xDot, yDot, zDot, registry, yoFrameVectorToDifferentiate);

      return ret;
   }

   public static BacklashCompensatingVelocityYoFrameVector createBacklashCompensatingVelocityYoFrameVector(String namePrefix, String nameSuffix, YoDouble alpha, double dt, YoDouble slopTime,
           YoVariableRegistry registry, YoFramePoint3D yoFramePointToDifferentiate)
   {
      BacklashCompensatingVelocityYoVariable xDot = new BacklashCompensatingVelocityYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), "", alpha, yoFramePointToDifferentiate.getYoX(), dt, slopTime,
                                           registry);
      BacklashCompensatingVelocityYoVariable yDot = new BacklashCompensatingVelocityYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), "", alpha, yoFramePointToDifferentiate.getYoY(), dt, slopTime, 
                                           registry);
      BacklashCompensatingVelocityYoVariable zDot = new BacklashCompensatingVelocityYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), "", alpha, yoFramePointToDifferentiate.getYoZ(), dt, slopTime,
                                           registry);

      BacklashCompensatingVelocityYoFrameVector ret = new BacklashCompensatingVelocityYoFrameVector(xDot, yDot, zDot, registry, yoFramePointToDifferentiate);

      return ret;
   }

   private BacklashCompensatingVelocityYoFrameVector(BacklashCompensatingVelocityYoVariable xDot, BacklashCompensatingVelocityYoVariable yDot, BacklashCompensatingVelocityYoVariable zDot,
           YoVariableRegistry registry, YoFrameVector3D yoFrameVectorToDifferentiate)
   {
      super(xDot, yDot, zDot, yoFrameVectorToDifferentiate.getReferenceFrame());

      this.xDot = xDot;
      this.yDot = yDot;
      this.zDot = zDot;
   }

   private BacklashCompensatingVelocityYoFrameVector(BacklashCompensatingVelocityYoVariable xDot, BacklashCompensatingVelocityYoVariable yDot, BacklashCompensatingVelocityYoVariable zDot, 
           YoVariableRegistry registry, YoFramePoint3D yoFramePointToDifferentiate)
   {
      super(xDot, yDot, zDot, yoFramePointToDifferentiate.getReferenceFrame());

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
