package us.ihmc.yoVariables.euclid.filters;

import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.filters.BacklashCompensatingVelocityYoVariable;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoDouble;

public class BacklashCompensatingVelocityYoFrameVector3D extends YoFrameVector3D
{
   private final BacklashCompensatingVelocityYoVariable xDot, yDot, zDot;

   public BacklashCompensatingVelocityYoFrameVector3D(String namePrefix,
                                                      String nameSuffix,
                                                      YoDouble alpha,
                                                      double dt,
                                                      YoDouble slopTime,
                                                      YoRegistry registry,
                                                      YoFramePoint3D yoFramePointToDifferentiate)
   {
      this(new BacklashCompensatingVelocityYoVariable(YoGeometryNameTools.createXName(namePrefix, nameSuffix),
                                                      "",
                                                      alpha,
                                                      yoFramePointToDifferentiate.getYoX(),
                                                      dt,
                                                      slopTime,
                                                      registry),
           new BacklashCompensatingVelocityYoVariable(YoGeometryNameTools.createYName(namePrefix, nameSuffix),
                                                      "",
                                                      alpha,
                                                      yoFramePointToDifferentiate.getYoY(),
                                                      dt,
                                                      slopTime,
                                                      registry),
           new BacklashCompensatingVelocityYoVariable(YoGeometryNameTools.createZName(namePrefix, nameSuffix),
                                                      "",
                                                      alpha,
                                                      yoFramePointToDifferentiate.getYoZ(),
                                                      dt,
                                                      slopTime,
                                                      registry),
           yoFramePointToDifferentiate);
   }

   private BacklashCompensatingVelocityYoFrameVector3D(BacklashCompensatingVelocityYoVariable xDot,
                                                       BacklashCompensatingVelocityYoVariable yDot,
                                                       BacklashCompensatingVelocityYoVariable zDot,
                                                       YoFramePoint3D yoFramePointToDifferentiate)
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
