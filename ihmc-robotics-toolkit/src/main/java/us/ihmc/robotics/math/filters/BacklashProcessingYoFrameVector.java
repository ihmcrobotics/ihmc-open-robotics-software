package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameTuple3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class BacklashProcessingYoFrameVector extends YoFrameVector3D implements ProcessingYoVariable
{
   private final BacklashProcessingYoVariable xDot, yDot, zDot;

   public static BacklashProcessingYoFrameVector createBacklashProcessingYoFrameVector(String namePrefix, String nameSuffix, double dt, DoubleProvider slopTime,
           YoVariableRegistry registry, YoFrameTuple3D yoFrameTupleToProcess)
   {
      String xName = YoFrameVariableNameTools.createXName(namePrefix, nameSuffix);
      String yName = YoFrameVariableNameTools.createYName(namePrefix, nameSuffix);
      String zName = YoFrameVariableNameTools.createZName(namePrefix, nameSuffix);

      YoDouble xRaw = yoFrameTupleToProcess.getYoX();
      YoDouble yRaw = yoFrameTupleToProcess.getYoY();
      YoDouble zRaw = yoFrameTupleToProcess.getYoZ();

      BacklashProcessingYoVariable x = new BacklashProcessingYoVariable(xName, "", xRaw, dt, slopTime, registry);
      BacklashProcessingYoVariable y = new BacklashProcessingYoVariable(yName, "", yRaw, dt, slopTime, registry);
      BacklashProcessingYoVariable z = new BacklashProcessingYoVariable(zName, "", zRaw, dt, slopTime, registry);

      ReferenceFrame referenceFrame = yoFrameTupleToProcess.getReferenceFrame();

      return new BacklashProcessingYoFrameVector(x, y, z, registry, referenceFrame);
   }

   private BacklashProcessingYoFrameVector(BacklashProcessingYoVariable xDot, BacklashProcessingYoVariable yDot, BacklashProcessingYoVariable zDot,
           YoVariableRegistry registry, ReferenceFrame referenceFrame)
   {
      super(xDot, yDot, zDot, referenceFrame);

      this.xDot = xDot;
      this.yDot = yDot;
      this.zDot = zDot;
   }

   @Override
   public void update()
   {
      xDot.update();
      yDot.update();
      zDot.update();
   }

   @Override
   public void reset()
   {
      xDot.reset();
      yDot.reset();
      zDot.reset();
   }
}
