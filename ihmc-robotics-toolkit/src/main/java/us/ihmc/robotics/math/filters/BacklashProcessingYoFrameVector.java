package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameTuple3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoDouble;

public class BacklashProcessingYoFrameVector extends YoFrameVector3D implements ProcessingYoVariable
{
   private final BacklashProcessingYoVariable xDot, yDot, zDot;

   public static BacklashProcessingYoFrameVector createBacklashProcessingYoFrameVector(String namePrefix, String nameSuffix, double dt, DoubleProvider slopTime,
           YoRegistry registry, YoFrameTuple3D yoFrameTupleToProcess)
   {
      String xName = YoGeometryNameTools.createXName(namePrefix, nameSuffix);
      String yName = YoGeometryNameTools.createYName(namePrefix, nameSuffix);
      String zName = YoGeometryNameTools.createZName(namePrefix, nameSuffix);

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
           YoRegistry registry, ReferenceFrame referenceFrame)
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
