package us.ihmc.yoVariables.euclid.filters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameTuple3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.filters.BacklashProcessingYoVariable;
import us.ihmc.yoVariables.filters.ProcessingYoVariable;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;

/**
 * This class is designed to perform an estimate of a velocity signal that may contain backlash. It does essentially the same as
 * {@link BacklashCompensatingVelocityYoFrameVector3D}, except it takes a velocity signal as input.
 *
 * It works by zeroing out the estimated velocity whenever the finite-differenced velocity changes sign. It then ramps this value back up to the value returned
 * by finite estimating over the course of the slop duration. It assumes that the slop time is some fixed, constant period when the estimate is unreliable.
 */
public class BacklashProcessingYoFrameVector3D extends YoFrameVector3D implements ProcessingYoVariable
{
   private final BacklashProcessingYoVariable xDot, yDot, zDot;

   public BacklashProcessingYoFrameVector3D(String namePrefix, String nameSuffix, double dt, DoubleProvider slopTime,
                                            YoRegistry registry, YoFrameTuple3D yoFrameTupleToProcess)
   {
      this(new BacklashProcessingYoVariable(YoGeometryNameTools.createXName(namePrefix, nameSuffix), "", yoFrameTupleToProcess.getYoX(), dt, slopTime, registry),
           new BacklashProcessingYoVariable(YoGeometryNameTools.createYName(namePrefix, nameSuffix), "", yoFrameTupleToProcess.getYoY(), dt, slopTime, registry),
           new BacklashProcessingYoVariable(YoGeometryNameTools.createZName(namePrefix, nameSuffix), "", yoFrameTupleToProcess.getYoZ(), dt, slopTime, registry),
           yoFrameTupleToProcess.getReferenceFrame());
   }

   private BacklashProcessingYoFrameVector3D(BacklashProcessingYoVariable xDot, BacklashProcessingYoVariable yDot, BacklashProcessingYoVariable zDot,
                                             ReferenceFrame referenceFrame)
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
