package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameTuple;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameTuple;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * <p>FilteredVelocityYoFrameVector </p>
 *
 * <p>Differentiates and Filters a YoFrameVector to get its derivative. </p>
 *
 * <p>Copyright (c) 2008</p>
 *
 * <p>IHMC and Yobotics </p>
 *
 * @author IHMC-Yobotics Biped Team
 * @version 1.0
 */
public class FilteredVelocityYoFrameVector extends YoFrameVector
{
   private final FilteredVelocityYoVariable xDot, yDot, zDot;

   public static FilteredVelocityYoFrameVector createFilteredVelocityYoFrameVector(String namePrefix, String nameSuffix, DoubleYoVariable alpha, double dt,
         YoVariableRegistry registry, YoFrameVector yoFrameVectorToDifferentiate)
   {
      FilteredVelocityYoVariable xDot = new FilteredVelocityYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), "", alpha,
            yoFrameVectorToDifferentiate.getYoX(), dt, registry);
      FilteredVelocityYoVariable yDot = new FilteredVelocityYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), "", alpha,
            yoFrameVectorToDifferentiate.getYoY(), dt, registry);
      FilteredVelocityYoVariable zDot = new FilteredVelocityYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), "", alpha,
            yoFrameVectorToDifferentiate.getYoZ(), dt, registry);

      FilteredVelocityYoFrameVector ret = new FilteredVelocityYoFrameVector(xDot, yDot, zDot, alpha, dt, registry,
            yoFrameVectorToDifferentiate.getReferenceFrame());

      return ret;
   }

   public static FilteredVelocityYoFrameVector createFilteredVelocityYoFrameVector(String namePrefix, String nameSuffix, DoubleYoVariable alpha, double dt,
         YoVariableRegistry registry, YoFramePoint yoFramePointToDifferentiate)
   {
      FilteredVelocityYoVariable xDot = new FilteredVelocityYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), "", alpha,
            yoFramePointToDifferentiate.getYoX(), dt, registry);
      FilteredVelocityYoVariable yDot = new FilteredVelocityYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), "", alpha,
            yoFramePointToDifferentiate.getYoY(), dt, registry);
      FilteredVelocityYoVariable zDot = new FilteredVelocityYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), "", alpha,
            yoFramePointToDifferentiate.getYoZ(), dt, registry);

      FilteredVelocityYoFrameVector ret = new FilteredVelocityYoFrameVector(xDot, yDot, zDot, alpha, dt, registry,
            yoFramePointToDifferentiate.getReferenceFrame());

      return ret;
   }

   public static FilteredVelocityYoFrameVector createFilteredVelocityYoFrameVector(String namePrefix, String nameSuffix, DoubleYoVariable alpha, double dt,
         YoVariableRegistry registry, ReferenceFrame referenceFrame)
   {
      FilteredVelocityYoVariable xDot = new FilteredVelocityYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), "", alpha, dt, registry);
      FilteredVelocityYoVariable yDot = new FilteredVelocityYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), "", alpha, dt, registry);
      FilteredVelocityYoVariable zDot = new FilteredVelocityYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), "", alpha, dt, registry);

      FilteredVelocityYoFrameVector ret = new FilteredVelocityYoFrameVector(xDot, yDot, zDot, alpha, dt, registry, referenceFrame);

      return ret;
   }

   private FilteredVelocityYoFrameVector(FilteredVelocityYoVariable xDot, FilteredVelocityYoVariable yDot, FilteredVelocityYoVariable zDot,
         DoubleYoVariable alpha, double dt, YoVariableRegistry registry, ReferenceFrame referenceFrame)
   {
      super(xDot, yDot, zDot, referenceFrame);

      this.xDot = xDot;
      this.yDot = yDot;
      this.zDot = zDot;
   }

   private FilteredVelocityYoFrameVector(FilteredVelocityYoVariable xDot, FilteredVelocityYoVariable yDot, FilteredVelocityYoVariable zDot,
         DoubleYoVariable alpha, double dt, YoVariableRegistry registry, YoFramePoint yoFramePointToDifferentiate)
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

   public void update(Tuple3DBasics tuple)
   {
      xDot.update(tuple.getX());
      yDot.update(tuple.getY());
      zDot.update(tuple.getZ());
   }

   public void update(FrameTuple<?, ?> frameTuple)
   {
      checkReferenceFrameMatch(frameTuple);
      xDot.update(frameTuple.getX());
      yDot.update(frameTuple.getY());
      zDot.update(frameTuple.getZ());
   }

   public void update(YoFrameTuple<?, ?> yoFrameTuple)
   {
      checkReferenceFrameMatch(yoFrameTuple.getReferenceFrame());
      xDot.update(yoFrameTuple.getX());
      yDot.update(yoFrameTuple.getY());
      zDot.update(yoFrameTuple.getZ());
   }

   public void reset()
   {
      xDot.reset();
      yDot.reset();
      zDot.reset();
   }
}
