package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

/**
 * <p>FilteredVelocityYoFrameVector </p>
 *
 * <p>Differentiates and Filters a YoFrameVector to get its derivative. </p>
 *
 * <p>IHMC </p>
 *
 * @author IHMC Biped Team
 * @version 1.0
 */
public class FilteredVelocityYoFrameVector extends YoFrameVector3D
{
   private final FilteredVelocityYoVariable xDot, yDot, zDot;

   public static FilteredVelocityYoFrameVector createFilteredVelocityYoFrameVector(String namePrefix, String nameSuffix, YoDouble alpha, double dt,
         YoVariableRegistry registry, YoFrameVector3D yoFrameVectorToDifferentiate)
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

   public static FilteredVelocityYoFrameVector createFilteredVelocityYoFrameVector(String namePrefix, String nameSuffix, YoDouble alpha, double dt,
         YoVariableRegistry registry, YoFramePoint3D yoFramePointToDifferentiate)
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

   public static FilteredVelocityYoFrameVector createFilteredVelocityYoFrameVector(String namePrefix, String nameSuffix, YoDouble alpha, double dt,
         YoVariableRegistry registry, ReferenceFrame referenceFrame)
   {
      FilteredVelocityYoVariable xDot = new FilteredVelocityYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), "", alpha, dt, registry);
      FilteredVelocityYoVariable yDot = new FilteredVelocityYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), "", alpha, dt, registry);
      FilteredVelocityYoVariable zDot = new FilteredVelocityYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), "", alpha, dt, registry);

      FilteredVelocityYoFrameVector ret = new FilteredVelocityYoFrameVector(xDot, yDot, zDot, alpha, dt, registry, referenceFrame);

      return ret;
   }

   private FilteredVelocityYoFrameVector(FilteredVelocityYoVariable xDot, FilteredVelocityYoVariable yDot, FilteredVelocityYoVariable zDot,
         YoDouble alpha, double dt, YoVariableRegistry registry, ReferenceFrame referenceFrame)
   {
      super(xDot, yDot, zDot, referenceFrame);

      this.xDot = xDot;
      this.yDot = yDot;
      this.zDot = zDot;
   }

   private FilteredVelocityYoFrameVector(FilteredVelocityYoVariable xDot, FilteredVelocityYoVariable yDot, FilteredVelocityYoVariable zDot,
         YoDouble alpha, double dt, YoVariableRegistry registry, YoFramePoint3D yoFramePointToDifferentiate)
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

   public void update(Tuple3DReadOnly tuple)
   {
      xDot.update(tuple.getX());
      yDot.update(tuple.getY());
      zDot.update(tuple.getZ());
   }

   public void update(FrameTuple3DReadOnly frameTuple)
   {
      checkReferenceFrameMatch(frameTuple);
      xDot.update(frameTuple.getX());
      yDot.update(frameTuple.getY());
      zDot.update(frameTuple.getZ());
   }

   public void reset()
   {
      xDot.reset();
      yDot.reset();
      zDot.reset();
   }
}
