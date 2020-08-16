package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoConstraintOptimizerParameters implements ConstraintOptimizerParametersReadOnly
{
   private static final double defaultMaxX = 0.5;
   private static final double defaultMaxY = 0.5;
   private static final double defaultDeltaInside = 0.0;
   private static final boolean defaultConstrainMaxAdjustment = true;

   private final YoDouble maxX;
   private final YoDouble maxY;
   private final YoDouble deltaInside;
   private final YoBoolean constrainMaxAdjustment;

   private boolean parametersChanged = false;

   public YoConstraintOptimizerParameters(YoRegistry registry)
   {
      maxX = new YoDouble("maxX", registry);
      maxY = new YoDouble("maxY", registry);
      deltaInside = new YoDouble("deltaInside", registry);
      constrainMaxAdjustment = new YoBoolean("constrainMaxAdjustment", registry);

      maxX.set(defaultMaxX);
      maxY.set(defaultMaxY);
      deltaInside.set(defaultDeltaInside);
      constrainMaxAdjustment.set(defaultConstrainMaxAdjustment);

      maxX.addListener(v -> parametersChanged = true);
      maxY.addListener(v -> parametersChanged = true);
      deltaInside.addListener(v -> parametersChanged = true);
      constrainMaxAdjustment.addListener(v -> parametersChanged = true);
   }


   public boolean pollParametersChanged()
   {
      if (parametersChanged)
      {
         parametersChanged = false;
         return true;
      }
      return false;
   }

   public void setMaxX(double maxX)
   {
      this.maxX.set(maxX);
   }

   public void setMaxY(double maxY)
   {
      this.maxY.set(maxY);
   }

   public void setDesiredDistanceInside(double distanceInside)
   {
      this.deltaInside.set(distanceInside);
   }

   public void setConstrainMaxAdjustment(boolean constraintMaxAdjustment)
   {
      this.constrainMaxAdjustment.set(constraintMaxAdjustment);
   }

   public double getDesiredDistanceInside()
   {
      return deltaInside.getDoubleValue();
   }

   public double getMaxX()
   {
      return maxX.getDoubleValue();
   }

   public double getMaxY()
   {
      return maxY.getDoubleValue();
   }

   public boolean getConstrainMaxAdjustment()
   {
      return constrainMaxAdjustment.getBooleanValue();
   }
}
