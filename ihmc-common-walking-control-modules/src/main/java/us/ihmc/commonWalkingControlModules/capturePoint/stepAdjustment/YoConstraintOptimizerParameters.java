package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoConstraintOptimizerParameters implements ConstraintOptimizerParametersReadOnly
{
   private final YoDouble maxX;
   private final YoDouble maxY;
   private final YoDouble deltaInside;
   private final YoBoolean constrainMaxAdjustment;
   private final YoBoolean shouldPerformOptimization;

   private boolean parametersChanged = false;

   public YoConstraintOptimizerParameters(YoRegistry registry)
   {
      this(null, registry);
   }

   public YoConstraintOptimizerParameters(ConstraintOptimizerParametersReadOnly parameters, YoRegistry registry)
   {
      maxX = new YoDouble("maxX", registry);
      maxY = new YoDouble("maxY", registry);
      deltaInside = new YoDouble("deltaInside", registry);
      constrainMaxAdjustment = new YoBoolean("constrainMaxAdjustment", registry);
      shouldPerformOptimization = new YoBoolean("shouldPerformOptimization", registry);

      if (parameters == null)
         parameters = new ConstraintOptimizerParameters();

      maxX.set(parameters.getMaxX());
      maxY.set(parameters.getMaxY());
      deltaInside.set(parameters.getDesiredDistanceInside());
      constrainMaxAdjustment.set(parameters.getConstrainMaxAdjustment());
      shouldPerformOptimization.set(parameters.shouldPerformOptimization());

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

   public void setShouldPerformOptimization(boolean shouldPerformOptimization)
   {
      this.shouldPerformOptimization.set(shouldPerformOptimization);
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

   public boolean shouldPerformOptimization()
   {
      return shouldPerformOptimization.getValue();
   }
}
