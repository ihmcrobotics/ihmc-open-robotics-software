package us.ihmc.quadrupedPlanning.pathPlanning;

import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedConstantAccelerationBodyPathPlanner
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble maxForwardAcceleration = new YoDouble("maxForwardAcceleration", registry);
   private final YoDouble maxLateralAcceleration = new YoDouble("maxLateralAcceleration", registry);
   private final YoDouble maxYawAcceleration = new YoDouble("maxYawAcceleration", registry);

   private BodyPathPlan bodyPathPlan;

   public QuadrupedConstantAccelerationBodyPathPlanner()
   {}

   public void setBodyPathPlan(BodyPathPlan bodyPathPlan)
   {
      this.bodyPathPlan = bodyPathPlan;
   }
}
