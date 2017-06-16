package us.ihmc.exampleSimulations.flyballGovernor;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;


public class FlyballGovernorCommonControllerParameters
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoDouble k_feedback = new YoDouble("k_feedback", registry);
   private final YoDouble q_d_cylinder_z = new YoDouble("q_d_cylinder_z", registry);
   
   public FlyballGovernorCommonControllerParameters()
   {
      initialize();
   }
   
   public void initialize()
   {
      k_feedback.set(0.2);
      q_d_cylinder_z.set(0.1);
   }

   public YoDouble getK_feedback()
   {
      return k_feedback;
   }

   public YoDouble getQ_d_cylinder_z()
   {
      return q_d_cylinder_z;
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }
}
