package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.configurations.InertialParameterManagerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.parameterEstimation.InertialParameterManager;
import us.ihmc.yoVariables.registry.YoRegistry;

public class InertialParameterManagerFactory
{
   private final YoRegistry registry;
   private HighLevelHumanoidControllerToolbox toolbox;
   private InertialParameterManagerParameters parameters;

   InertialParameterManagerFactory(YoRegistry registry)
   {
      this.registry =  registry;
      this.toolbox = null;
      this.parameters = null;
   }

   public void setControllerToolbox(HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      this.toolbox = controllerToolbox;
   }

   public void setInertialParameterManagerProperties(InertialParameterManagerParameters properties)
   {
      this.parameters = properties;
   }

   public InertialParameterManager createInertialParameterManager()
   {
      return new InertialParameterManager(toolbox, parameters, registry);
   }
}
