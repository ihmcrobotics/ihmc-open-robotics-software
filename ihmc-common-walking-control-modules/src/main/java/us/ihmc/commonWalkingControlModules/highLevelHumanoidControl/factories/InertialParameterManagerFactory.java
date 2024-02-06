package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.configurations.InertialEstimationParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.parameterEstimation.InertialParameterManager;
import us.ihmc.yoVariables.registry.YoRegistry;

public class InertialParameterManagerFactory
{
   private final YoRegistry registry;
   private HighLevelHumanoidControllerToolbox toolbox;
   private InertialEstimationParameters parameters;

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

   public void setInertialParameterManagerProperties(InertialEstimationParameters properties)
   {
      this.parameters = properties;
   }

   public InertialParameterManager createInertialParameterManager()
   {
      return new InertialParameterManager(toolbox, parameters, registry);
   }
}
