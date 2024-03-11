package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.configurations.InertialEstimationParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.parameterEstimation.InertialParameterManager;
import us.ihmc.yoVariables.registry.YoRegistry;

public class InertialParameterManagerFactory
{
   private final YoRegistry registry;
   private final InertialEstimationParameters parameters;
   private HighLevelHumanoidControllerToolbox toolbox;

   InertialParameterManagerFactory(InertialEstimationParameters parameters, YoRegistry registry)
   {
      this.registry =  registry;
      this.parameters = parameters;
      this.toolbox = null;
   }

   public void setControllerToolbox(HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      this.toolbox = controllerToolbox;
   }


   public InertialParameterManager createInertialParameterManager()
   {
      return new InertialParameterManager(toolbox, parameters, registry);
   }
}
