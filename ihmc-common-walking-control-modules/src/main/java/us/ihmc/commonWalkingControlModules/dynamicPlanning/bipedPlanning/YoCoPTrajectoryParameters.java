package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoCoPTrajectoryParameters implements CoPTrajectoryParametersReadOnly
{
   private final DoubleProvider stepLengthPlanToeOff;

   public YoCoPTrajectoryParameters(CoPTrajectoryParametersReadOnly defaultParameters, YoRegistry registry)
   {
      stepLengthPlanToeOff = new DoubleParameter("stepLengthToPlanToeOff", registry, defaultParameters.getStepLengthToPlanToeOff());
   }

   public double getStepLengthToPlanToeOff()
   {
      return stepLengthPlanToeOff.getValue();
   }
}
