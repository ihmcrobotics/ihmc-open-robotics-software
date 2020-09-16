package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoCoPTrajectoryPolygonParameters implements CoPTrajectoryPolygonParametersReadOnly
{
   private final DoubleProvider stepLengthPlanToeOff;

   public YoCoPTrajectoryPolygonParameters(CoPTrajectoryPolygonParametersReadOnly defaultParameters, YoRegistry registry)
   {
      stepLengthPlanToeOff = new DoubleParameter("stepLengthToPlanToeOff", registry, defaultParameters.getStepLengthToPlanToeOff());
   }

   public double getStepLengthToPlanToeOff()
   {
      return stepLengthPlanToeOff.getValue();
   }
}
