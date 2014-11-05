package us.ihmc.darpaRoboticsChallenge.logProcessor;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public interface LogDataProcessorFunction
{
   public abstract void processDataAtControllerRate();
   public abstract void processDataAtStateEstimatorRate();
   
   public abstract YoVariableRegistry getYoVariableRegistry();
   public abstract YoGraphicsListRegistry getYoGraphicsListRegistry();
}
