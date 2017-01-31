package us.ihmc.avatar.logProcessor;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public interface LogDataProcessorFunction
{
   public abstract void processDataAtControllerRate();
   public abstract void processDataAtStateEstimatorRate();
   
   public abstract YoVariableRegistry getYoVariableRegistry();
   public abstract YoGraphicsListRegistry getYoGraphicsListRegistry();
}
