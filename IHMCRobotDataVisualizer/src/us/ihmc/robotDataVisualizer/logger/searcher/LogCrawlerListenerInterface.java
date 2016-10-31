package us.ihmc.robotDataVisualizer.logger.searcher;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

public interface LogCrawlerListenerInterface
{

   public abstract YoVariable<?>[] getYovariablesToUpdate(YoVariableRegistry registry);
   public abstract void update(LogCrawler logSearcher, double timestamp);
   public abstract void onFinish();
}
