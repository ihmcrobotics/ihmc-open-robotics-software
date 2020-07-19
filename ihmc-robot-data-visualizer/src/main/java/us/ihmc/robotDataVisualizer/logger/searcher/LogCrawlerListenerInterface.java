package us.ihmc.robotDataVisualizer.logger.searcher;

import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public interface LogCrawlerListenerInterface
{

   public abstract YoVariable[] getYovariablesToUpdate(YoRegistry registry, RobotDescription robotDescription);
   public abstract void update(double timestamp);
   public abstract void onStart(LogCrawler logSearcher, SpecificLogVariableUpdater logVariableUpdater);
   public abstract void onFinish();
}
