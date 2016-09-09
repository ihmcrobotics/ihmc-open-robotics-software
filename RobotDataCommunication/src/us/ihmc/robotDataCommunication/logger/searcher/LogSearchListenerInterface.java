package us.ihmc.robotDataCommunication.logger.searcher;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

public interface LogSearchListenerInterface
{

   YoVariable<?>[] getYovariablesToUpdate(YoVariableRegistry registry);

   void update(LogSearcher logSearcher, double timestamp);

}
