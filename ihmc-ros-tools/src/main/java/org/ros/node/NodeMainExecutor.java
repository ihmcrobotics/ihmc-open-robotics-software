package org.ros.node;

import java.util.Collection;
import java.util.concurrent.ScheduledExecutorService;

public interface NodeMainExecutor {
   ScheduledExecutorService getScheduledExecutorService();

   void execute(NodeMain var1, NodeConfiguration var2, Collection<NodeListener> var3);

   void execute(NodeMain var1, NodeConfiguration var2);

   void shutdownNodeMain(NodeMain var1);

   void shutdown();
}