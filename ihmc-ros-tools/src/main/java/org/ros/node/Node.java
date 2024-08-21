package org.ros.node;

import java.net.URI;
import java.util.concurrent.ScheduledExecutorService;
//import org.apache.commons.logging.Log;
//import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageFactory;
//import org.ros.message.MessageSerializationFactory;
import org.ros.namespace.GraphName;
//import org.ros.namespace.NodeNameResolver;

public interface Node {
   GraphName getName();

   GraphName resolveName(GraphName var1);

   GraphName resolveName(String var1);


   URI getUri();

   URI getMasterUri();

//   Log getLog();


   MessageFactory getTopicMessageFactory();

   MessageFactory getServiceResponseMessageFactory();

   MessageFactory getServiceRequestMessageFactory();

   void addListener(NodeListener var1);

   ScheduledExecutorService getScheduledExecutorService();


   void shutdown();
}