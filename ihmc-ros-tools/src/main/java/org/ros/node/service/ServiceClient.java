package org.ros.node.service;

import java.net.URI;
import org.ros.namespace.GraphName;

public interface ServiceClient<T, S> {
   void connect(URI var1);

   boolean isConnected();

   void call(T var1, ServiceResponseListener<S> var2);

   GraphName getName();

   void shutdown();

   T newMessage();
}