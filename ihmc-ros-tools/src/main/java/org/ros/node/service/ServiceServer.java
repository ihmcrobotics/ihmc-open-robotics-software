package org.ros.node.service;

import java.net.URI;
import org.ros.namespace.GraphName;

public interface ServiceServer<T, S> {
   GraphName getName();

   URI getUri();

   void shutdown();

   void addListener(ServiceServerListener<T, S> var1);
}