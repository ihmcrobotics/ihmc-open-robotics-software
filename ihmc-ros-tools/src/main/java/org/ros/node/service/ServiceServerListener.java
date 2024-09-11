package org.ros.node.service;

//import org.ros.internal.node.RegistrantListener;

public interface ServiceServerListener<T, S> {
   void onShutdown(ServiceServer<T, S> var1);
}