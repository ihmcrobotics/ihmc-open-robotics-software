package org.ros.node;

import java.net.URI;
//import org.ros.exception.ServiceNotFoundException;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

public interface ConnectedNode extends Node {
   Time getCurrentTime();

   <T> Publisher<T> newPublisher(GraphName var1, String var2);

   <T> Publisher<T> newPublisher(String var1, String var2);

   <T> Subscriber<T> newSubscriber(GraphName var1, String var2);

   <T> Subscriber<T> newSubscriber(String var1, String var2);

   <T, S> ServiceServer<T, S> newServiceServer(GraphName var1, String var2, ServiceResponseBuilder<T, S> var3);

   <T, S> ServiceServer<T, S> newServiceServer(String var1, String var2, ServiceResponseBuilder<T, S> var3);

   <T, S> ServiceServer<T, S> getServiceServer(GraphName var1);

   <T, S> ServiceServer<T, S> getServiceServer(String var1);

   URI lookupServiceUri(GraphName var1);

   URI lookupServiceUri(String var1);

//   <T, S> ServiceClient<T, S> newServiceClient(GraphName var1, String var2) throws ServiceNotFoundException;
//
//   <T, S> ServiceClient<T, S> newServiceClient(String var1, String var2) throws ServiceNotFoundException;

   ParameterTree getParameterTree();
}