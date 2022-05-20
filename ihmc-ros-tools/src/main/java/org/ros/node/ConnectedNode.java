/*
 * Copyright (C) 2011 Google Inc.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ros.node;

import org.ros.exception.ServiceNotFoundException;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.node.topic.TransportHints;

import java.net.URI;

/**
 * A node in the ROS graph that has successfully contacted the master.
 * <p>
 * A {@link ConnectedNode} serves as a factory for:
 * <ul>
 * <li>{@link Publisher}</li>
 * <li>{@link Subscriber}</li>
 * <li>{@link ServiceServer}</li>
 * <li>{@link ServiceClient}</li>
 * <li>{@link ParameterTree}</li>
 * </ul>
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public interface ConnectedNode extends Node {

  /**
   * In ROS, time can be wallclock (actual) or simulated, so it is important to
   * use {@link ConnectedNode#getCurrentTime()} instead of using the standard
   * Java routines for determining the current time.
   * 
   * @return the current time
   */
  Time getCurrentTime();

  /**
   * @param <T>
   *          the message type to create the publisher for
   * @param topicName
   *          the topic name, will be pushed down under this namespace unless
   *          '/' is prepended.
   * @param messageType
   *          the message data type (e.g. "std_msgs/String")
   * @return a {@link Publisher} for the specified topic
   */
  <T> Publisher<T> newPublisher(GraphName topicName, String messageType);

  /**
   * @see #newPublisher(GraphName, String)
   */
  <T> Publisher<T> newPublisher(String topicName, String messageType);

  /**
   * @param <T>
   *          the message type to create the {@link Subscriber} for
   * @param topicName
   *          the topic name to be subscribed to, this will be auto resolved
   * @param messageType
   *          the message data type (e.g. "std_msgs/String")
   * @return a {@link Subscriber} for the specified topic
   */
  <T> Subscriber<T> newSubscriber(GraphName topicName, String messageType);

  /**
   * @param <T>
   *          the message type to create the {@link Subscriber} for
   * @param topicName
   *          the topic name to be subscribed to, this will be auto resolved
   * @param messageType
   *          the message data type (e.g. "std_msgs/String")
   * @param transportHints
   *          the transport hints
   * @return a {@link Subscriber} for the specified topic
   */
  <T> Subscriber<T> newSubscriber(GraphName topicName, String messageType, TransportHints transportHints);

  /**
   * @see #newSubscriber(GraphName, String)
   */
  <T> Subscriber<T> newSubscriber(String topicName, String messageType);

  /**
   * @see #newSubscriber(GraphName, String, TransportHints)
   */
  <T> Subscriber<T> newSubscriber(String topicName, String messageType, TransportHints transportHints);

  /**
   * Create a new {@link ServiceServer}.
   * 
   * @param serviceName
   *          the name of the service
   * @param serviceType
   *          the type of the service (e.g. "rosjava_test_msgs/AddTwoInts")
   * @param serviceResponseBuilder
   *          called for every request to build a response
   * @return a {@link ServiceServer}
   */
  <T, S> ServiceServer<T, S> newServiceServer(GraphName serviceName, String serviceType,
      ServiceResponseBuilder<T, S> serviceResponseBuilder);

  /**
   * @see ConnectedNode#newServiceServer(GraphName, String,
   *      ServiceResponseBuilder)
   */
  <T, S> ServiceServer<T, S> newServiceServer(String serviceName, String serviceType,
      ServiceResponseBuilder<T, S> serviceResponseBuilder);

  /**
   * @param serviceName
   *          the {@link GraphName} of the {@link ServiceServer}
   * @return the {@link ServiceServer} with the given name or {@code null} if it
   *         does not exist
   */
  <T, S> ServiceServer<T, S> getServiceServer(GraphName serviceName);

  /**
   * @see ConnectedNode#getServiceServer(GraphName)
   */
  <T, S> ServiceServer<T, S> getServiceServer(String serviceName);

  /**
   * @param serviceName
   *          the {@link GraphName} of the service {@link URI} to lookup
   * @return the {@link URI} of the service or {@code null} if it does not exist
   */
  URI lookupServiceUri(GraphName serviceName);

  /**
   * @see #lookupServiceUri(GraphName)
   */
  URI lookupServiceUri(String serviceName);

  /**
   * Create a {@link ServiceClient}.
   * 
   * @param serviceName
   *          the name of the service
   * @param serviceType
   *          the type of the service (e.g. "rosjava_test_msgs/AddTwoInts")
   * @return a {@link ServiceClient}
   * @throws ServiceNotFoundException
   *           thrown if no matching service could be found
   */
  <T, S> ServiceClient<T, S> newServiceClient(GraphName serviceName, String serviceType)
      throws ServiceNotFoundException;

  /**
   * @see #newServiceClient(GraphName, String)
   */
  <T, S> ServiceClient<T, S> newServiceClient(String serviceName, String serviceType)
      throws ServiceNotFoundException;

  /**
   * Create a {@link ParameterTree} to query and set parameters on the ROS
   * parameter server.
   * 
   * @return {@link ParameterTree} with {@link NameResolver} in this namespace.
   */
  ParameterTree getParameterTree();
}