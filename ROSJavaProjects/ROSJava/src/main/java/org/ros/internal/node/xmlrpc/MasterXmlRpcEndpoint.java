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

package org.ros.internal.node.xmlrpc;

import java.util.List;

/**
 * An XML-RPC endpoint description of a ROS master.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public interface MasterXmlRpcEndpoint extends XmlRpcEndpoint {

  /**
   * Get the PID for the master process.
   * 
   * @param callerId
   *          ROS caller ID
   * @return The pid of the process.
   */
  List<Object> getPid(String callerId);

  /**
   * Register the caller as a provider of the specified service.
   * 
   * @param callerId
   *          ROS caller ID
   * @param service
   *          Fully-qualified name of service
   * @param serviceApi
   *          XML-RPC URI of caller node
   * @param callerApi
   * @return ignore
   */
  List<Object>
      registerService(String callerId, String service, String serviceApi, String callerApi);

  /**
   * Unregister the caller as a provider of the specified service.
   * 
   * @param callerId
   *          ROS caller ID
   * @param service
   *          Fully-qualified name of service
   * @param serviceApi
   *          API URI of service to unregister. Unregistration will only occur
   *          if current registration matches.
   * @return Number of unregistrations (either 0 or 1). If this is zero it means
   *         that the caller was not registered as a service provider. The call
   *         still succeeds as the intended final state is reached.
   */
  List<Object> unregisterService(String callerId, String service, String serviceApi);

  /**
   * Subscribe the caller to the specified topic. In addition to receiving a
   * list of current publishers, the subscriber will also receive notifications
   * of new publishers via the publisherUpdate API.
   * 
   * 
   * @param callerId
   *          ROS caller ID
   * @param topicName
   *          Fully-qualified name of topic
   * @param topicType
   *          topic type, must be a package-resource name, i.e. the .msg name
   * @param callerApi
   *          API URI of subscriber to register. Will be used for new publisher
   *          notifications
   * @return publishers as a list of XMLRPC API URIs for nodes currently
   *         publishing the specified topic
   */
  List<Object> registerSubscriber(String callerId, String topicName, String topicType,
      String callerApi);

  /**
   * Unregister the caller as a publisher of the topic.
   * 
   * @param callerId
   *          ROS caller ID
   * @param topicName
   *          Fully-qualified name of topic.
   * @param callerApi
   *          API URI of service to unregister. Unregistration will only occur
   *          if current registration matches.
   * @return If numUnsubscribed is zero it means that the caller was not
   *         registered as a subscriber. The call still succeeds as the intended
   *         final state is reached.
   */
  List<Object> unregisterSubscriber(String callerId, String topicName, String callerApi);

  /**
   * Register the caller as a publisher the topic.
   * 
   * @param callerId
   *          ROS caller ID
   * @param topicName
   *          fully-qualified name of topic to register
   * @param topicType
   *          topic type, must be a package-resource name, i.e. the .msg name.
   * @param callerApi
   *          API URI of publisher to register
   * @return list of current subscribers of topic in the form of XML-RPC URIs
   */
  List<Object> registerPublisher(String callerId, String topicName, String topicType,
      String callerApi);

  /**
   * Unregister the caller as a publisher of the topic.
   * 
   * @param callerId
   *          ROS caller ID
   * @param topicName
   *          Fully-qualified name of topic.
   * @param callerApi
   *          API URI of publisher to unregister. Unregistration will only occur
   *          if current registration matches.
   * @return If numUnsubscribed is zero it means that the caller was not
   *         registered as a subscriber. The call still succeeds as the intended
   *         final state is reached.
   */
  List<Object> unregisterPublisher(String callerId, String topicName, String callerApi);

  /**
   * Get the XML-RPC URI of the node with the associated name/caller_id. This
   * API is for looking information about publishers and subscribers. Use
   * lookupService instead to lookup ROS-RPC URIs.
   * 
   * @param callerId
   *          ROS caller ID
   * @param nodeName
   *          Name of node to lookup
   * @return URI of the node
   */
  List<Object> lookupNode(String callerId, String nodeName);

  /**
   * Get list of topics that can be subscribed to. This does not return topics
   * that have no publishers. See getSystemState() to get more comprehensive
   * list.
   * 
   * @param callerId
   *          ROS caller ID
   * @param subgraph
   *          Restrict topic names to match within the specified subgraph.
   *          Subgraph namespace is resolved relative to the caller's namespace.
   *          Use empty string to specify all names.
   * @return Topics is in list representation [[topic, message type], [topic,
   *         message type] ...]
   */
  List<Object> getPublishedTopics(String callerId, String subgraph);

  /**
   * Get a list of all topic types.
   * 
   * @param callerId
   *          ROS caller ID
   * @return The types are in the list representation [[topic, message type],
   *         [topic, message type] ...]
   */
  List<Object> getTopicTypes(String callerId);

  /**
   * Retrieve list representation of system state (i.e. publishers, subscribers,
   * and services).
   * 
   * @param callerId
   *          ROS caller ID
   * @return System state is in list representation [publishers, subscribers,
   *         services] publishers is of the form [ [topic1,
   *         [topic1Publisher1...topic1PublisherN]] ... ] subscribers is of the
   *         form [ [topic1, [topic1Subscriber1...topic1SubscriberN]] ... ]
   *         services is of the form [ [service1,
   *         [service1Provider1...service1ProviderN]] ... ]
   */
  List<Object> getSystemState(String callerId);

  /**
   * Get the URI of the the master.
   * 
   * @param callerId
   *          ROS caller ID
   * @return URI of the the master
   */
  List<Object> getUri(String callerId);

  /**
   * Lookup all provider of a particular service.
   * 
   * @param callerId
   *          ROS caller ID
   * @param service
   *          Fully-qualified name of service
   * @return service URL is provides address and port of the service. Fails if
   *         there is no provider.
   */
  List<Object> lookupService(String callerId, String service);

}
