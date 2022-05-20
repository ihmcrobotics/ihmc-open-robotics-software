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

import org.apache.commons.logging.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.internal.node.xmlrpc.MasterXmlRpcEndpoint;
import org.ros.message.MessageFactory;
import org.ros.message.MessageSerializationFactory;
import org.ros.namespace.GraphName;
import org.ros.namespace.NodeNameResolver;

import java.net.URI;
import java.util.concurrent.ScheduledExecutorService;

/**
 * A node in the ROS graph.
 * 
 * @author khughes@google.com (Keith M. Hughes)
 */
public interface Node {

  /**
   * @return the fully resolved name of this {@link Node}, e.g. "/foo/bar/boop"
   */
  GraphName getName();

  /**
   * Resolve the given name, using ROS conventions, into a full ROS namespace
   * name. Will be relative to the current namespace unless the name is global.
   * 
   * @param name
   *          the name to resolve
   * @return fully resolved ros namespace name
   */
  GraphName resolveName(GraphName name);

  /**
   * @see #resolveName(GraphName)
   */
  GraphName resolveName(String name);

  /**
   * @return {@link NodeNameResolver} for this namespace
   */
  NodeNameResolver getResolver();

  /**
   * @return the {@link URI} of this {@link Node}
   */
  URI getUri();

  /**
   * @return {@link URI} of {@link MasterXmlRpcEndpoint} that this node is
   *         attached to.
   */
  URI getMasterUri();

  /**
   * @return Logger for this node, which will also perform logging to /rosout.
   */
  Log getLog();

  /**
   * @return the {@link MessageSerializationFactory} used by this node
   */
  MessageSerializationFactory getMessageSerializationFactory();

  /**
   * @return the {@link MessageFactory} used by this node
   */
  MessageFactory getTopicMessageFactory();

  /**
   * @return the {@link MessageFactory} used by this node for service responses
   */
  MessageFactory getServiceResponseMessageFactory();

  /**
   * @return the {@link MessageFactory} used by this node for service requests
   */
  MessageFactory getServiceRequestMessageFactory();

  /**
   * Add a new {@link NodeListener} to the {@link Node}.
   * 
   * @param listener
   *          the {@link NodeListener} to add
   */
  void addListener(NodeListener listener);

  /**
   * @return the {@link ScheduledExecutorService} that this {@link Node} uses
   */
  ScheduledExecutorService getScheduledExecutorService();

  /**
   * Executes a {@link CancellableLoop} using the {@link Node}'s
   * {@link ScheduledExecutorService}. The {@link CancellableLoop} will be
   * canceled when the {@link Node} starts shutting down.
   * 
   * <p>
   * Any blocking calls executed in the provided {@link CancellableLoop} can
   * potentially delay {@link Node} shutdown and should be avoided.
   * 
   * @param cancellableLoop
   *          the {@link CancellableLoop} to execute
   */
  void executeCancellableLoop(CancellableLoop cancellableLoop);

  /**
   * Shut the node down.
   */
  void shutdown();

  /**
   * Stops and Clears node listeners.
   */

  void removeListeners();
}
